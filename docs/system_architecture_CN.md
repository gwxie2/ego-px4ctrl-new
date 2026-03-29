# 系统架构设计文档（中文）

> 适用范围：`cleanroom_ws` —— 基于 ROS Noetic + PX4 SITL + EGO-Planner 的无人机自主导航工作区

---

## 1. 总体架构

本系统采用**分层解耦**的设计思想，从下至上分为五个层次：

```
╔══════════════════════════════════════════════════════════╗
║                  【任务/规划层】                           ║
║      EGO-Planner（路径搜索 + B样条优化 + FSM）             ║
╠══════════════════════════════════════════════════════════╣
║                  【控制层】                               ║
║          px4ctrl（级联PID + 推力模型 + 姿态控制）           ║
╠══════════════════════════════════════════════════════════╣
║                  【适配/胶水层】                           ║
║  clean_uav_core（里程计桥接 + 触发器 + launch 组织）        ║
╠══════════════════════════════════════════════════════════╣
║                  【通信中间件层】                          ║
║              MAVROS（ROS ↔ MAVLink 桥接）                 ║
╠══════════════════════════════════════════════════════════╣
║                  【物理/仿真层】                           ║
║            Gazebo + PX4 SITL（飞控固件模拟）              ║
╚══════════════════════════════════════════════════════════╝
```

---

## 2. 数据流向图

### 2.1 单机（Phase 1）完整数据流

```
Gazebo 世界
    │
    │ /gazebo/model_states（位姿+速度）
    ▼
truth_odom_adapter.py
    │
    │ /truth_odom（nav_msgs/Odometry）
    │ /truth_pose（geometry_msgs/PoseStamped）
    ├──────────────────────────────┐
    ▼                              ▼
ego_planner_node              px4ctrl_node
    │                              ▲
    │ /planning/bspline             │ ~cmd（quadrotor_msgs/PositionCommand）
    ▼                              │
traj_server ────────────────────►─┘
    
px4ctrl_node
    │
    │ /mavros/setpoint_raw/attitude（mavros_msgs/AttitudeTarget）
    ▼
MAVROS
    │
    │ MAVLink（SET_ATTITUDE_TARGET）
    ▼
PX4 SITL
    │
    ▼
Gazebo（接收推力/力矩，更新物理状态）
```

### 2.2 进入点与出口点汇总

| 话题/服务 | 方向 | 消息类型 | 说明 |
|-----------|------|----------|------|
| `/gazebo/model_states` | 仿真→适配器 | `gazebo_msgs/ModelStates` | 真值位姿来源 |
| `/truth_odom` | 适配器→规划/控制 | `nav_msgs/Odometry` | 统一里程计话题 |
| `/move_base_simple/goal` | 外部→规划器 | `geometry_msgs/PoseStamped` | 目标点输入 |
| `/traj_start_trigger` | 触发器→规划器 | `geometry_msgs/PoseStamped` | 规划启动门控 |
| `~cmd`（/px4ctrl/cmd） | 规划→控制 | `quadrotor_msgs/PositionCommand` | 位置/速度/加速度指令 |
| `~takeoff_land` | 触发器→控制 | `quadrotor_msgs/TakeoffLand` | 无RC自动起降 |
| `/mavros/setpoint_raw/attitude` | 控制→MAVROS | `mavros_msgs/AttitudeTarget` | 姿态+推力指令 |
| `/mavros/state` | MAVROS→控制 | `mavros_msgs/State` | Offboard/Armed状态 |

---

## 3. 各模块详细设计

### 3.1 px4ctrl（位置控制器）

**源码位置**：`~/XTDrone/px4ctrl/src/`（通过符号链接引入）

#### 核心组件

| 文件 | 职责 |
|------|------|
| `px4ctrl_node.cpp` | ROS 节点入口，订阅话题，启动 FSM 主循环 |
| `PX4CtrlFSM.h/.cpp` | 有限状态机，管理五个飞行状态 |
| `controller.h/.cpp` | 级联 PID + 推力模型，生成姿态指令 |
| `input.h/.cpp` | 所有输入数据的封装结构体 |
| `PX4CtrlParam.h/.cpp` | YAML 参数加载与解析 |

#### 有限状态机（FSM）五态模型

```
                  ┌─────────────────┐
     上电/无RC     │   MANUAL_CTRL   │ ← RC 控制模式（FSM 不介入）
                  └────────┬────────┘
                           │ 解锁进 Offboard
                           ▼
                  ┌─────────────────┐
                  │   AUTO_TAKEOFF  │ ← 收到 TakeoffLand.TAKEOFF
                  └────────┬────────┘
                           │ 到达目标高度
                           ▼
                  ┌─────────────────┐
    无指令悬停 ←→  │   AUTO_HOVER    │ ← 等待 PositionCommand
                  └────────┬────────┘
                           │ 收到 PositionCommand
                           ▼
                  ┌─────────────────┐
                  │    CMD_CTRL     │ ← 执行轨迹跟踪（主工作状态）
                  └────────┬────────┘
                           │ 收到 TakeoffLand.LAND
                           ▼
                  ┌─────────────────┐
                  │    AUTO_LAND    │ ← 下降并检测落地
                  └─────────────────┘
```

#### 控制算法：级联 PID

```
目标位置 p_des
    │
    │ 位置环（Kp0/Kp1/Kp2）
    ▼
目标速度 v_des
    │
    │ 速度环（Kv0/Kv1/Kv2）
    ▼
目标加速度 a_des（含重力补偿）
    │
    │ 推力映射（hover_percentage 或精确推力模型）
    ▼
归一化推力 thrust + 目标朝向 q_des
    │
    ▼
mavros_msgs/AttitudeTarget → MAVROS → PX4
```

**推力模型（精确模式）**：

$$F = K_1 \cdot V_{bat}^{K_2} \cdot (K_3 \cdot u^2 + (1-K_3) \cdot u)$$

其中 $u \in [0,1]$ 为归一化推力信号，$V_{bat}$ 为电池电压。

#### 关键参数（`ctrl_param_fpv.yaml`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mass` | 1.2 kg | 机体质量 |
| `hover_percentage` | 0.58 | 悬停油门百分比 |
| `Kp0/1/2` | 1.5 | 位置 PID 比例增益（x/y/z）|
| `Kv0/1/2` | 1.5 | 速度 PID 比例增益（x/y/z）|
| `no_RC` | false | 是否无遥控器模式 |
| `takeoff_height` | 1.0 m | 自动起飞目标高度 |

---

### 3.2 EGO-Planner（轨迹规划器）

**源码位置**：`~/XTDrone/motion_planning/3d/ego_planner/`（通过符号链接引入）

#### 子包依赖关系

```
ego_planner（plan_manage）
    ├── bspline_opt      ← B样条轨迹优化（梯度下降/LBFGS）
    ├── path_searching   ← 初始路径搜索（动态 A*）
    ├── plan_env         ← 占据格地图（GridMap）
    ├── traj_utils       ← B样条工具库 + traj_server
    └── uav_utils        ← 数学工具（四元数/欧拉角）
```

#### 核心节点与功能

| 节点 | 包 | 功能 |
|------|-----|------|
| `ego_planner_node` | `ego_planner` | 主规划 FSM + 在线重规划 |
| `traj_server` | `traj_utils` | B样条轨迹求值，发布 `PositionCommand` |

#### EGO-Planner FSM（规划状态机）

```
INIT
  │ 收到里程计
  ▼
WAIT_TARGET
  │ 收到目标点（或 /traj_start_trigger）
  ▼
GEN_NEW_TRAJ ──────────────────────────► REPLAN_TRAJ
  │                                           │
  │ 规划成功                                   │ 检测到障碍/偏差
  ▼                                           │
EXEC_TRAJ ◄─────────────────────────────────┘
  │ 到达目标
  ▼
EMERGENCY_STOP（失败保护）
```

#### 关键参数（`phase1_minimal_demo.launch` 内嵌）

| 参数 | 说明 |
|------|------|
| `fsm/realworld_experiment` | `true` 时需等待 `/traj_start_trigger` 才开始规划 |
| `fsm/flight_type` | 2=预设航点模式 |
| `fsm/planning_horizon` | 规划水平距离（默认 7.5 m）|
| `grid_map/resolution` | 地图分辨率（默认 0.1 m）|
| `optimization/max_vel` | 最大速度约束 |
| `optimization/max_acc` | 最大加速度约束 |

---

### 3.3 truth_odom_adapter（里程计桥接器）

**源码**：`src/clean_uav_core/scripts/truth_odom_adapter.py`

**功能**：将 Gazebo 仿真器发布的原始模型状态转换为标准 ROS 里程计消息。

```
/gazebo/model_states（所有模型的位姿+速度数组）
    │
    │ 按 model_name 检索索引
    ▼
nav_msgs/Odometry（/truth_odom）+ geometry_msgs/PoseStamped（/truth_pose）
```

- 首次收到消息时自动查找模型索引，之后直接按索引访问（零额外查找开销）
- 支持 `~model_name` 参数指定 Gazebo 模型名称（Phase 1: `iris`，Phase 2: `iris_0`/`iris_1`）

---

### 3.4 clean_uav_core（胶水包）

**职责**：Launch 文件组织、脚本工具、参数覆盖。不包含独立发布/订阅逻辑的 C++ 节点。

详见 [目录结构文档](directory_structure_CN.md)。

---

## 4. 多机（Phase 2）架构扩展

Phase 2 在 Phase 1 基础上增加命名空间隔离：

```
┌────────────────────────────────────────────────────────────────┐
│                        Phase 2 话题命名空间                      │
│                                                                 │
│  /uav0/truth_odom          /uav1/truth_odom                    │
│  /uav0/ego_planner_node    /uav1/ego_planner_node              │
│  /uav0/position_cmd        /uav1/position_cmd                  │
│  /uav0/px4ctrl/...         /uav1/px4ctrl/...                   │
│                                                                 │
│  /iris_0/mavros/...        /iris_1/mavros/...  （MAVROS原始）   │
└────────────────────────────────────────────────────────────────┘
```

**关键设计决策**：
- `phase1_minimal_demo.launch` 将所有 mavros 相关话题/服务都参数化（`mavros_state_topic`、`mavros_attitude_cmd_topic` 等），使得多机场景下可将其重映射到 `/irisN/mavros/...`
- `px4_param_bootstrap.py` 为每架机独立设置 `COM_RCL_EXCEPT=4`（禁用 RC 丢失保护）
- `dual_traj_start_trigger.py` 同步触发两架机的规划器，避免先后出发造成的碰撞风险

---

## 5. 启动时序（单机 Phase 1）

```
t=0s    Gazebo + PX4 SITL 启动（phase1_px4_sim.launch）
t=~3s   MAVROS 建立 MAVLink 连接
t=~5s   px4_param_bootstrap 设置 COM_RCL_EXCEPT=4
t=~5s   truth_odom_adapter 锁定 iris 模型，开始发布 /truth_odom
t=~5s   ego_planner_node 收到里程计，进入 WAIT_TARGET 状态
t=10s   takeoff_land_trigger 发布 TakeoffLand.TAKEOFF
t=~11s  px4ctrl 请求 Offboard 模式并解锁，电机怠速（2.8s）
t=~14s  无人机开始爬升至 1.0m
t=~15s  traj_start_trigger 发布 /traj_start_trigger
t=~15s  ego_planner 开始规划，traj_server 发布 PositionCommand
t=~15s  px4ctrl 进入 CMD_CTRL 状态，开始轨迹跟踪
```

---

## 6. 核心消息类型说明

### quadrotor_msgs/PositionCommand（自定义）

```
Header header
geometry_msgs/Point position       # 目标位置 (m)
geometry_msgs/Vector3 velocity     # 前馈速度 (m/s)
geometry_msgs/Vector3 acceleration # 前馈加速度 (m/s²)
geometry_msgs/Vector3 jerk         # 前馈加加速度（本地副本新增）
float64 yaw                        # 目标偏航角 (rad)
float64 yaw_dot                    # 偏航角速率 (rad/s)
```

### quadrotor_msgs/TakeoffLand（本地副本新增）

```
Header header
uint8 TAKEOFF=1
uint8 LAND=2
uint8 takeoff_land_cmd
```

---

## 7. 设计原则与约束

1. **里程计单一来源原则**：所有节点（EGO 和 px4ctrl）消费同一个 `/truth_odom` 话题，避免时间戳不一致问题。
2. **No-RC 启动门控**：`px4ctrl` 在 `no_RC=true` 时完全禁用 RC 输入订阅，由 `TakeoffLand` 消息触发起降，防止误操作。
3. **规划启动门控**：`fsm/realworld_experiment=true` + `/traj_start_trigger` 确保无人机在完成起飞悬停稳定后再开始执行轨迹，避免低高度规划带来的安全问题。
4. **quadrotor_msgs 本地化**：EGO 原始版本缺少 `TakeoffLand`、`Px4ctrlDebug`、`jerk`，必须使用 `src/quadrotor_msgs/` 本地副本，**禁止替换为 apt 包或 EGO 原版**。
5. **环境隔离**：必须 source `tools/source_phase1_env.sh` 才能让 catkin 找到 PX4 固件包路径，否则 `phase1_px4_sim.launch` 无法解析。
