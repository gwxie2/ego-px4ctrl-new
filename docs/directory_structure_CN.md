# 工程目录结构详解（中文）

> 本文档重点解析 `cleanroom_ws` 工作区的目录组织，尤其针对核心胶水包 `clean_uav_core`。

---

## 1. 工作区顶层结构

```
cleanroom_ws/
├── .github/
│   └── copilot-instructions.md     # AI 编程助手工作区指令
├── src/                            # Catkin 源码空间（核心）
├── build/                          # 构建产物（catkin_make 生成，勿手动修改）
├── devel/                          # 开发空间，含 setup.bash 等（catkin_make 生成）
├── docs/                           # 技术文档集合（中文）
├── logs/                           # 实施日志
├── tools/                          # 工具脚本
├── eeprom/                         # PX4 SITL 参数持久化目录
├── log/                            # ROS 日志（运行时生成）
└── README.md
```

---

## 2. src/ 源码空间

```
src/
├── CMakeLists.txt          # → /opt/ros/noetic/share/catkin/cmake/toplevel.cmake（符号链接）
│
├── clean_uav_core/         # ★ 本地胶水包（重点）
├── quadrotor_msgs/         # ★ 本地消息包（定制版）
│
│   ── 以下均为符号链接 ──
├── px4ctrl    → ~/XTDrone/px4ctrl
├── ego_planner → ~/XTDrone/motion_planning/3d/ego_planner/plan_manage
├── bspline_opt → ~/XTDrone/motion_planning/3d/ego_planner/bspline_opt
├── path_searching → ~/XTDrone/motion_planning/3d/ego_planner/path_searching
├── plan_env    → ~/XTDrone/motion_planning/3d/ego_planner/plan_env
├── traj_utils  → ~/XTDrone/motion_planning/3d/ego_planner/traj_utils
├── uav_utils   → ~/XTDrone/motion_planning/3d/ego_planner/Utils/uav_utils
└── cmake_utils → ~/XTDrone/motion_planning/3d/ego_planner/Utils/cmake_utils
```

> **为什么用符号链接？**
> 避免代码二次拷贝，方便直接修改原始仓库代码后在 cleanroom_ws 内即可验证。但 `quadrotor_msgs` 必须是带定制的本地副本。

---

## 3. clean_uav_core/ 核心胶水包（完整解析）

```
src/clean_uav_core/
├── CMakeLists.txt          # 仅含 catkin_python_setup() 和脚本安装
├── package.xml             # 声明依赖：ego_planner, px4ctrl, quadrotor_msgs 等
├── README.md               # 包级说明
│
├── config/                 # 参数覆盖配置
│   └── phase1_px4ctrl_no_rc.yaml   # px4ctrl no-RC 模式参数（覆盖默认值）
│
├── launch/                 # 所有 ROS launch 文件
│   ├── phase1_fullstack.launch         # Phase1 全栈入口（含 Gazebo+算法）
│   ├── phase1_px4_sim.launch           # Phase1 仿真层（Gazebo+PX4+MAVROS）
│   ├── phase1_minimal_demo.launch      # Phase1 算法栈核心（★ 最重要）
│   ├── phase1_algo_stack.launch        # Phase1 仅算法层（无仿真）
│   ├── phase1_px4ctrl_stack.launch     # Phase1 仅控制层（无仿真无规划）
│   ├── phase1_truth_odom.launch        # 里程计适配器单独入口
│   ├── phase1_manual_goal_stack.launch # 手动目标点模式（RViz交互）
│   ├── phase1_preset_mission_stack.launch  # 预设航点任务模式
│   ├── phase2_px4_multi_sim.launch     # Phase2 双机仿真层
│   └── phase2_dual_uav_stack.launch    # Phase2 双机算法+控制栈
│
├── scripts/                # Python 辅助节点
│   ├── truth_odom_adapter.py       # 里程计桥接（Gazebo→ROS标准格式）
│   ├── takeoff_land_trigger.py     # 自动起降指令发布器
│   ├── traj_start_trigger.py       # 单机规划触发器
│   ├── dual_traj_start_trigger.py  # 双机同步规划触发器
│   ├── goal_point_publisher.py     # 固定目标点发布器
│   ├── mission_progress_monitor.py # 任务进度监控节点
│   └── px4_param_bootstrap.py      # PX4 参数设置节点（COM_RCL_EXCEPT）
│
├── src/                    # （保留，目前无C++源文件）
└── include/                # （保留，目前无C++头文件）
```

---

## 4. launch 文件详细说明

### 4.1 phase1_minimal_demo.launch — 核心入口

这是**最重要的参数化模板**，Phase 1 和 Phase 2 都通过 `<include>` 复用它。

**可配置的关键参数：**

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `drone_id` | 0 | 无人机编号（多机时区分） |
| `model_name` | `iris_0` | Gazebo 模型名称 |
| `truth_odom_topic` | `/iris_0/truth_odom` | 里程计话题 |
| `use_truth_odom` | true | 是否启动里程计适配器 |
| `use_planner` | true | 是否启动 EGO 规划器 |
| `use_px4ctrl` | true | 是否启动 px4ctrl |
| `use_takeoff_trigger` | true | 是否使用自动起飞 |
| `takeoff_delay` | 8.0 s | 起飞指令延迟 |
| `target_x/y/z` | 5,0,1 | 目标位置（m）|
| `planner_realworld_experiment` | false | true时需等待traj_start_trigger |
| `mavros_attitude_cmd_topic` | `/mavros/setpoint_raw/attitude` | 可重映射到 `/irisN/mavros/...` |

**节点拓扑（use_truth_odom=true, use_planner=true, use_px4ctrl=true 时）：**

```
/truth_odom_adapter    订阅 /gazebo/model_states → 发布 /truth_odom, /truth_pose
/ego_planner_node      订阅 /truth_odom → 发布 /planning/bspline
/traj_server           订阅 /planning/bspline → 发布 /position_cmd
/px4ctrl               订阅 /truth_odom, /position_cmd → 发布 /mavros/setpoint_raw/attitude
/phase1_takeoff_trigger 延迟N秒后发布 TakeoffLand.TAKEOFF → /px4ctrl/takeoff_land
/traj_start_trigger    延迟M秒后发布 PoseStamped → /traj_start_trigger
```

### 4.2 phase1_fullstack.launch — 全栈入口

包含三层：
1. `phase1_px4_sim.launch`（仿真层）
2. `phase1_algo_stack.launch`（规划层）
3. `phase1_px4ctrl_stack.launch`（控制层）

是日常**单机完整仿真**的唯一入口。

### 4.3 phase2_dual_uav_stack.launch — 双机算法栈

通过 `<group ns="uav0">` 和 `<group ns="uav1">` 分别 include `phase1_minimal_demo.launch`，并将 MAVROS 相关话题重映射为 `/iris_0/mavros/...` 和 `/iris_1/mavros/...`。

**多机参数重映射示例：**

```xml
<group ns="uav0">
  <include file="$(find clean_uav_core)/launch/phase1_minimal_demo.launch">
    <arg name="mavros_state_topic"        value="/iris_0/mavros/state"/>
    <arg name="mavros_attitude_cmd_topic" value="/iris_0/mavros/setpoint_raw/attitude"/>
    <arg name="mavros_set_mode_service"   value="/iris_0/mavros/set_mode"/>
    <!-- ... -->
  </include>
</group>
```

---

## 5. scripts/ 脚本节点详细说明

### 5.1 truth_odom_adapter.py

```
可配置参数（ROS param）：
  ~model_name   默认 "iris"       Gazebo 中的模型名
  ~world_frame  默认 "world"      里程计坐标系
  ~child_frame  默认 "base_link"  机体坐标系
  ~odom_topic   默认 "/cleanroom/truth_odom"
  ~pose_topic   默认 "/cleanroom/truth_pose"

订阅：/gazebo/model_states
发布：<odom_topic>, <pose_topic>
```

**性能优化**：首次消息时查找模型索引，后续直接数组索引访问，避免每帧字符串比较。

### 5.2 takeoff_land_trigger.py

```
可配置参数：
  ~command  默认 "takeoff"   可选 "takeoff" 或 "land"
  ~topic    默认 "/px4ctrl/takeoff_land"
  ~delay    默认 5.0 s       等待时间
  ~repeat   默认 5           重复发送次数
  ~rate     默认 2.0 Hz      发送频率

以 latch=True 模式发布，确保后订阅的节点也能收到
```

### 5.3 traj_start_trigger.py

```
可配置参数：
  ~odom_topic     默认 "/truth_odom"
  ~trigger_topic  默认 "/traj_start_trigger"
  ~delay          默认 12.0 s
  ~repeat         默认 5
  ~rate           默认 2.0 Hz

等待里程计到达后，将当前位姿作为 PoseStamped 发布到规划器
用于触发 realworld_experiment 模式下的 ego_planner_node
```

### 5.4 dual_traj_start_trigger.py

```
可配置参数：
  ~uav0_odom_topic     "/uav0/truth_odom"
  ~uav0_trigger_topic  "/uav0/traj_start_trigger"
  ~uav1_odom_topic     "/uav1/truth_odom"
  ~uav1_trigger_topic  "/uav1/traj_start_trigger"
  ~delay               12.0 s
  ~wait_odom_timeout   30.0 s（等待两架机里程计的超时）

同步触发双机规划器，确保两架机同时开始规划、避免时序差异造成碰撞
```

### 5.5 goal_point_publisher.py

```
可配置参数：
  ~topic     默认 "/move_base_simple/goal"
  ~frame_id  默认 "map"
  ~x/y/z     默认 5.0/0.0/1.0 m
  ~delay     默认 2.0 s
  ~repeat    默认 5
  ~rate      默认 2.0 Hz

用于 Phase1 自动发布固定目标点，替代手动在 RViz 中点击
```

### 5.6 mission_progress_monitor.py

```
订阅：
  ~odom_topic（里程计）
  ~cmd_topic（PositionCommand，可选）
  ~planner_bspline_topic（Bspline，可选）
发布：~status_topic（std_msgs/String，latch=True）

功能：
  - 监控无人机到达预设航点的进度
  - 支持两种备用判断模式：
    a) use_cmd_reach_fallback：通过 PositionCommand 目标距离判断
    b) use_planner_activity_fallback：通过规划器活跃度判断
  - force_complete_timeout：超时强制完成（避免死锁）
```

### 5.7 px4_param_bootstrap.py

```
可配置参数：
  ~service_name       "/mavros/param/set"
  ~param_name         "COM_RCL_EXCEPT"
  ~integer_value      4（位掩码：禁用RC丢失保护）

功能：
  启动后等待 MAVROS param 服务可用，拉取参数缓存后设置目标参数
  对多机场景，每架机有独立的 param 服务，需各自 bootstrap
```

---

## 6. config/ 配置文件说明

### 6.1 phase1_px4ctrl_no_rc.yaml

```yaml
auto_takeoff_land:
  enable: true          # 启用自动起降
  enable_auto_arm: true # 允许软件解锁（无遥控器时必须）
  no_RC: true           # 完全禁用 RC 输入
  takeoff_height: 1.0   # 目标高度 1.0 m
  takeoff_land_speed: 0.4  # 起降速度 0.4 m/s
```

此文件通过 `rosparam load` 覆盖 `ctrl_param_fpv.yaml` 中的 `no_RC: false`，实现无遥控器全自动起飞。

---

## 7. quadrotor_msgs/ 本地定制说明

```
src/quadrotor_msgs/
├── msg/
│   ├── PositionCommand.msg   # 新增 geometry_msgs/Vector3 jerk 字段
│   ├── TakeoffLand.msg       # 新增消息类型（EGO原版无此消息）
│   ├── Px4ctrlDebug.msg      # 新增消息类型（调试信息）
│   └── ... （其他原有消息）
├── include/quadrotor_msgs/
│   ├── encode_msgs.h
│   └── decode_msgs.h
└── src/
    ├── encode_msgs.cpp
    └── decode_msgs.cpp
```

**为什么不能用外部版本？**
- `px4ctrl` 需要 `TakeoffLand`（起降指令）和 `Px4ctrlDebug`（调试输出）
- `traj_server` 发布的 `PositionCommand` 包含 `jerk` 字段，`px4ctrl` 也使用该字段做前馈控制
- 三个改动都在同一文件，只有本地副本能同时满足所有需求

---

## 8. px4ctrl/src/ 源文件说明

```
src/px4ctrl/src/
├── px4ctrl_node.cpp          # ROS 入口，订阅8个话题，启动 250Hz 主循环
├── PX4CtrlFSM.h / .cpp       # 五态有限状态机（详见架构文档）
├── controller.h / .cpp        # LinearControl 类：级联PID + 推力模型
├── input.h / .cpp             # 输入数据结构（Odom/RC/Cmd/Battery 等）
├── PX4CtrlParam.h / .cpp      # 从 ROS 参数服务器加载 YAML 参数
│
│   ── 注释文档 ──
├── px4ctrl_node.cpp_guide.md
├── PX4CtrlFSM.cpp_guide.md
├── controller.cpp_guide.md
├── input.cpp_guide.md
└── PX4CtrlParam.cpp_guide.md
```

---

## 9. tools/ 工具脚本

```
tools/
└── source_phase1_env.sh   # 环境变量设置脚本

# 内容（关键部分）：
source /opt/ros/noetic/setup.bash
source ~/XTDrone/cleanroom_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:\
  ~/XTDrone/PX4_Firmware:\
  ~/XTDrone/PX4_Firmware/Tools/sitl_gazebo
```

**每次新开终端都必须执行此脚本**，否则 `roslaunch clean_uav_core phase1_fullstack.launch` 会报找不到 `px4` 包。

---

## 10. docs/ 文档索引

| 文件 | 内容 |
|------|------|
| `system_architecture_CN.md` | 系统架构设计（本系列）|
| `directory_structure_CN.md` | 目录结构详解（本文）|
| `beginner_guide_CN.md` | 新手入门教程 |
| `phase1_runtime_chain_design.md` | Phase1 运行链设计（英文）|
| `phase1_dependency_inventory.md` | 依赖清单 |
| `phase1_px4ctrl_config_explanation_CN.md` | px4ctrl 参数详解 |
| `phase1_split_launch_guide.md` | 拆分 launch 使用指南 |
| `phase1_takeoff_tuning_CN.md` | 起飞调参指南 |
| `CLEANROOM_PHASE2_MULTI_UAV_PLAN_CN.md` | Phase2 多机实施计划 |
| `phase2_architecture_and_principles_CN.md` | Phase2 架构原则 |
| `phase2_runtime_log_reading_guide_CN.md` | Phase2 运行日志解读 |
| `px4ctrl_and_system_changes_CN.md` | px4ctrl 系统改动说明 |
