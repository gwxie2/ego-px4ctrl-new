# Phase 1：单机仿真系统

## 概述

Phase 1 是系统的入门阶段，实现了单架无人机的自主导航功能。它包含了完整的规划-控制闭环，是理解整个系统的基础。

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│  Phase 1 单机系统                                            │
│                                                             │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐    │
│  │   里程计     │ → │  规划器      │ → │  控制器      │    │
│  │ truth_odom   │   │ EGO Planner  │   │  px4ctrl     │    │
│  └──────────────┘   └──────────────┘   └──────────────┘    │
│         ↓                  ↓                  ↓             │
│    Gazebo 真值        B样条轨迹        姿态指令               │
└─────────────────────────────────────────────────────────────┘
```

## 启动方式

### 方式一：全栈启动（推荐）

```bash
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase1_fullstack.launch
```

**包含组件**：
- Gazebo 仿真环境
- PX4 SITL
- MAVROS
- truth_odom_adapter
- ego_planner + traj_server
- px4ctrl
- 自动起降触发器
- 规划启动触发器

### 方式二：分层启动（调试用）

**终端 1**：仿真层
```bash
roslaunch clean_uav_core phase1_px4_sim.launch
```

**终端 2**：规划层
```bash
roslaunch clean_uav_core phase1_algo_stack.launch
```

**终端 3**：控制层
```bash
roslaunch clean_uav_core phase1_px4ctrl_stack.launch \
  use_takeoff_trigger:=true
```

## 可配置参数

### 目标点参数

```bash
roslaunch clean_uav_core phase1_fullstack.launch \
  target_x:=5.0 \
  target_y:=0.0 \
  target_z:=1.0
```

### 速度参数

```bash
roslaunch clean_uav_core phase1_fullstack.launch \
  planner_max_vel:=2.0 \
  planner_max_acc:=3.0 \
  planner_max_jerk:=4.0
```

### 起飞参数

```bash
roslaunch clean_uav_core phase1_fullstack.launch \
  use_takeoff_trigger:=true \
  takeoff_delay:=8.0 \
  takeoff_height:=1.0
```

### GUI 控制

```bash
# 无 GUI（headless 模式，节省资源）
roslaunch clean_uav_core phase1_fullstack.launch gui:=false

# 有 GUI（可视化调试）
roslaunch clean_uav_core phase1_fullstack.launch gui:=true
```

## 关键话题

### 输入话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/truth_odom` | `nav_msgs/Odometry` | 里程计（Gazebo 真值） |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 目标点 |
| `/traj_start_trigger` | `std_msgs/Bool` | 规划触发 |
| `/px4ctrl/takeoff_land` | `quadrotor_msgs/TakeoffLand` | 起降指令 |

### 输出话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/planning/bspline` | `traj_utils/Bspline` | B 样条轨迹 |
| `/position_cmd` | `quadrotor_msgs/PositionCommand` | 位置指令 |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | 姿态指令 |
| `/px4ctrl/debug` | `quadrotor_msgs/Px4ctrlDebug` | 调试信息 |

## 启动时序详解

```
t=0s    ┌─────────────────────────────────────────┐
        │ phase1_px4_sim.launch 启动              │
        │ - Gazebo 加载世界                       │
        │ - PX4 SITL 启动                         │
        │ - MAVROS 连接                          │
        └─────────────────────────────────────────┘
t=~3s   ┌─────────────────────────────────────────┐
        │ MAVROS 连接建立                         │
        │ - /mavros/state: connected=true         │
        └─────────────────────────────────────────┘
t=~5s   ┌─────────────────────────────────────────┐
        │ 算法栈启动                               │
        │ - truth_odom_adapter 锁定模型           │
        │ - ego_planner_node 初始化               │
        │ - px4ctrl_node 初始化                   │
        └─────────────────────────────────────────┘
t=~8s   ┌─────────────────────────────────────────┐
        │ takeoff_land_trigger 触发               │
        │ - 发布 TakeoffLand.TAKEOFF              │
        └─────────────────────────────────────────┘
t=~9s   ┌─────────────────────────────────────────┐
        │ px4ctrl 状态转换                         │
        │ - MANUAL_CTRL → AUTO_TAKEOFF            │
        └─────────────────────────────────────────┘
t=~11s  ┌─────────────────────────────────────────┐
        │ 电机怠速结束                             │
        │ - 开始爬升至 takeoff_height             │
        └─────────────────────────────────────────┘
t=~14s  ┌─────────────────────────────────────────┐
        │ 到达目标高度                             │
        │ - AUTO_TAKEOFF → AUTO_HOVER             │
        └─────────────────────────────────────────┘
t=~15s  ┌─────────────────────────────────────────┐
        │ traj_start_trigger 触发                 │
        │ - ego_planner 开始规划                  │
        └─────────────────────────────────────────┘
t=~16s  ┌─────────────────────────────────────────┐
        │ px4ctrl 状态转换                         │
        │ - AUTO_HOVER → CMD_CTRL                 │
        │ - 开始执行轨迹跟踪                       │
        └─────────────────────────────────────────┘
```

## 监控与调试

### 基本监控

```bash
# 终端 1：运行仿真
roslaunch clean_uav_core phase1_fullstack.launch

# 终端 2：监控关键话题
source tools/source_phase1_env.sh

# 查看里程计
rostopic hz /truth_odom

# 查看控制指令
rostopic echo /position_cmd -n 1

# 查看 PX4 状态
rostopic echo /mavros/state -n 1

# 查看规划器状态
rostopic echo /planning/bspline -n 1
```

### 可视化

```bash
# RViz 可视化
roslaunch clean_uav_core phase1_fullstack.launch \
  use_rviz:=true

# 节点图
rqt_graph
```

### 日志记录

```bash
# 记录所有关键话题
rosbag record /truth_odom /position_cmd /planning/bspline \
  /mavros/state /px4ctrl/debug -O phase1_test

# 回放
rosbag play phase1_test.bag

# 分析
rqt_bag phase1_test.bag
```

## 常见场景

### 场景 1：点对点导航

```bash
# 简单点对点飞行
roslaunch clean_uav_core phase1_fullstack.launch \
  target_x:=5.0 target_y:=0.0 target_z:=1.0
```

### 场景 2：多点航点

修改 `src/clean_uav_core/launch/phase1_preset_mission_stack.launch` 中的航点列表：

```xml
<rosparam param="fsm/waypoints">
  - [0.0, 0.0, 1.0]
  - [2.0, 2.0, 1.5]
  - [5.0, 0.0, 1.0]
  - [0.0, 0.0, 1.0]
</rosparam>
```

### 场景 3：手动目标设置

```bash
# 启动后通过 RViz 或命令行设置目标
roslaunch clean_uav_core phase1_manual_goal_stack.launch

# 命令行设置目标
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  "header: {frame_id: 'world'} \
   pose: {position: {x: 3.0, y: 2.0, z: 1.5}}"
```

## 故障排查

### 问题 1：无人机不起飞

**检查清单**：
```bash
# 1. 检查 MAVROS 连接
rostopic echo /mavros/state

# 2. 检查里程计
rostopic echo /truth_odom -n 1

# 3. 检查起飞指令
rostopic echo /px4ctrl/takeoff_land

# 4. 检查 px4ctrl 状态
rostopic echo /px4ctrl/debug | grep state
```

### 问题 2：起飞后不移动

**检查清单**：
```bash
# 1. 检查规划触发器
rostopic echo /traj_start_trigger

# 2. 检查规划器输出
rostopic echo /planning/bspline

# 3. 检查位置指令
rostopic echo /position_cmd
```

### 问题 3：飞行不稳定

**可能原因**：
- PID 增益过高 → 降低 `Kp/Kv`
- 速度设置过高 → 降低 `max_vel`
- 仿真负载过高 → 关闭 GUI

## 性能基准

### 典型性能指标

| 指标 | 典型值 | 说明 |
|------|--------|------|
| 控制频率 | 50-100 Hz | px4ctrl 输出频率 |
| 规划频率 | 10-30 Hz | ego_planner 重规划频率 |
| 跟踪误差 | < 0.2 m | 正常飞行时 |
| 起飞时间 | ~3 s | 地面到 1.0m |
| 航点到达时间 | 取决于距离 | 通常 5-10s |

### 资源占用

| 模式 | CPU | 内存 |
|------|-----|------|
| headless | 20-30% | ~500 MB |
| with GUI | 40-60% | ~800 MB |

## 下一步

完成 Phase 1 学习后：
- **Phase 2 多机系统**：`02_Phase2_多机系统.md`
- **深入控制器**：`03_核心组件/01_px4ctrl控制器.md`
- **深入规划器**：`03_核心组件/02_EGO_Planner规划器.md`
