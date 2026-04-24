# Phase 2：多机协同系统

## 概述

Phase 2 在 Phase 1 基础上扩展了多无人机协同导航能力。它实现了双机到多机的完整协同栈，包括命名空间隔离、轨迹广播和避碰功能。

## 系统架构

```
┌──────────────────────────────────────────────────────────────┐
│  Phase 2 多机系统                                             │
│                                                              │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐   │
│  │   UAV 0      │    │   UAV 1      │    │   UAV N      │   │
│  │ ┌─────────┐  │    │ ┌─────────┐  │    │ ┌─────────┐  │   │
│  │ │ Planner │◄─┼────┼─│ Planner │◄─┼────┼─│ Planner │  │   │
│  │ └────┬────┘  │    │ └────┬────┘  │    │ └────┬────┘  │   │
│  │      │       │    │      │       │    │      │       │   │
│  │ ┌────▼────┐  │    │ ┌────▼────┐  │    │ ┌────▼────┐  │   │
│  │ │ px4ctrl │  │    │ │ px4ctrl │  │    │ │ px4ctrl │  │   │
│  │ └────┬────┘  │    │ └────┬────┘  │    │ └────┬────┘  │   │
│  └───────┼───────┘    └───────┼───────┘    └───────┼───────┘   │
│          │                    │                    │           │
│          └────────────────────┴────────────────────┘           │
│                     ↓ /swarm/broadcast_bspline                 │
│                  轨迹广播与协同                                │
└──────────────────────────────────────────────────────────────┘
```

## 启动方式

### 双机标准启动

**终端 1**：启动仿真层（必须先启动！）
```bash
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase2_px4_multi_sim.launch gui:=false
```

等待 10-15 秒，确保 Gazebo 完全加载。

**终端 2**：启动算法与控制层
```bash
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase2_dual_uav_stack.launch
```

### 多机扩展启动

修改 `src/clean_uav_core/config/swarm_config.yaml`：

```yaml
drone_num: 3  # 无人机数量

drone_1:
  target_x: -2.0
  target_y: 3.5
  target_z: 1.9

drone_2:
  target_x: 1.0
  target_y: 6.5
  target_z: 1.9

drone_3:
  target_x: 3.0
  target_y: 2.0
  target_z: 1.9
```

生成 launch 文件：
```bash
python3 src/clean_uav_core/scripts/swarm_launch_generator_yaml.py
```

## 命名空间设计

### 话题命名空间

```
全局话题：
├── /gazebo/model_states              # Gazebo 真值
├── /swarm/broadcast_bspline          # 轨迹广播
└── /swarm/traj_start_trigger         # 同步触发

UAV0 命名空间：
├── /uav0/truth_odom                  # 里程计
├── /uav0/position_cmd                # 控制指令
├── /uav0/ego_planner_node            # 规划器
├── /uav0/traj_server                 # 轨迹服务器
├── /uav0/px4ctrl/...                 # 控制器
└── /iris_0/mavros/...                # MAVROS

UAV1 命名空间：
├── /uav1/truth_odom
├── /uav1/position_cmd
└── ...
```

### 参数重映射

```xml
<group ns="uav0">
  <include file="$(find clean_uav_core)/launch/phase1_minimal_demo.launch">
    <arg name="mavros_state_topic" value="/iris_0/mavros/state"/>
    <arg name="position_cmd_topic" value="/uav0/position_cmd"/>
    <!-- ... -->
  </include>
</group>
```

## 可配置参数

### 双机目标点

```bash
roslaunch clean_uav_core phase2_dual_uav_stack.launch \
  uav0_target_x:=-2.0 uav0_target_y:=3.5 uav0_target_z:=1.9 \
  uav1_target_x:=1.0 uav1_target_y:=6.5 uav1_target_z:=1.9
```

### 速度参数

```bash
roslaunch clean_uav_core phase2_dual_uav_stack.launch \
  planner_max_vel:=2.0 \
  planner_max_acc:=3.0 \
  planner_max_jerk:=4.0
```

### 协同开关

```bash
# 启用轨迹广播
roslaunch clean_uav_core phase2_dual_uav_stack.launch \
  use_traj_broadcast:=true

# 启用避碰
roslaunch clean_uav_core phase2_dual_uav_stack.launch \
  use_collision_avoidance:=true
```

## 关键话题

### 全局话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/swarm/broadcast_bspline` | `traj_utils/Bspline` | 轨迹广播 |
| `/swarm/traj_start_trigger` | `std_msgs/Bool` | 同步触发 |
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | Gazebo 真值 |

### 单机话题（UAV0 为例）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/uav0/truth_odom` | `nav_msgs/Odometry` | 里程计 |
| `/uav0/position_cmd` | `quadrotor_msgs/PositionCommand` | 控制指令 |
| `/iris_0/mavros/state` | `mavros_msgs/State` | PX4 状态 |
| `/iris_0/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | 姿态指令 |

## 启动时序

```
t=0s    ┌─────────────────────────────────────────┐
        │ phase2_px4_multi_sim.launch 启动        │
        │ - 加载 N 个无人机模型                    │
        │ - 启动 N 个 MAVROS                      │
        └─────────────────────────────────────────┘
t=~10s  ┌─────────────────────────────────────────┐
        │ Gazebo 完全加载                         │
        │ - 所有模型可见                           │
        │ - MAVROS 连接建立                        │
        └─────────────────────────────────────────┘
t=~10s  ┌─────────────────────────────────────────┐
        │ phase2_dual_uav_stack.launch 启动       │
        │ - 初始化 N 个规划器                      │
        │ - 初始化 N 个控制器                      │
        └─────────────────────────────────────────┘
t=~15s  ┌─────────────────────────────────────────┐
        │ 双机同步起飞                             │
        │ - dual_traj_start_trigger 触发           │
        └─────────────────────────────────────────┘
t=~20s  ┌─────────────────────────────────────────┐
        │ 双机同步规划                             │
        │ - 开始轨迹跟踪                           │
        └─────────────────────────────────────────┘
```

## 监控与调试

### 基本监控

```bash
# 监控 UAV0
rostopic echo /uav0/truth_odom/pose/pose/position
rostopic echo /uav0/position_cmd

# 监控 UAV1
rostopic echo /uav1/truth_odom/pose/pose/position
rostopic echo /uav1/position_cmd

# 监控轨迹广播
rostopic echo /swarm/broadcast_bspline
```

### 可视化

```bash
# 节点图（所有节点）
rqt_graph

# 查看特定命名空间
rosnode list | grep uav0
rosnode list | grep uav1
```

### 日志记录

```bash
# 记录双机数据
rosbag record \
  /uav0/truth_odom /uav1/truth_odom \
  /uav0/position_cmd /uav1/position_cmd \
  /swarm/broadcast_bspline \
  -O phase2_dual_test
```

## 协同机制

### 轨迹广播

每个无人机将自己的轨迹发布到全局话题：

```
UAV0: /uav0/planning/bspline → /swarm/broadcast_bspline
UAV1: /uav1/planning/bspline → /swarm/broadcast_bspline
```

其他无人机订阅并用于避碰规划。

### 同步触发

```bash
# 双机同步规划触发
rostopic pub /swarm/traj_start_trigger std_msgs/Bool "data: true"
```

### 避碰策略

1. **静态避碰**：EGO-Planner 内置障碍物避让
2. **动态避碰**：考虑其他无人机的轨迹
3. **安全裕度**：设置最小安全间距

## 常见场景

### 场景 1：对称任务

```bash
# 双机对称飞行
roslaunch clean_uav_core phase2_dual_uav_stack.launch \
  uav0_target_x:=-2.0 uav0_target_y:=3.5 \
  uav1_target_x:=2.0 uav1_target_y:=-3.5
```

### 场景 2：汇聚任务

```bash
# 双机汇聚到同一点
roslaunch clean_uav_core phase2_dual_uav_stack.launch \
  uav0_target_x:=0.0 uav0_target_y:=0.0 \
  uav1_target_x:=0.0 uav1_target_y:=0.0
```

### 场景 3：序列任务

通过脚本动态更新目标点实现序列任务。

## 故障排查

### 问题 1：启动顺序错误

**症状**：MAVROS 无法连接

**解决**：
1. 必须先启动仿真层
2. 等待 10-15 秒
3. 再启动算法层

### 问题 2：话题命名空间混乱

**症状**：UAV0 和 UAV1 数据交叉

**解决**：
```bash
# 检查话题命名空间
rostopic list | grep uav0
rostopic list | grep uav1

# 确认话题映射正确
rostopic info /uav0/position_cmd
```

### 问题 3：双机碰撞风险

**解决**：
1. 启用轨迹广播
2. 启用避碰功能
3. 设置安全裕度

## 性能基准

### 典型性能指标

| 指标 | 单机 | 双机 | 三机 |
|------|------|------|------|
| 控制频率 | 50-100 Hz | 50-100 Hz | 50-100 Hz |
| CPU 占用 | 20-30% | 30-40% | 40-50% |
| 内存占用 | ~500 MB | ~900 MB | ~1.3 GB |

### 资源建议

| 无人机数 | 最低配置 | 推荐配置 |
|----------|----------|----------|
| 1-2 | 4 核 CPU | 8 核 CPU |
| 3-6 | 8 核 CPU | 16 核 CPU |
| 6+ | 16 核 CPU | 32 核 CPU |

## 下一步

完成 Phase 2 学习后：
- **Phase 3 VINS 集成**：`03_Phase3_VINS集成.md`
- **集群系统**：`05_集群系统/`
- **基准测试**：`06_基准测试/`
