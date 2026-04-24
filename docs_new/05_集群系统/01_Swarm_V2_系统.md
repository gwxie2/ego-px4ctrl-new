# Swarm V2 集群系统

## 概述

Swarm V2 是新一代集群系统，相比 V1 进行了全面重构：

- **V1**：基于 PoseStamped 的原始版本
- **V2**：基于 GoalSet 和 MINCOTraj 的现代化架构

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│  Swarm V2 架构                                               │
│                                                             │
│  ┌──────────────┐      ┌──────────────────────────────┐    │
│  │  GoalSet     │ ───→ │   EGO Planner V2            │    │
│  │  目标分配器   │      │   (MINCOTraj 支持)           │    │
│  └──────────────┘      └──────────┬───────────────────┘    │
│                                  ↓                         │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              /drone_X/position_cmd                  │   │
│  └─────────────────────────────────────────────────────┘   │
│                                  ↓                         │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              px4ctrl (独立实例)                      │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## V1 与 V2 对比

| 特性 | V1 | V2 |
|------|----|----|
| 消息类型 | PoseStamped | GoalSet |
| 轨迹表示 | Bspline | MINCOTraj |
| 包隔离 | 否 | 是 |
| 可执行文件 | 共享 | 独立 |
| Launch 入口 | 混合 | 独立 |

## 启动方式

### 生成顶层 launch

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh
python3 src/clean_uav_core/scripts/swarm_launch_generator_yaml.py \
  --version v2
```

生成文件：`src/clean_uav_core/launch/swarm_top_level_v2.launch`

### 基础启动

```bash
roslaunch clean_uav_core swarm_top_level_v2.launch gui:=false
```

### 带增强组件启动

```bash
# 启用 GoalSet 工具链
roslaunch clean_uav_core swarm_top_level_v2.launch \
  gui:=false \
  enable_goal_tooling_v2:=true

# 启用随机目标分配
roslaunch clean_uav_core swarm_top_level_v2.launch \
  gui:=false \
  enable_goal_tooling_v2:=true \
  enable_random_goals_v2:=true

# 启用动态障碍
roslaunch clean_uav_core swarm_top_level_v2.launch \
  gui:=false \
  enable_moving_obstacles_v2:=true
```

## 配置文件

### swarm_config_v2.yaml

```yaml
# 集群配置
swarm_config:
  drone_num: 3

# 无人机配置
drone_0:
  id: 0
  model_name: iris_0
  init_x: 0.0
  init_y: 0.0
  init_z: 0.2

drone_1:
  id: 1
  model_name: iris_1
  init_x: 1.0
  init_y: 0.0
  init_z: 0.2

drone_2:
  id: 2
  model_name: iris_2
  init_x: -1.0
  init_y: 0.0
  init_z: 0.2
```

### swarm_planner_v2.yaml

```yaml
# V2 规划器参数
planner_v2:
  max_vel: 2.0
  max_acc: 3.0
  max_jerk: 4.0

  # MINCOTraj 参数
  minco_traj:
    order: 5
    lambda1: 1.0
    lambda2: 1.0

  # GoalSet 参数
  goal_set:
    enable: true
    mode: "manual"  # manual, random, file
```

## 关键组件

### 1. assign_goals_v2

**功能**：目标分配与调度

**话题**：
- 订阅：`/swarm/v2/goal_set`
- 发布：`/drone_X/goal`

### 2. ego_planner_v2

**功能**：V2 规划器，支持 MINCOTraj

**改进**：
- 包名隔离：`ego_planner_v2`
- 可执行文件隔离：`ego_planner_node_v2`
- 消息类型：`GoalSet`

### 3. traj_server_v2

**功能**：V2 轨迹服务器

**话题**：
- 订阅：`/drone_X/planning/bspline_v2`
- 发布：`/drone_X/position_cmd`

### 4. moving_obstacles_v2

**功能**：动态障碍物广播

**话题**：
- 发布：`/swarm/v2/moving_obstacles`

## 可选增强组件

### GoalSet 工具链

```bash
enable_goal_tooling_v2:=true
enable_random_goals_v2:=true
```

组件：
- `assign_goals_v2`：目标分配
- `random_goals_v2`：随机目标生成
- `rviz_plugins_v2`：RViz 插件

### 动态障碍

```bash
enable_moving_obstacles_v2:=true
enable_moving_obstacles_joy_node_v2:=true
```

组件：
- `moving_obstacles_v2`：障碍发布
- `joy_node`：摇杆控制

### 手动接管

```bash
enable_manual_take_over_v2:=true
enable_manual_take_over_station_v2:=true
```

组件：
- `manual_take_over_v2`：单机接管
- `manual_take_over_station`：地面站

### 轨迹可视化

```bash
enable_odom_visualization_v2:=true
odom_visualization_scale_v2:=0.35
```

## 话题命名空间

```
全局话题：
├── /swarm/v2/goal_set                    # 目标集合
├── /swarm/v2/moving_obstacles            # 动态障碍
├── /swarm/v2/manual_take_over/joystick   # 手动接管
└── /gazebo/model_states                  # Gazebo 真值

单机话题（drone_0）：
├── /drone_0/odom                         # 里程计
├── /drone_0/goal                         # 分配目标
├── /drone_0/position_cmd                 # 控制指令
├── /drone_0/ego_planner_node_v2          # 规划器
├── /drone_0/traj_server_v2               # 轨迹服务器
└── /iris_0/mavros/...                    # MAVROS
```

## 验证脚本

### 烟雾测试

```bash
./test_swarm_v2_smoke.sh
```

验证：
- V2 launch 生成
- 节点图展开
- 增强组件链接

### 健康检查

```bash
./test_swarm_v2_runtime_health.sh
```

验证：
- 启动成功
- 里程计在线
- MAVROS 连接
- 轨迹服务器心跳

### 运行矩阵

```bash
./test_swarm_v2_runtime_matrix.sh
```

覆盖：
- 默认主链
- Goal tooling
- moving_obstacles
- manual_take_over
- 全组合场景

## 与 V1 共存

V2 设计为与 V1 完全隔离：

```
src/
├── ego_planner/           # V1 (符号链接)
├── ego_planner_v2/        # V2 (独立包)
├── plan_env/              # V1
├── plan_env_v2/           # V2
└── ...
```

可以同时运行 V1 和 V2：

```bash
# 终端 1：V1
roslaunch clean_uav_core swarm_top_level.launch

# 终端 2：V2
roslaunch clean_uav_core swarm_top_level_v2.launch
```

## 迁移指南

### 从 V1 迁移到 V2

1. **消息类型更新**
   ```cpp
   // V1
   #include <quadrotor_msgs/Goal.h>

   // V2
   #include <ego_planner_v2/GoalSet.h>
   ```

2. **话题更新**
   ```python
   # V1
   goal_pub = rospy.Publisher('/uav0/goal', Goal, queue_size=1)

   # V2
   goal_set_pub = rospy.Publisher('/swarm/v2/goal_set', GoalSet, queue_size=1)
   ```

3. **Launch 文件更新**
   ```xml
   <!-- V1 -->
   <include file="$(find ego_planner)/launch/..."/>

   <!-- V2 -->
   <include file="$(find ego_planner_v2)/launch/..."/>
   ```

## 性能对比

| 指标 | V1 | V2 |
|------|----|----|
| 启动时间 | ~10s | ~12s |
| 内存占用 | 基准 | +5% |
| CPU 占用 | 基准 | -10%（优化后） |
| 规划成功率 | 基准 | +3% |

## 下一步

- **基准测试**：`06_基准测试/01_基准测试框架.md`
- **高级功能**：`07_进阶主题/`
- **API 参考**：`08_API参考/`
