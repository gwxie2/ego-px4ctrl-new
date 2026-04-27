# vins_ego_forest

## 项目概述

本项目是一个专为UAV（无人机）模拟设计的Gazebo世界生成器，专注于创建各种复杂环境用于测试和基准测试UAV的导航、规划和控制算法。项目名称“vins_ego_forest”源于其最初用于VINS（视觉惯性导航系统）和EGO-Planner的森林环境模拟。

### 原理

项目的核心原理是生成可配置的Gazebo世界，用于评估UAV算法在不同挑战性环境中的表现：

- **多层迷宫**：模拟室内多层建筑环境，测试UAV的垂直导航和3D路径规划能力。设计包括楼梯和楼层间隙，EGO-Planner的`map_size_z`参数需调整以覆盖完整高度。
- **高速轨道**：创建高速运动场景，评估UAV的跟踪控制和多机协作。支持不同目标模式（如“same”模式下所有无人机共享相同目标）。
- **群体基准测试**：分为四个阶段的森林环境基准测试，逐步增加复杂度：
  - 阶段1：基础森林布局，测试基本导航。
  - 阶段2：稀疏校准，添加柱子确保起点/终点无障碍。
  - 阶段3：密集静态压力测试，增加障碍物密度。
  - 阶段4：动态或高级场景（通过脚本扩展）。
- **语义搜索**：群体UAV在森林中执行语义目标搜索任务，结合感知和规划。
- **森林布局生成**：自动化添加柱子障碍物，随机分布但避免关键区域。
- **占用检查**：验证世界中位置的可用性，避免碰撞。
- **动态目标指挥**：发布实时目标更新，支持动态任务。
- **世界缩放**：调整世界大小以适应不同仿真需求。

这些原理确保了从简单到复杂的渐进测试，覆盖静态/动态障碍、单机/多机场景。

### 使用方法

1. **环境准备**：
   - 安装Gazebo和ROS Noetic。
   - 克隆仓库并进入目录。
   - 激活虚拟环境：`source .venv/bin/activate`。
   - 安装Python依赖（如果需要）：`pip install -r requirements.txt`（假设存在）。

2. **生成世界**：
   - 运行相应Python脚本生成.world文件。
   - 示例：`python scripts/generate_vins_multi_floor_maze.py`。

3. **加载仿真**：
   - 使用`gazebo worlds/xxx.world`加载世界。
   - 对于ROS集成，源ROS环境：`source /opt/ros/noetic/setup.bash`。

4. **运行群体命令**：
   - 启动swarm_commander：`python3 scripts/swarm_commander.py --num-drones 3 --rate-hz 10 --frame-id world`。
   - 检查ROS话题：`rostopic echo /drone_0_planning/goal`。

5. **自定义参数**：
   - 大多数脚本支持命令行参数，如--num-drones, --radius等。
   - 参考脚本帮助：`python script.py --help`。

6. **验证和调试**：
   - 使用check_occupancy.py检查位置占用。
   - 调整世界缩放以适应硬件。

### 所有功能

注意：phase1/2/3遵循下面的基本原则:
- phase1: 仅包含地图边界，起点/终点锚点和视觉初始化锚点。
- phase2: 在phase1基础上增加柱子障碍物，保持起点/终点周围2.0m无障碍区。（推荐）
- phase3: 在phase2基础上增加动态障碍物，保持相同的无障碍区约束。（不推荐，因为已知gazebo在动态障碍物存在视觉和碰撞箱不一致问题）


- **世界生成脚本**：
  - `generate_vins_multi_floor_maze.py`：生成多层迷宫世界。
  - `generate_vins_high_speed_track.py`：生成高速轨道世界，支持多无人机和目标模式。
  - `generate_swarm_benchmark_forest_phase1.py`：生成群体基准阶段1世界。
  - `generate_swarm_benchmark_forest_phase2.py`：生成稀疏校准世界。
  - `generate_swarm_benchmark_forest_phase3.py`：生成密集静态世界。
  - `generate_forest_layout.py`：生成森林布局，添加柱子障碍物。
  - `generate_stage3_dynamic_world.py`：生成动态世界（扩展阶段3）。
- **工具脚本**：
  - `swarm_commander.py`：ROS-based群体指挥官，发布目标姿势。
  - `dynamic_goal_commander.py`：动态目标发布器（简化版）。
  - `check_occupancy.py`：检查世界中位置占用。
  - `scale_worlds.py`：缩放世界文件。
- **世界文件**：
  - 基础世界：vins_multi_floor_maze.world, vins_high_speed_track.world 等。
  - 扩展世界：包括_x1.5（1.5倍缩放）、_x2（2倍缩放）、_6UAV（6无人机配置）等变体。
  - 阶段世界：vins_ego_forest_stage1.world 到 stage3.world。

### 文件结构

```
.
├── .venv/                              # Python虚拟环境（忽略）
├── README.md                           # 项目文档（本文件）
├── scripts/                            # Python脚本目录
│   ├── __pycache__/                    # Python缓存（忽略）
│   ├── check_occupancy.py              # 检查位置占用工具
│   ├── dynamic_goal_commander.py       # 动态目标指挥官
│   ├── generate_forest_layout.py       # 生成森林布局脚本
│   ├── generate_stage3_dynamic_world.py # 生成动态世界脚本
│   ├── generate_swarm_benchmark_forest_phase1.py # 生成群体基准阶段1
│   ├── generate_swarm_benchmark_forest_phase2.py # 生成群体基准阶段2
│   ├── generate_swarm_benchmark_forest_phase3.py # 生成群体基准阶段3
│   ├── generate_vins_high_speed_track.py # 生成高速轨道
│   ├── generate_vins_multi_floor_maze.py # 生成多层迷宫
│   ├── scale_worlds.py                 # 世界缩放工具
│   └── swarm_commander.py              # 群体指挥官
├── swarm_benchmark_forest_protocol.md  # 群体基准森林协议文档
├── uav_position_goal.md                # UAV起始和目标位置文档
└── worlds/                             # 生成的世界文件目录
    ├── swarm_benchmark_forest_phase1.world
    ├── swarm_benchmark_forest_phase12.world
    ├── swarm_benchmark_forest_phase12_x1.5.world
    ├── swarm_benchmark_forest_phase1_6UAV.world
    ├── swarm_benchmark_forest_phase1_6UAV_x1.5.world
    ├── swarm_benchmark_forest_phase1_x1.5.world
    ├── swarm_benchmark_forest_phase2.world
    ├── swarm_benchmark_forest_phase2_6UAV.world
    ├── swarm_benchmark_forest_phase2_6UAV_x1.5.world
    ├── swarm_benchmark_forest_phase2_x1.5.world
    ├── swarm_benchmark_forest_phase3.world
    ├── swarm_benchmark_forest_phase3_6UAV.world
    ├── swarm_benchmark_forest_phase3_6UAV_x1.5.world
    ├── swarm_benchmark_forest_phase3_x1.5.world
    ├── swarm_semantic_search.world
    ├── vins_ego_forest_stage1.world
    ├── vins_ego_forest_stage1_x1.5.world
    ├── vins_ego_forest_stage1_x2.world
    ├── vins_ego_forest_stage2.world
    ├── vins_ego_forest_stage2_x1.5.world
    ├── vins_ego_forest_stage2_x2.world
    ├── vins_ego_forest_stage2_x2_filllight.world
    ├── vins_ego_forest_stage3.world
    ├── vins_ego_forest_stage3_x1.5.world
    ├── vins_ego_forest_stage3_x2.world
    ├── vins_high_speed_track.world
    └── vins_multi_floor_maze.world
```

## 多层迷宫

生成跨楼层基准世界和任务表：

```bash
./scripts/generate_vins_multi_floor_maze.py
gazebo worlds/vins_multi_floor_maze.world
```

建议的EGO-Planner设置是`map_size_z = 10.0`到`12.0`，以确保第二层、楼梯和最高任务点不被裁剪。

## 高速轨道

生成新的高速基准世界及其UAV坐标表：

```bash
./scripts/generate_vins_high_speed_track.py --num-drones 3 --goal-mode same
gazebo worlds/vins_high_speed_track.world
```

脚本会写入`worlds/vins_high_speed_track.world`并重新生成`uav_position_goal.md`，包含选定的`N`和目标映射模式。

## 群体基准协议

详见[swarm_benchmark_forest_protocol.md](swarm_benchmark_forest_protocol.md)了解可重用的阶段1-4协议。

## 群体基准阶段1

生成基准世界：

```bash
./scripts/generate_swarm_benchmark_forest_phase1.py
```

自定义无人机数量和生成半径：

```bash
./scripts/generate_swarm_benchmark_forest_phase1.py --num-drones 9 --radius 15
```

```bash
gazebo worlds/swarm_benchmark_forest_phase1.world
```

## 群体基准阶段2

生成稀疏校准世界：

```bash
./scripts/generate_swarm_benchmark_forest_phase2.py
```

生成的稀疏柱子在每个起点/终点锚点周围保持2.0m无障碍区，其半径相对于原始稀疏布局加倍。

```bash
gazebo worlds/swarm_benchmark_forest_phase2.world
```

## 群体基准阶段3

生成密集静态压力测试世界：

```bash
./scripts/generate_swarm_benchmark_forest_phase3.py
```

密集柱子也尊重每个起点/终点锚点周围相同的2.0m无障碍区，采样柱子半径范围加倍。

```bash
gazebo worlds/swarm_benchmark_forest_phase3.world
```

### 生成6 UAV示例

若需生成并运行6架无人机的派生世界，请按以下顺序执行（示例将自动命名为带`_6UAV`的文件）：

```bash
./scripts/generate_swarm_benchmark_forest_phase1.py --num-drones 6 --out-world worlds/swarm_benchmark_forest_phase1_6UAV.world
./scripts/generate_swarm_benchmark_forest_phase2.py --base-world worlds/swarm_benchmark_forest_phase1_6UAV.world --out-world worlds/swarm_benchmark_forest_phase2_6UAV.world
./scripts/generate_swarm_benchmark_forest_phase3.py --base-world worlds/swarm_benchmark_forest_phase2_6UAV.world --out-world worlds/swarm_benchmark_forest_phase3_6UAV.world
gazebo worlds/swarm_benchmark_forest_phase3_6UAV.world
```

## 阶段1快速开始

生成基准世界：

```bash
./scripts/generate_swarm_benchmark_forest_phase1.py
```

```bash
gazebo worlds/swarm_benchmark_forest_phase1.world
```

## 阶段2生成静态森林

```bash
python scripts/generate_forest_layout.py
```

## 阶段2快速开始

```bash
gazebo worlds/vins_ego_forest_stage2.world
```

## 阶段4群体指挥官 (ROS 1)

```bash
source /opt/ros/noetic/setup.bash
python3 scripts/swarm_commander.py --num-drones 3 --rate-hz 10 --frame-id world
```

可选参数：

```bash
source /opt/ros/noetic/setup.bash
python3 scripts/swarm_commander.py --num-drones 6 --rate-hz 10 --frame-id world
```

指挥官现在使用与阶段1生成器相同的有界角度扩展规则，因此更大的群体规模保持在固定飞行包络内。

话题检查：

```bash
rostopic hz /drone_0_planning/goal
rostopic echo /drone_0_planning/goal
rostopic echo /drone_1_planning/goal
rostopic echo /drone_2_planning/goal
```

兼容性说明：

- `scripts/dynamic_goal_commander.py` 作为简单的2无人机目标发布器仍然可用。

## 参考生成点

- Drone 0: `(0, 1.5, 0.1)`
- Drone 1: `(0, -1.5, 0.1)`

## 群体基准阶段1初始姿势

- Drone 0: `(4.330, -2.500, 0.100, 2.618)`
- Drone 1: `(5.000, 0.000, 0.100, -3.142)`
- Drone 2: `(4.330, 2.500, 0.100, -2.618)`
