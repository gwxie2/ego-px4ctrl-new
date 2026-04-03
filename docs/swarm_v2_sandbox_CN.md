# Swarm V2 Sandbox 使用说明

## 1. 目标

本文档说明当前工作区中 Ego-Planner V2 sandbox 的启动入口、可选增强组件以及 Utils 迁入状态。

V2 sandbox 的设计目标是：

- 保持 V1 与 V2 的包名、可执行文件名、launch 入口完全分离
- 让 V2 能复用现有 PX4 SITL 与 px4ctrl 底座
- 用 GoalSet 与 MINCOTraj 形成独立于 V1 的第二套上层链路

## 2. 当前入口

### 2.1 生成顶层 launch

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh
python3 src/clean_uav_core/scripts/swarm_launch_generator_yaml.py --version v2
```

默认输出：

- `src/clean_uav_core/launch/swarm_top_level_v2.launch`

### 2.2 启动 V2 主链

```bash
roslaunch clean_uav_core swarm_top_level_v2.launch gui:=false
```

当前最小主链为：

- GoalSet 或预设 waypoint
- ego_planner_v2
- traj_server_v2
- position_cmd
- px4ctrl

## 3. 可选增强开关

### 3.1 GoalSet 工具链

```bash
roslaunch clean_uav_core swarm_top_level_v2.launch \
  gui:=false \
  enable_goal_tooling_v2:=true
```

该开关会额外接入：

- `assign_goals_v2`
- RViz 选择点工具参数入口

如果同时希望启用随机目标分配器，可增加：

```bash
enable_random_goals_v2:=true
```

### 3.2 动态障碍广播

```bash
roslaunch clean_uav_core swarm_top_level_v2.launch \
  gui:=false \
  enable_moving_obstacles_v2:=true
```

如果本机存在摇杆设备，也可额外启用 joy 节点：

```bash
enable_moving_obstacles_joy_node_v2:=true
```

### 3.3 手动接管与轨迹可视化

```bash
# 接入每机 manual_take_over 节点
roslaunch clean_uav_core swarm_top_level_v2.launch \
  gui:=false \
  enable_manual_take_over_v2:=true

# 额外接入地面站 joy -> manual_take_over 转发
roslaunch clean_uav_core swarm_top_level_v2.launch \
  gui:=false \
  enable_manual_take_over_v2:=true \
  enable_manual_take_over_station_v2:=true

# 接入每机 odom_visualization
roslaunch clean_uav_core swarm_top_level_v2.launch \
  gui:=false \
  enable_odom_visualization_v2:=true
```

当前新增开关说明：

- `enable_manual_take_over_v2`：为每架无人机接入 `manual_take_over_v2`
- `enable_manual_take_over_station_v2`：接入 `manual_take_over_station` 和 `joy_node`
- `enable_manual_take_over_joy_node_v2`：控制是否启动 `joy_node`，做 headless 验证时可设为 `false`
- `manual_take_over_joy_input_topic_v2`：摇杆输入话题，默认 `/joy`
- `manual_take_over_joy_topic_v2`：转发后的共享接管话题，默认 `/swarm/v2/manual_take_over/joystick`
- `manual_take_over_joy_dev_v2`：摇杆设备，默认 `/dev/input/js0`
- `enable_odom_visualization_v2`：为每架无人机接入 `odom_visualization_v2`
- `odom_visualization_scale_v2`：模型缩放比例，默认 `0.35`

## 4. 已迁入的 Utils 包

当前已经从 `Utils/` 迁入 `src/` 的包：

- `assign_goals`
- `random_goals`
- `rviz_plugins`
- `selected_points_publisher`
- `moving_obstacles`
- `manual_take_over`
- `odom_visualization`
- `pose_utils`

这些包已经做了最小必要改造：

- 去掉或收敛硬编码全局话题
- 保留为参数化接口，方便在 V2 launch 内收口
- 不直接修改 V1 现有 PoseStamped 主链行为
- 其中 `manual_take_over` 和 `odom_visualization` 已具备 V2 可选 launch 接入能力

## 5. 仍保留在 Utils 参考态的包

当前仍保留在 `Utils/`、未迁入 `src/` 的辅助包包括：

- `uav_utils`
- `quadrotor_msgs`

其中：

- `uav_utils` 和 `quadrotor_msgs` 已有工作区本地版本，禁止整体覆盖
- `manual_take_over` 已改成参数化 topic 入口，方便后续按命名空间接入
- `odom_visualization`、`pose_utils` 已补齐 catkin 依赖与安装规则，但尚未并入默认 V2 启动链

## 6. 验证脚本

V2 当前独立 smoke test：

```bash
./test_swarm_v2_smoke.sh
```

当前脚本覆盖：

- V2 顶层 launch 生成
- 基础 V2 节点图展开
- GoalSet 工具链接入开关
- moving_obstacles 接入开关
- manual_take_over / manual_take_over_station 接入开关
- odom_visualization 接入开关

V2 当前真实运行态健康检查：

```bash
./test_swarm_v2_runtime_health.sh
```

当前脚本覆盖：

- headless 启动 `swarm_top_level_v2.launch`
- `GoalSet` 发布链路
- `/drone_X/odom` 在线
- `/iris_X/mavros/state` 已连接且已解锁
- `/drone_X/traj_server_v2/heartbeat` 在线
- `/drone_X/position_cmd` 在线
- 三机 `ego_planner_v2` / `traj_server_v2` / `px4ctrl` 进程图完整

V2 当前组合场景运行矩阵：

```bash
./test_swarm_v2_runtime_matrix.sh
```

当前矩阵覆盖：

- 默认 V2 主链
- Goal tooling 场景
- moving_obstacles 场景
- manual_take_over + odom_visualization 场景
- 全软件可选栈组合场景

## 7. 关键文件

- `src/clean_uav_core/launch/swarm_top_level_v2.launch`
- `src/clean_uav_core/launch/swarm_uav_runtime_instance_v2.launch`
- `src/clean_uav_core/launch/swarm_goal_tooling_v2.launch`
- `src/clean_uav_core/launch/swarm_moving_obstacles_v2.launch`
- `src/clean_uav_core/config/swarm_planner_v2.yaml`
- `src/clean_uav_core/scripts/swarm_dynamic_commander_v2.py`
- `src/clean_uav_core/scripts/swarm_launch_generator_yaml.py`
- `test_swarm_v2_smoke.sh`
- `test_swarm_v2_runtime_health.sh`
- `test_swarm_v2_runtime_matrix.sh`

## 8. 当前边界

当前已经完成的是：

- V2 包级隔离
- V2 可执行文件隔离
- GoalSet 消息补齐
- V2 最小主链入口
- 第一批 GoalSet 工具迁入
- 第二批 moving_obstacles 迁入
- 第三批辅助包迁入与构建打通
- V2 smoke test
- V2 真实运行态健康检查
- V2 组合场景运行矩阵

当前尚未做的是：

- 更系统的多场景验收矩阵固化
- 第三批辅助包按需并入默认 V2 launch
- 更完整的 RViz 配置整合