# ego-px4ctrl-new 一站式开发者交接手册

本文档的目标不是“介绍项目”，而是让后续接手者在 10 分钟内重新跑通全链路：环境、编译、清理、冒烟、基线复现、日志定位、以及已知风险边界。

## 1. 系统拓扑

```text
Task / Python 调度层
  ├─ 目标发布、起飞/降落触发、任务门控
  ├─ 代表脚本：swarm_dynamic_commander_v2.py / takeoff_land_trigger.py / wait_for_takeoff_and_exec.py
  v
Planning / EGO-Planner V2
  ├─ 里程计输入、GoalSet、B-spline 优化、replan 统计
  ├─ 代表节点：ego_replan_fsm -> traj_server
  v
Control / PX4Ctrl
  ├─ FSM 状态机、AUTO_TAKEOFF / AUTO_HOVER / CMD_CTRL / AUTO_LAND
  ├─ 输入层：odom / imu / rc / cmd / takeoff_land
  v
Sim / PX4 / Gazebo
  ├─ Gazebo model_states -> truth_odom_adapter
  ├─ PX4 SITL -> MAVROS -> mavros/state / mavros/imu / setpoint_raw/attitude
```

更具体的数据流如下：

```text
Gazebo / model_states
  -> truth_odom_adapter.py
  -> /drone_X/odom (或 /truth_odom)
  -> ego_planner_v2 / px4ctrl
  -> traj_server.py / PositionCommand
  -> px4ctrl FSM
  -> MAVROS attitude / bodyrate setpoint
  -> PX4 SITL
```

## 2. 10 分钟快速复现

下面是交接后的最短复现路径。所有 ROS / catkin 命令都必须先 source 环境。

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh
```

```bash
catkin_make -j4
```

```bash
./cleanup_swarm.sh
```

```bash
./test_swarm_smoke.sh
```

```bash
./test_swarm_v2_smoke.sh
```

```bash
./test_swarm_v2_runtime_health.sh
```

如果以上三层都通过，说明“源码、launch、命名空间、MAVROS、planner、controller”这条链路是完整的。

## 3. 当前最重要的基线

### 3.1 当前最稳的 3 机避障基线

结论：当前最稳的 Stage G 3 机基线是 g11_horizon15。

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh
/usr/bin/python3 tools/benchmark_research/run_benchmark_campaign.py --phase stage_g --case g11_horizon15
```

这个 case 的关键点：

- planning_horizon = 15m
- astar_step_factor = 6.0
- astar_pool_size = 40
- local_update_range = 15m
- max_vel = 10m/s

实测上，它比 30m horizon 的基线稳定得多，三架机的 tracking_error 都能压到亚米级。

### 3.2 10m/s 极速模式的 Headless 基线

结论：当前最稳的 10m/s headless 基线是 stage_f 的 f_10p0_inf_330。

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh
/usr/bin/python3 tools/benchmark_research/run_benchmark_campaign.py --phase stage_f --case f_10p0_inf_330
```

如果你只想直接看 launch，不通过 benchmark runner，也可以使用对应生成的 headless launch，但日常推荐先走 benchmark runner，因为它会把日志、会话目录和健康检查一起收口。

## 4. 关键命令清单

下面这 10 条是目前最常用、最关键的命令。建议接手者先记住它们，而不是先记内部实现。

| # | 命令 | 作用 |
|---|---|---|
| 1 | `source tools/source_phase1_env.sh` | 必做环境初始化，ROS / PX4 / Gazebo 路径全部依赖它 |
| 2 | `catkin_make -j4` | 全工作区编译 |
| 3 | `./cleanup_swarm.sh` | 启动前清理残留进程、锁文件和源码树垃圾 |
| 4 | `./test_swarm_smoke.sh` | 单机 / 双机 / 3 机基础冒烟 |
| 5 | `./test_swarm_v2_smoke.sh` | V2 沙盒冒烟，覆盖可选功能节点 |
| 6 | `./test_swarm_v2_runtime_health.sh` | V2 头less 运行健康检查 |
| 7 | `./monitor_swarm.sh` | 运行期间查看 swarm 进程、话题和状态 |
| 8 | `tools/benchmark_research/run_benchmark_campaign.py --phase stage_g --case g11_horizon15` | 当前最稳 3 机避障基线 |
| 9 | `tools/benchmark_research/run_benchmark_campaign.py --phase stage_f --case f_10p0_inf_330` | 当前最稳 10m/s headless 基线 |
| 10 | `tools/benchmark_research/collect_campaign_index.py` | 新 run 完成后刷新 campaign_index |

额外常用的回放命令：

```bash
tools/benchmark_research/replay_benchmark_rosbag.sh --input benchmark_artifacts/research_profiles/stage_g/g11_horizon15/run_01/
```

## 5. 调参红线

### 5.1 hover_percentage 是起飞生命线

`hover_percentage` 不是一个“可以随便试试”的参数，它直接决定 PX4Ctrl 在实机起飞时能不能把重力抵住。

- 太低：起飞阶段会在地面附近反复挣扎，容易进入“推力不够但控制器持续输出”的伪稳定状态。
- 太高：会导致离地瞬间过冲，进而放大姿态瞬态和高度超调。

实机交接时，这个参数应被视为生命线参数，优先于所有“更快、更激进”的追求。

### 5.2 10m/s 下 planning_horizon 与物理转弯半径解耦

`planning_horizon` 不是“转弯半径”，它是局部目标搜索窗口和局部轨迹重规划视距。

在 10m/s 场景下，它必须和机体可实现的物理转弯能力解耦理解：

- horizon 太长，不会让飞机“真的能转更大圈”，只会让规划器把目标看得太远，优化过程更保守、更滞后。
- horizon 太短，又会把局部目标窗口压得过小，容易触发频繁重规划和局部抖动。

当前经验值：

- 30m：太长，Stage G 里会显著放大 tracking error。
- 15m：当前最稳的折中点。
- 10m：能明显提升局部响应，但某些机体 / 场景会出现 false goal_reached 或高频重规划问题。

## 6. 运行顺序建议

### 单机

1. `source tools/source_phase1_env.sh`
2. `catkin_make -j4`
3. `./cleanup_swarm.sh`
4. `./test_swarm_smoke.sh`

### 多机 / 3 机 / Stage G

1. 先启动仿真层
2. 再启动 planner / controller / trigger 层
3. 最后看 runtime health 和 benchmark 输出

### 典型排障顺序

1. 先看 `/iris_X/mavros/state` 是否 connected
2. 再看 `/drone_X/odom` 是否稳定
3. 再看 `/drone_X/position_cmd` 是否持续产生
4. 最后看 planner 日志里的 `planner_success` / `planner_latency_ms`

## 7. Known Issues

这些不是“理论缺陷”，而是目前已经踩过或复现过的真实风险：

- 高频重规划会抬高 CPU 瞬时负载，尤其是在 Stage G 这种多机密集目标环境里。
- `planning_horizon` 对 10m/s 场景非常敏感，过长会明显放大保守路径和跟踪误差。
- `goal_reached` 与真实 odom 位置之间仍可能出现“假完成”现象，尤其在局部窗口过短或目标切换过快时。
- `swarm_dynamic_commander_v2` 过去存在目标发布与 planner 订阅的竞态，现已加 subscriber grace period，但接手时仍要保留这种意识。
- `traj_server` 对 heartbeat 和 trajectory finite 值的防线很重要，空轨迹或 NaN / Inf 不能进入控制器。
- `PX4Ctrl` 的输入层必须继续保持“坏消息拒绝进入 FSM”的原则，不能把异常 odom / imu / cmd 直接传播到状态机。
- 10m/s 快速模式下，`hover_percentage`、`inflation`、`clearance`、`planning_horizon` 之间是耦合的，不要单独放大其中一个就期待线性改善。

## 8. 推荐的开发边界

如果你继续接手开发，优先顺序建议如下：

1. 先保持 g11_horizon15 作为 Stage G 新基线，不要回退到 30m horizon 重新摸索。
2. 优先做输入层鲁棒性和日志闭环，而不是先改控制增益。
3. 如果要继续压 tracking error，先从 `planning_horizon` / `local_update_range` / `inflation` 三者联动下手。
4. 如果要继续追求实机稳定，先把 `hover_percentage`、起飞流程、以及失联处理做成固定流程。

## 9. 关键脚本使用文档

### 9.1 swarm_launch_generator.py - Launch 文件动态生成

这个脚本根据一个描述无人机位置的 Markdown 文件，动态生成 Phase1/Phase2 的 launch 文件。

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
python3 src/clean_uav_core/scripts/swarm_launch_generator.py \
  --version v2 \
  --config docs/uav_position_goal.md \
  --output /tmp/my_launch.launch
```

**输入文件格式** - `docs/uav_position_goal.md`：
```markdown
## Phase1 Drone Definition (Three UAVs)

### Drone ID: iris_0
- Start Position: (-5, 0, 1)
- Goal Position: (5, 0, 1)

### Drone ID: iris_1
- Start Position: (0, -3, 1)
- Goal Position: (0, 3, 1)

### Drone ID: iris_2
- Start Position: (5, 0, 1)
- Goal Position: (-5, 0, 1)
```

**关键参数**：
- `--version v2`：V2 版本（支持 benchmark 环境检测、多机协作）
- `--version v1`：遗留 V1 版本（仅基础 3 机）
- `--config`：Markdown 配置文件路径
- `--output`：生成的 launch 文件目标路径
- `--benchmark_enable`：可选，启用 benchmark 工具链

**输出产物**：
- 完整的可运行 launch 文件
- 自动绑定的 Gazebo 启动参数、MAVROS 配置、planner 参数
- 多机场景下的命名空间分隔（`/iris_0`, `/iris_1`, 等）

### 9.2 swarm_dynamic_commander_v2.py - 多机目标发布与同期化

这是 Phase2+ 的核心任务调度引擎，负责发布 GoalSet 并触发规划：

```bash
rosrun clean_uav_core swarm_dynamic_commander_v2.py \
  --uav_list iris_0,iris_1,iris_2 \
  --goal_file goals.json \
  --rate 1.0
```

**关键逻辑**：
- 订阅 `/drone_X/odom` 和 `/drone_X/traj_server_ready` 以等待就绪
- 按配置时间间隔发布 `/drone_X/goal_set`（一次发布包含多个路径点）
- 每次发布前检查订阅者接收就绪（grace period = 1.0s）
- 追踪每个 UAV 的 `goal_reached` 反馈，用于任务门控

**重要参数**：
- `--uav_list`：逗号分隔的 UAV 名称列表
- `--goal_file`：目标坐标 JSON 文件
- `--rate`：目标发布频率（Hz），通常 1.0 足够
- `--disable_trigger`：可选，禁用自动触发，改为外部触发

### 9.3 takeoff_land_trigger.py - 自动起降流程

这是进入 CMD_CTRL 状态前的前置条件。它触发 px4ctrl 的 AUTO_TAKEOFF 和 AUTO_LAND 状态：

```bash
rosrun clean_uav_core takeoff_land_trigger.py \
  --target_height 2.0 \
  --uav_list iris_0,iris_1,iris_2 \
  --auto_land_after 60.0
```

**事件时序**：
1. 订阅 `/iris_X/mavros/state`，确保 connected 且 armed
2. 发送 `/drone_X/takeoff_land` 消息，进入 AUTO_TAKEOFF 状态
3. 等待达到 `target_height`（通过 odom 检查）
4. 转向 AUTO_HOVER，此时 planner 可以开始规划
5. 规划完成后可进入 CMD_CTRL
6. 如果设置 `auto_land_after`，在超时后自动发送降落命令

**关键参数**：
- `--target_height`：米为单位，推荐 2.0-3.0m
- `--uav_list`：多机场景必须列出所有 UAV
- `--auto_land_after`：自动降落超时（秒），0 表示不自动降落
- `--wait_for_odom`：等待 odom 稳定的超时时间（秒），推荐 3-5s

### 9.4 traj_start_trigger.py - 规划与控制触发

这个脚本在 planner 初始化完成后发送触发信号，让 ego_planner_fsm 进入 PLANNING 状态：

```bash
rosrun clean_uav_core traj_start_trigger.py \
  --uav_list iris_0,iris_1,iris_2 \
  --delay 2.0
```

**工作流**：
1. 等待所有 UAV 的 `/drone_X/ego_plan_fsm/frame` topic 可用（说明 fsm 已就绪）
2. 等待 planner 的全局地图初始化完成
3. 延迟 N 秒后发送触发消息到 `/traj_start_trigger`
4. planner 收到触发后从 INIT → WAIT_TARGET → PLANNING → TRACKING 转移

**关键参数**：
- `--uav_list`：同时触发多个 UAV 的规划
- `--delay`：起降和规划之间的延迟（秒）
- `--timeout`：等待 fsm 就绪的超时时间

### 9.5 swarm_mission_manager.py - 关键词搜索与任务编排

这是新增的高级任务管理器（仅在 benchmark 或研究场景中启用），用于多机合作搜索目标区域：

```bash
rosrun clean_uav_core swarm_mission_manager.py \
  --mission_config mission_profiles/cooperative_search.yaml \
  --enable_coverage_tracking
```

**核心功能**：
- 维护全局搜索覆盖地图，防止重复探索
- 监听所有 UAV 的 `/drone_X/goal_reached` 和 `/drone_X/target_detected`
- 根据集群任务状态（WAIT_READY → SEARCHING → TRACKING → RETURNING → DONE）转移
- 每个 UAV 失败的线段会被记录，下次避开（路由记忆机制）
- 定期输出覆盖率和目标检测统计

**关键参数**：
- `--mission_config`：YAML 格式的任务配置文件
- `--enable_coverage_tracking`：启用 2D 网格覆盖统计
- `--target_detection_sensor`：目标检测传感器来源（camera / lidar / fusion）

### 9.6 run_benchmark_campaign.py - 批量基准测试

这是基准测试框架的主入口，用于标准化 case 的自动化运行：

```bash
python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_g \
  --case g11_horizon15 \
  --num_runs 3 \
  --output_dir benchmark_artifacts
```

**工作流**：
1. 查找 `benchmark_profiles/stage_g/g11_horizon15/config.yaml`
2. 为每个 run（默认 3 次独立运行）生成带时间戳的会话目录
3. 调用 `test_swarm_v2_runtime_health.sh` 来启动对应基线的仿真、planner、controller
4. 实时收集日志：planner replan count、controller tracking error、swarm 间通信延迟
5. 运行结束后自动清理进程，生成 `run_spec.json` 和 metrics CSV
6. 最后调用 `collect_campaign_index.py` 聚合到全局索引

**关键参数**：
- `--phase stage_g|stage_f|...`：研究阶段标识
- `--case g11_horizon15|...`：配置 case 名称
- `--num_runs`：重复运行次数（用于统计稳定性）
- `--output_dir`：所有 run 产物的根目录
- `--timeout_per_run`：每个 run 的最大执行时间（秒）

**输出产物**：
```text
benchmark_artifacts/research_profiles/stage_g/g11_horizon15/
├── run_01/
│   ├── run_spec.json          # 参数配置快照
│   ├── metrics.csv            # 核心指标时间序列
│   ├── planner_debug.log      # 规划器日志
│   ├── controller_debug.log   # 控制器日志
│   └── full_rosbag.bag        # 完整 bag 供回放
├── run_02/
├── run_03/
└── campaign_index.json        # 三个 run 的聚合统计
```

---

## 10. 关键 ROS Topic 与消息格式

### 10.1 核心 Odometry & Control Topics

| Topic | 类型 | 源 | 用途 | 频率 |
|-------|------|-----|------|------|
| `/iris_X/odometry/filtered` 或 `/drone_X/odom` | `nav_msgs/Odometry` | Gazebo / truth_odom_adapter | 无人机位置速度反馈 | 50Hz |
| `/iris_X/mavros/state` | `mavros_msgs/State` | PX4 / MAVROS | 连接、锁定、模式状态 | 10Hz |
| `/iris_X/mavros/imu/data` | `sensor_msgs/Imu` | PX4 IMU / MAVROS | 加速度、角速度、四元数 | 200Hz |
| `/drone_X/goal_set` | `quadrotor_msgs/GoalSet` | swarm_dynamic_commander | 路径点集（目标列表）| 1Hz |
| `/drone_X/position_cmd` | `quadrotor_msgs/PositionCommand` | traj_server (from planner) | 期望位置、速度、加速度、yaw | 50Hz |
| `/iris_X/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | px4ctrl | 期望姿态、角速率、油门 | 50Hz |
| `/drone_X/takeoff_land` | `quadrotor_msgs/TakeoffLand` | takeoff_land_trigger.py | 起飞/降落命令 | 触发型（1-2Hz） |
| `/traj_start_trigger` 或 `/drone_X/traj_start_trigger` | `std_msgs/Bool` | traj_start_trigger.py | 触发规划器开始规划 | 触发型（一次） |

### 10.2 Debug 与状态反馈 Topics

| Topic | 类型 | 源 | 用途 |
|-------|------|-----|------|
| `/drone_X/ego_plan_fsm/frame` | `quadrotor_msgs/PlannerState` | ego_planner_fsm | 规划器 FSM 状态（INIT/WAIT_TARGET/PLANNING/TRACKING） |
| `/drone_X/goal_reached` | `std_msgs/Bool` | ego_planner_fsm | 目标是否到达信号 |
| `/drone_X/px4ctrl/debug` | `quadrotor_msgs/Px4ctrlDebug` | px4ctrl | 控制器调试信息（误差、推力等） |
| `/drone_X/trajectory` | `quadrotor_msgs/Trajectory` | traj_server | 当前规划的轨迹段（B-spline) |
| `/drone_X/vision/detected_targets` | `geometry_msgs/PoseArray` | VLM bridge / detection node（可选） | 检测到的目标位置 |

---

## 11. Debug 与问题排查指南

### 11.1 启动检查清单

启动仿真前，确认以下几点：

```bash
# 终端 1：环境确认
source tools/source_phase1_env.sh
echo $ROS_PACKAGE_PATH  # 应该包含 ego-px4ctrl-new, PX4_Firmware, mavlink_sitl_gazebo
which gazebo                # 应该能找到
which px4                   # 应该能找到

# 终端 2：ROS Master 启动
rosmaster  # 或者 roscore（如果 launch 没有自动启动 roscore）

# 终端 3：查看仿真是否正常启动
rostopic list | grep -E "(iris_0|model_states)" 
# 应该看到 /gazebo/model_states, /iris_0/mavros/state 等
```

### 11.2 常见卡点与排查

**问题 1：launch 启动失败，找不到 MAVROS / PX4 模块**

排查步骤：
```bash
# 1. 确认环境变量
echo $ROS_PACKAGE_PATH | grep -E "(PX4_Firmware|mavlink_sitl_gazebo)"

# 2. 检查 MAVROS 是否安装
dpkg -l | grep mavros

# 3. 重新 source
source tools/source_phase1_env.sh
source /opt/ros/noetic/setup.bash  # 确保系统 ROS 也 source 了

# 4. 重新编译本工作区
catkin_make -j4
```

**问题 2：UAV 不起飞，px4ctrl 卡在 MANUAL_CTRL 状态**

排查步骤：
```bash
# 1. 检查连接状态
rostopic echo /iris_0/mavros/state -n 1
# 应该看到 connected: true, armed: true

# 2. 检查 odom 是否到达
rostopic echo /drone_0/odom/pose/pose/position -n 1
# 应该看到 z 坐标稳定在 0 附近（地面高度）

# 3. 检查 takeoff_land 消息是否被 px4ctrl 接收
rostopic echo /drone_0/takeoff_land -n 1
# 应该看到消息流入，通常是 action: 0（takeoff）

# 4. 查看 px4ctrl 日志
# launch 的 px4ctrl 终端应该显示："[px4ctrl] FSM: MANUAL_CTRL -> AUTO_TAKEOFF"
# 如果没有，说明消息没有被处理或状态机锁住

# 5. 强制重置 px4ctrl（如果是锁定）
rostopic pub /drone_0/takeoff_land quadrotor_msgs/TakeoffLand "action: 0" -1
```

**问题 3：起飞后不移动，position_cmd 收不到或不变化**

排查步骤：
```bash
# 1. 规划器是否就绪
rostopic echo /drone_0/ego_plan_fsm/frame -n 1
# 应该看到状态序列：INIT -> WAIT_TARGET -> PLANNING -> TRACKING

# 2. 检查目标是否发布
rostopic echo /drone_0/goal_set -n 1
# 应该看到坐标数组（3 维点阵）

# 3. 检查规划是否成功
rostopic hz /drone_0/position_cmd
# 应该看到 ~50Hz 的消息流，如果是 0Hz 说明规划还没开始或失败

# 4. 检查地图初始化
# 查看 ego_planner launch 终端，应该有 "[ego_planner] VoxelGrid initialized" 消息

# 5. 手动触发规划（如果自动触发失败）
rostopic pub /traj_start_trigger std_msgs/Bool "data: true" -1
```

**问题 4：多机场景，某个 UAV 的命名空间错误或通信延迟大**

排查步骤：
```bash
# 1. 查看所有活跃的命名空间
rostopic list | grep -E "^/iris_|^/drone_" | sort | uniq

# 2. 检查多机间的通信延迟
# 在 monitor_swarm.sh 中查看 topic 列表，对比时间戳

# 3. 查看 launch 是否正确分配了命名空间
# 查看生成的 launch 文件：cat /tmp/my_launch.launch | grep -A2 "ns=\""

# 4. 强制清理并重启
./cleanup_swarm.sh
sleep 3
roslaunch clean_uav_core phase1_fullstack.launch
```

---

## 12. 配置文件详解

### 12.1 Phase1 px4ctrl 配置（无 RC 模式）

**文件位置**：`src/clean_uav_core/config/phase1_px4ctrl_no_rc.yaml`

```yaml
mass: 1.5                    # 无人机实际质量（kg）
gravity: 9.8

# 关键！这个决定了起飞能否成功
hover_percentage: 0.58       # 悬停油门百分比，1.2kg 在仿真里一般是 55-60%
                            # 实机需要通过低油门测试来校准

# 自动起降参数
auto_takeoff_land:
  no_RC: true               # 启用无 RC 模式（mavros 代替遥控）
  takeoff_height: 2.0       # 起飞目标高度（米）
  motor_idle_sec: 2.8       # 电机怠速时间（秒），给转动稳定
  motor_wait_down: 0.5      # 降落时等待电机停止的时间
  
# 位置环增益（Kp）
gain:
  Kp0: 10.0  # x 方向
  Kp1: 10.0  # y 方向
  Kp2: 20.0  # z 方向（竖直方向通常更高）
  
# 速度环增益（Kv）
  Kv0: 5.0   # x 速度
  Kv1: 5.0   # y 速度
  Kv2: 8.0   # z 速度

# 关键！消息超时检测
msg_timeout:
  odom_timeout: 1.0         # 1 秒无 odom 消息则警报
  imu_timeout: 0.5          # 0.5 秒无 IMU 则警报
  cmd_timeout: 1.0          # 1 秒无 position_cmd 则警报
  # 触发超时时，px4ctrl 会自动回到 AUTO_HOVER 状态（安全降级）
```

### 12.2 Phase2 规划器参数

**嵌入位置**：launch 文件中的 `<param>` 节点或独立 YAML

```yaml
# EGO-Planner 规划约束
planner:
  max_vel: 10.0             # 最大速度（m/s）
  max_acc: 2.0              # 最大加速度（m/s^2）
  
  # ===== 关键：planning_horizon ====
  planning_horizon: 15.0    # 局部规划视距（米）
  # 太长 (30m)：保守、滞后、tracking error 大
  # 太短 (10m)：激进、局部抖动、false goal_reached
  # 当前折中：15m

  # 局部更新范围
  local_update_range: 15.0  # 当 drone 进入此范围后触发 replan（米）

# A* 路径搜索参数
astar:
  step_factor: 6.0          # 栅格搜索步长因子（与 grid 分辨率联动）
  pool_size: 40             # 搜索缓冲池大小
  
# 空间膨胀（障碍物安全距离）
inflation:
  obstacle: 0.3             # 与静态障碍物的最小距离（米）
  agent: 0.5                # 与其他 UAV 的最小距离（米）
  
# VoxelGrid（3D 栅格地图）
grid_map:
  resolution: 0.1           # 栅格单元边长（米），0.1m 精度较好，但计算量大
  origin: [-10, -10, 0]     # 地图原点（世界坐标）
  size: [20, 20, 5]         # 地图大小（20x20x5 米）
```

---

## 13. 性能指标说明

运行基准测试后，常见的输出指标含义如下：

| 指标 | 单位 | 正常范围 | 说明 |
|------|------|--------|------|
| `tracking_error` | m | 0.1-0.5 | 实际位置与 position_cmd 目标的均方误差，越小越好 |
| `replan_count` | count | 5-20 | 任务期间总重规划次数，太高说明环境变化快或 horizon 太短 |
| `replan_latency` | ms | 50-200 | 单次规划耗时，通常与场景复杂度成正比 |
| `flight_time` | s | task-dependent | 完成任务总耗时，受最大速度和路径长度影响 |
| `jerk_integral` | (m/s^3)·s | 1-10 | 加加速度积分，反映轨迹平滑度，越小越舒适 |
| `collective_coverage` | % | >90 | 多机搜索场景下的覆盖面积百分比 |
| `max_goal_distance` | m | task-dependent | 起点到目标的距离 |
| `collision_avoidance_events` | count | 0 | 碰撞避免事件数，应为 0 |

---

## 14. 新增功能模块（Phase 2+ 扩展）

### 14.1 Benchmark 工具链 - 标准化性能测试

这个工作区新增了完整的基准测试框架，用于量化不同参数配置的性能差异。

**文件结构**：
```
tools/benchmark_research/
├── benchmark_profiles/          # 预定义的配置 case
│   ├── stage_g/
│   │   └── g11_horizon15/
│   │       └── config.yaml      # 参数配置
│   └── stage_f/
│       └── f_10p0_inf_330/
│           └── config.yaml
├── run_benchmark_campaign.py    # 主入口（运行一个 case 的所有 runs）
├── run_single_profile.sh         # 单个 run 的 shell wrapper
├── collect_campaign_index.py    # 聚合多个 runs 的统计数据
├── parameter_sensitivity_analysis.py   # 参数敏感度分析（Pearson 相关系数）
├── pareto_frontier_analysis.py         # Pareto 前沿提取
├── risk_statistics_analysis.py         # 风险等级评分
└── replay_benchmark_rosbag.sh           # 回放 bag 车辆
```

**快速开始 - 运行一个基线**：
```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh

# 运行 Stage G 的 g11_horizon15 case，3 个独立 run
python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_g \
  --case g11_horizon15 \
  --num_runs 3

# 运行完后，查看聚合结果
cat benchmark_artifacts/research_profiles/stage_g/g11_horizon15/campaign_index.json
```

**输出结构**：
```
benchmark_artifacts/research_profiles/stage_g/g11_horizon15/
├── run_01/
│   ├── run_spec.json            # 本次 run 的参数快照和命令行信息
│   ├── metrics.csv              # 时间序列指标（tracking_error, replan_count 等）
│   ├── planner_debug.log        # planner 节点的完整日志输出
│   ├── controller_debug.log     # px4ctrl 节点的完整日志输出
│   ├── full_rosbag.bag          # 完整的 ROS bag，用于离线分析
│   └── health_check.json        # 本 run 的健康检查结果（是否成功、错误摘要）
├── run_02/
├── run_03/
└── campaign_index.json          # 3 个 run 的均值、标准差、min/max 统计
```

**后续分析**：
```bash
# 参数敏感度分析（找哪个参数影响最大）
python3 tools/benchmark_research/parameter_sensitivity_analysis.py \
  --campaign_dir benchmark_artifacts/research_profiles/stage_g/g11_horizon15

# Pareto 前沿分析（flight_time vs jerk_integral）
python3 tools/benchmark_research/pareto_frontier_analysis.py \
  --campaign_dir benchmark_artifacts/research_profiles/stage_g/g11_horizon15 \
  --objective1 flight_time \
  --objective2 jerk_integral

# 风险评分（哪些 case 有碰撞或失败风险）
python3 tools/benchmark_research/risk_statistics_analysis.py \
  --campaign_dir benchmark_artifacts/research_profiles/stage_g/g11_horizon15
```

### 14.2 VLM Bridge - 视觉语言模型目标定位

这是可选的高阶功能，用于将摄像头像素坐标投影到世界坐标，支持手动像素点击或 Grounding DINO 自动检测。

**启用方式**（仅在 launch 中设置）：
```xml
<!-- swarm_uav_runtime_instance.launch 中 -->
<arg name="enable_vlm_bridge" default="false"/>
<group if="$(arg enable_vlm_bridge)">
  <include file="$(find clean_uav_core)/launch/swarm_vlm_bridge_instance.launch">
    <arg name="uav_id" value="$(arg uav_id)"/>
  </include>
</group>
```

**使用流程**：
```bash
# 启动带 VLM bridge 的 launch
roslaunch clean_uav_core phase2_dual_uav_stack.launch enable_vlm_bridge:=true

# 在另一个终端启动 VLM bridge Python 节点
rosrun clean_uav_core vlm_bridge.py \
  --uav_list iris_0,iris_1 \
  --vlm_mode grounding_dino  # 或 manual_click

# 发布目标（通过 VLM bridge 的像素坐标投影）
# 实际目标会被发布到 /drone_X/vision/detected_targets
```

**关键参数说明**：
- `--vlm_mode grounding_dino`：自动目标识别（需要 DINO 模型）
- `--vlm_mode manual_click`：交互式像素点击，要求用户在图像上标记目标
- `--depth_sensor depth_camera`：指定深度传感器类型
- `--camera_intrinsics path/to/intrinsics.yaml`：相机内参文件

### 14.3 Mission Manager - 多机合作搜索编排

这是高级功能，用于多机群体搜索和坐标协作。目前只在研究场景中启用。

**初始化**：
```bash
rosrun clean_uav_core swarm_mission_manager.py \
  --mission_config mission_profiles/cooperative_search.yaml \
  --uav_list iris_0,iris_1,iris_2 \
  --enable_coverage_tracking \
  --log_level debug
```

**核心功能**：
- 维护全局 2D 搜索覆盖栅格，防止多机重复探索
- 动态分配搜索目标给各 UAV，基于当前覆盖差距
- 监听目标检测事件（`/drone_X/target_detected`）和 goal_reached 反馈
- 若某 UAV 在某线段失败（碰撞、规划超时），记录该线段并标记为禁行，其他 UAV 从中吸取教训
- 定期输出覆盖率、平均搜索速度、目标检测统计

**配置示例**（YAML 格式）：
```yaml
mission:
  type: cooperative_search
  search_region: [-20, -20, 20, 20]    # 搜索域
  cell_size: 1.0                       # 覆盖栅格单元大小（米）
  
coordinator:
  allocation_strategy: frontiermax      # 分配策略：frontiermax / minuncovered
  replan_interval: 10                  # 多少秒重新评估分配
  
safety:
  collision_penalty_distance: 2.0       # 避障距离（米）
  max_failure_retries: 3               # 相同线段失败 3 次后放弃
```

---

## 15. 日志分析与故障快速定位

### 15.1 日志目录结构（基准测试场景）

```
benchmark_artifacts/research_profiles/<stage>/<case>/run_01/
├── run_spec.json                          # JSON 格式的参数配置记录
├── metrics.csv                            # 时间序列指标导出
├── planner_debug.log                      # [ego_planner] 日志
├── controller_debug.log                   # [px4ctrl] 日志
├── physics_state.log                      # [sim] 主要物理量变化日志
├── comm_latency.log                       # [swarm_bridge] 多机通信延迟
├── full_rosbag.bag                        # 完整的 ROS bag 文件
└── health_check.json                      # {"status": "PASSED"|"FAILED", "error": "..."}
```

### 15.2 常见故障日志签名

**案例 1：规划超时 / 无可行轨迹**

查看 `planner_debug.log`：
```
[ego_planner] [ERROR] A* search failed: no path found within pool_size=40
[ego_planner] [WARN] Replan FSM: PLANNING -> WAIT_TARGET (recovery timed out)
```

**对策**：
- 增大 `astar_pool_size` 或 `astar_step_factor`
- 增大 `planning_horizon` 给规划器更大视距
- 检查起点/目标是否在障碍物内部

**案例 2：Tracking Error 过大**

查看 `controller_debug.log`：
```
[px4ctrl] [WARN] Position error: x=2.5m, y=1.8m, z=0.9m (threshold=1.0m)
[px4ctrl] [DEBUG] Cmd timeout or zero command, reverting to hover
```

**对策**：
- 检查 `position_cmd` 频率是否稳定（应该是 50Hz）
- 确认 `planning_horizon` 不要太短（容易频繁重规划，导致命令跳跃）
- 调整 PID 增益（Kp / Kv）

**案例 3：多机碰撞**

查看 `health_check.json`：
```json
{
  "status": "FAILED",
  "error": "Collision detected between iris_0 and iris_1 at t=12.5s",
  "collision_count": 2
}
```

**对策**：
- 增大 `inflation.agent`（多机间最小距离）
- 减小 `max_vel` 或增大 `planning_horizon`
- 检查目标是否过于密集（两个 UAV 的目标过近）

### 15.3 快速离线分析工具

**提取 metrics 的特定时间段**：
```python
import pandas as pd

df = pd.read_csv('metrics.csv')
# 只看前 30 秒的 tracking_error
early_phase = df[df['timestamp_s'] < 30]
print(early_phase['tracking_error_m'].describe())
```

**绘制轨迹对比图**：
```bash
# 用 RViz 回放 rosbag 并记最后一张截图
tools/benchmark_research/replay_benchmark_rosbag.sh \
  --input benchmark_artifacts/research_profiles/stage_g/g11_horizon15/run_01/ \
  --output /tmp/traj_compare.png
```

---

## 16. 实机部署前检查清单

如果后续要迁移到真实无人机（仿真 → 实机），以下检查点不能跳过：

### 16.1 参数校准（部署前 2-3 天）

- [ ] **hover_percentage**：在低空 (<2m) 悬停，逐步调整油门百分比直到能稳定悬停
- [ ] **Kp / Kv 增益**：从保守值（50% 取值）开始，逐步提高，监控超调
- [ ] **max_vel 和 max_acc**：从低速 (3 m/s) 开始，逐步提速，监控姿态角和振颤

### 16.2 传感器检查

- [ ] **IMU 漂移**：进行 10 分钟静止漂移测试，Z 轴加速度应稳定在 9.8±0.3 m/s²
- [ ] **Odometry 跳跃**：检查 odom 时间序列是否有不连续或突跳
- [ ] **GNSS / Barometer**：如有外部定位，验证与 local odom 的融合效果

### 16.3 通信与同步

- [ ] **MAVROS 延迟**：测试 `/mavros/state` 更新频率，应稳定在 10Hz
- [ ] **多机时钟同步**：验证多台无人机的 ROS time 偏差 < 100ms
- [ ] **网络吞吐量**：在多机场景下，监控 ROS 消息丢包率 < 1%

### 16.4 安全限制

- [ ] **Geofence**：设置飞行环境的边界，防止逃逸
- [ ] **故障恢复**：测试 MAVROS 连接断开后的自动降落行为
- [ ] **电池管理**：设置电量不足时的自动返回与降落阈值

### 16.5 初飞检查

- [ ] 第一次飞行限制在 2m 高度和 5m 飞行距离
- [ ] 操作手始终保持遥控器就绪，以便紧急制动
- [ ] 记录第一次飞行的日志，供后续对标对比

---

## 17. 常见问题速查表

| 问题 | 快速诊断 | 目标参数 |
|------|---------|--------|
| 起飞卡住 | `rostopic echo /iris_0/mavros/state` → armed=true? | 检查 MAVROS 连接、motors 是否解锁 |
| 不动 | `rostopic hz /drone_0/position_cmd` → 0 Hz? | 等待规划完成，或手动触发 `/traj_start_trigger` |
| 跟踪误差大 | `rostopic echo /px4ctrl/debug` → tracking_error > 1m? | 检查命令频率、PID 增益、规划 horizon |
| 频繁重规划 | `grep "replan" planner_debug.log \| wc -l` > 30? | 增大 `planning_horizon` 或 `local_update_range` |
| 多机碰撞 | health_check.json → collision_count > 0? | 增大 `inflation.agent` 或减速 |
| 消息丢失 | 查看 launch 终端，error 关键字 | 确认 topic 订阅/发布是否正确、频率够不够 |

---

## 19. 开发工作流与持续改进

如果接手后要继续开发或改进，建议按照以下工作流进行：

### 19.1 标准开发流程

```
1. 创建特性分支
   git checkout -b feature/your_feature
   
2. 修改源码（src/ 或 tools/）
   vim src/px4ctrl/src/controller_node.cpp
   
3. 编译并通过基础检查
   source tools/source_phase1_env.sh
   catkin_make -j4
   ./cleanup_swarm.sh
   ./test_swarm_smoke.sh
   
4. 针对你的改动运行相关基准
   source tools/source_phase1_env.sh
   python3 tools/benchmark_research/run_benchmark_campaign.py \
     --phase stage_g --case g11_horizon15 --num_runs 1
   
5. 检查性能指标是否下降（对比上次运行的 campaign_index.json）
   
6. 更新相关文档（如改了参数，要更新本文档的第 12 节）
   
7. 提交并推送
   git add .
   git commit -m "feat: improved planning horizon adaptive control"
   git push origin feature/your_feature
```

### 19.2 分支与发布约定

| 分支名 | 用途 | 稳定性 |
|--------|------|--------|
| `main` | 生产/交接分支，必须通过全套 smoke + benchmark | ✅ 最高 |
| `develop` | 开发集成分支，可能包含小 bug 但通过 smoke | ⚠️ 中等 |
| `feature/*` | 单特性分支，可能无法通过 smoke | ❌ 低 |
| `hotfix/*` | 紧急修复分支，一旦合并立即发布 | ✅ 最高 |

### 19.3 代码审查要点

提交 PR 时，审查者会关注以下几点：

- [ ] 代码是否遵循现有的命名和风格（C++ 类名 CamelCase，变量 snake_case）
- [ ] 是否添加了必要的防错检查（参考第 7 节的 Known Issues）
- [ ] 涉及的 ROS topic 或消息格式是否有向后兼容考虑
- [ ] 新增的参数是否有默认值和说明注释
- [ ] 是否更新了相关文档（如改了 launch 参数要更新本文档）
- [ ] 基准测试性能是否无明显下降（对比运行结果 < 5% 差异可接受）

---

## 20. 故障恢复与灾备

### 20.1 快速恢复步骤（完全重启）

```bash
# 如果整个系统卡顿或出现多个节点不响应
cd /home/guanwen/XTDrone/ego-px4ctrl-new

# 第 1 步：环境重置
source tools/source_phase1_env.sh

# 第 2 步：杀死所有相关进程
./cleanup_swarm.sh
sleep 3

# 第 3 步：重新编译（以防代码在途中被破坏）
catkin_make -j4

# 第 4 步：检查编译是否成功
catkin_make --pkg px4ctrl | grep -i "built target"

# 第 5 步：重新启动冒烟测试
./test_swarm_smoke.sh

# 如果仍然失败，查看 build/CMakeFiles/CMakeError.log
```

### 20.2 备份关键数据

```bash
# 备份现有的基准测试成果
tar czf benchmark_artifacts_backup_$(date +%Y%m%d_%H%M%S).tar.gz \
  benchmark_artifacts/

# 备份 launch 生成产物
tar czf generated_launch_backup.tar.gz \
  src/clean_uav_core/launch/generated_*.launch

# 定期备份（建议周备份一次）
# 使用 git 作为主要版本控制，tar 作为物理备份
```

### 20.3 数据丢失恢复

如果不小心删除了关键文件：

```bash
# 1. 查看 git log
git log --all --oneline | head -20

# 2. 恢复到特定 commit
git checkout <commit_hash> -- path/to/file.cpp

# 3. 如果整个 commit 被删除，从 reflog 找回
git reflog
git checkout <reflog_hash>
```

---

## 21. 团队协作与沟通

### 21.1 日常同步方式

- **周一例会**：汇报上周进度与本周计划
- **日志分享**：有重要发现（如新的 bug 模式）时及时分享 run log 和 health_check.json
- **参数变更通知**：如果改动了基准参数（hover_percentage、planning_horizon 等），需要通知所有使用者

### 21.2 问题上报格式

遇到 bug 时，请包含以下信息：

```
标题：[BUG] px4ctrl 在 10m/s 高速下姿态超调过大

描述：
- 基线：stage_g / g11_horizon15
- 复现步骤：
  1. 启动仿真
  2. 起飞至 2m 高度
  3. 发送目标让 UAV 以 10m/s 最大速度移动
- 现象：UAV 俯仰角超过 45 度，出现明显振颤
- 日志文件：benchmark_artifacts/research_profiles/.../run_01/full_rosbag.bag

期望行为：
- UAV 应该以平稳的曲线轨迹到达目标，最大俯仰角 < 30 度

可能原因分析：
- Kp/Kv 增益设置过高
- planning_horizon 太短导致频繁重规划
```

---

## 22. 版本与兼容性说明

### 22.1 系统版本对应关系

| 版本 | ROS | Python | Gazebo | PX4 Firmware | 特性 |
|------|-----|--------|--------|--------------|------|
| Phase1 | Noetic | 3.8 | 11.x | v1.13.x | 单机演示 |
| Phase2 | Noetic | 3.8 | 11.x | v1.13.x | 多机无避障 |
| Phase3 | Noetic | 3.8 | 11.x | v1.13.x | 多机 + VINS |
| Phase4 | Noetic | 3.8 | 11.x | v1.13.x | 动态目标 + benchmark |

### 22.2 已知限制

- **最大 UAV 数**：当前测试过的最大规模是 6 机，超过此数字可能出现通信延迟和 CPU 瓶颈
- **规划分辨率**：Grid map 分辨率 0.1m 是折中点，更精细的分辨率会显著增加内存占用（5000+ MB）
- **Gazebo 仿真速度**：在 4 核 CPU + 8GB RAM 机器上，4 机场景的实时因子约为 0.8-0.9（有轻微拖累）

### 22.3 向后兼容性

- 本工作区严格保持 `src/quadrotor_msgs/` 的本地版本，不会迁移到系统 apt 包
- 所有新增功能（benchmark、VLM 等）都是可选的，不影响现有的 Phase1/Phase2 基础流程
- 如果发现兼容性问题，优先在本文档的第 7 节（Known Issues）记录，而不是立即推送大型重构

---

## 23. 参考文档
