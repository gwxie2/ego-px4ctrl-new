# drone_0 回放异常诊断与修复说明

日期：2026-04-07

## 1. 问题描述

在回放以下实验时：

- benchmark_artifacts/research_profiles/stage_d/d_rigid_clearance_balance/run_01

观察到边缘位置的 drone_0 在 RViz 中表现为“前几秒原地打转，随后才慢慢向前飞”，而另外两架无人机没有同样现象。

本次工作的目标是确认该现象属于：

1. 真实飞行行为异常
2. odom 本身异常
3. RViz 回放表达方式误导

## 2. 结论

结论是：

- 不是 odom 不动
- 不是 drone_0 前几秒完全原地打转
- 根因是旧 RViz 配置使用了 rviz/Odometry 的历史箭头显示，姿态变化在低位移阶段被画成一团扇形，看起来像“原地自转”

也就是说，这是一个回放可视化问题，不是这架无人机在 bag 中真实静止。

## 3. 证据

### 3.1 run_04 的 rosbag 直接证据

对以下 bag 直接读取：

- benchmark_artifacts/research_profiles/stage_d/d_rigid_clearance_balance/run_04/auto_20260407_004122/drone_0_vel10_20260407_004122.bag

在 bag 起始段，drone_0 的 odom 位置已经连续变化，例如：

- x: 12.9683 -> 12.9493
- y: -7.4864 -> -7.4758
- z: 1.1435 -> 1.1458

这说明机体一开始就在发生平移，而不是停在原地。

同一时间段，position_cmd 也已经在前进：

- x: 12.9433 -> 12.8768
- y: -7.4718 -> -7.4338
- z: 1.1418 -> 1.1456

因此，真实数据里不存在“位置完全不变，只是姿态旋转”的情况。

### 3.2 run_08 的复跑证据

为避免只依赖旧数据，又重新执行了同 case 的 headless 复跑：

- benchmark_artifacts/research_profiles/stage_d/d_rigid_clearance_balance/run_08

drone_0 的 bag 起始段同样显示：

- odom 已连续前进
- position_cmd 也已连续前进

说明这个现象不是 run_01 / run_04 的偶发问题，而是旧 RViz 表达方式对 drone_0 这种“边走边大幅 yaw 变化”的轨迹更容易造成误判。

## 4. 为什么只有 drone_0 更明显

drone_0 位于边侧，首段轨迹方向变化和 yaw 变化相对更明显，而旧 RViz 配置使用的是：

- 历史 Odometry 箭头
- 每一帧都保留姿态箭头

这样在“位移不大但朝向变化较快”的短时间窗内，会堆积出明显的扇形箭头簇。

另外，当前系统本来就存在“规划 yaw 跟随未来轨迹方向”的逻辑，因此首段朝向变化是真实存在的，但它不是“原地打转不走”，而是“边走边转头”。

## 5. 已实施修复

### 5.1 新增 replay 辅助节点

新增文件：

- src/clean_uav_core/scripts/replay_path_publisher.py

作用：

- 订阅 /drone_N/odom
- 订阅 /drone_N/position_cmd
- 发布 RViz 友好的调试话题：
  - /drone_N/replay/actual_pose
  - /drone_N/replay/actual_path
  - /drone_N/replay/command_pose
  - /drone_N/replay/command_path

这样就可以把“真实路径”和“指令路径”拆开看，而不再依赖 Odometry 历史箭头。

### 5.2 更新 replay 脚本

修改文件：

- tools/benchmark_research/replay_benchmark_rosbag.sh

修复内容：

- replay 时自动启动 replay_path_publisher.py
- 回放结束时自动清理该辅助节点

### 5.3 更新 RViz 配置

修改文件：

- src/clean_uav_core/rviz/benchmark_rosbag_replay.rviz

修复内容：

- 不再使用 drone_0/1/2 的历史 Odometry 箭头作为主显示
- 改为：
  - 当前实际姿态：replay/actual_pose
  - 实际飞行路径：replay/actual_path
  - 当前指令姿态：replay/command_pose
  - 指令路径：replay/command_path

修复后的显示可以直接回答三个问题：

1. 实际位置有没有动
2. 指令有没有向前发布
3. 真实路径和指令路径之间差了多少

## 6. 复跑验证

### 6.1 GUI/RViz 路线验证

尝试复跑：

- run_07，对应命令带 visualize

结果：

- 失败
- runtime-health 报错：No position_cmd for drone_0

这与此前 Stage D 已知结论一致：

- 10m/s 下 GUI/RViz 路线仍不稳定
- 这不是本次 drone_0 回放误判问题导致的
- 而是 Stage D 的独立已知限制

### 6.2 Headless 路线验证

随后按 Stage D 当前唯一稳定路径重新执行：

- run_08

结果：

- runtime-health 通过
- output_dir: benchmark_artifacts/research_profiles/stage_d/d_rigid_clearance_balance/run_08

从 campaign_index 聚合结果看，run_08 是一次完成的有效样本：

- stop_reason: all_goals_reached
- avg_replan_success_rate: 0.8226
- min_safety_margin_m: 0.1125
- avg_tracking_error_p95_m: 10.2108
- avg_control_lag_p95_ms: 13595.3
- avg_jerk_integral: 161.27
- total_safety_violation_count: 571

这次 run_08 的整体性能比之前的优秀样本差，主要退化集中在 drone_2，而不是 drone_0。

因此本次问题的核心结论不变：

- drone_0 的“原地打转”不是实际运动层面的主要故障
- 它只是旧回放配置误导了观察

## 7. 现在应如何使用回放

推荐命令：

```bash
source tools/source_phase1_env.sh
tools/benchmark_research/replay_benchmark_rosbag.sh \
  --input benchmark_artifacts/research_profiles/stage_d/d_rigid_clearance_balance/run_08
```

建议在 RViz 中重点看以下话题：

- /drone_0/replay/actual_path
- /drone_0/replay/command_path
- /drone_0/replay/actual_pose
- /drone_0/replay/command_pose

判断规则：

- 如果 actual_path 从一开始就在延伸，说明机体真实在移动
- 如果 command_path 从一开始就在延伸，说明规划输出从一开始就在前推
- 如果两条 path 同向但有偏差，说明是控制跟踪误差，不是“原地不动”
- 如果 actual_path 基本不动而 command_path 明显向前，才是真正的执行层异常

## 8. 最终结论

本次问题已经确认并修复：

- 根因：旧 RViz 用历史 Odometry 箭头显示，导致 drone_0 的姿态变化被误读成“原地打转”
- 修复：新增 replay_path_publisher.py，并将 RViz 主显示切换为 actual_path / command_path / actual_pose / command_pose
- 验证：
  - run_04 和 run_08 的 bag 都证明 drone_0 一开始就在移动
  - 同 case 复跑中，GUI 路线仍失败，这是 Stage D 的既有问题；headless 路线 run_08 成功完成

后续如果再遇到某架机“看起来原地转圈”，应优先先看：

- replay/actual_path
- replay/command_path

而不要先看历史 Odometry 箭头簇。

## 9. 第二阶段根因修复

上面的结论解决了“RViz 误判”这一层问题，但后续进一步排查发现，drone_0 启动段之所以仍然比另外两架机更像“先转后走”，还存在一个真实的控制启动瞬态问题。

### 9.1 新确认的根因

根因不是 odom 不动，而是 V2 轨迹服务器在启动阶段存在两个初始化缺陷：

1. yaw 冷启动错误

- 文件：src/plan_manage_v2/src/traj_server.cpp
- 旧逻辑把 last_yaw_ 固定初始化为 0.0
- 但本场景三架机的真实 init_yaw 分别约为：
  - drone_0: 2.61799
  - drone_1: -3.14159
  - drone_2: -2.61799

这会让首段 yaw 控制从错误朝向起步，drone_0 因为位于边侧、首段方向变化更明显，所以视觉上最容易表现为“先明显转头”。

2. 首次控制时间步初始化错误

- 同文件旧逻辑使用 static ros::Time time_last = ros::Time::now()
- 新轨迹刚开始时，如果直接沿用旧静态时间状态，首次 dt 不是“本轨迹首步”的真实时间差
- 这会把首个 yaw 过渡步放大，进一步加重启动瞬态

### 9.2 本次已实施的代码修复

修改文件：

- src/plan_manage_v2/src/traj_server.cpp
- src/clean_uav_core/launch/swarm_uav_runtime_instance_v2.launch

修复内容：

1. traj_server_v2 新增 odom 订阅

- 通过 ~odom 获取当前真实机体姿态
- 在未开始主动轨迹控制前，用 odom 中的 yaw 同步 last_yaw_

2. 每次收到新轨迹时重置首步时间状态

- polyTrajCallback 中将 cmd_time_initialized_ 置回 false
- cmdCallback 在本轨迹首个有效控制周期把 time_last 重置为当前时间

## 10. 最新验证：drone_2 首轨迹失败已明显缩短

在完成上面的最小修复后，又重新跑了同一 Stage D 验证：

- 运行根目录：[benchmark_artifacts/validation/drone0_sync_run_03](../benchmark_artifacts/validation/drone0_sync_run_03)
- 本次 session：[benchmark_artifacts/validation/drone0_sync_run_03/auto_20260407_223431](../benchmark_artifacts/validation/drone0_sync_run_03/auto_20260407_223431)
- drone_2 planner stdout：[benchmark_artifacts/validation/drone0_sync_run_03/planner_logs/drone0_sync_run_03/drone_2_ego_planner_v2.log](../benchmark_artifacts/validation/drone0_sync_run_03/planner_logs/drone0_sync_run_03/drone_2_ego_planner_v2.log)
- drone_2 replan CSV：[benchmark_artifacts/validation/drone0_sync_run_03/auto_20260407_223431/drone_2_auto_20260407_223431_replan.csv](../benchmark_artifacts/validation/drone0_sync_run_03/auto_20260407_223431/drone_2_auto_20260407_223431_replan.csv)

这次的关键现象是：

- 首次 `SEQUENTIAL_START` 进入后，日志明确打印了“Initial global trajectory failed once ... switching to random init retries”
- 之后只经历了少量启动重试，就进入 `EXEC_TRAJ`
- 和 run_02 相比，drone_2 的启动失败段明显缩短

对比 run_02 和 run_03：

- run_02 中，drone_2 首段连续失败更多，前 13 次 replan 都没有成功，直到 replan 14 才开始进入执行
- run_03 中，drone_2 在第 5 次 replan 就进入 `EXEC_TRAJ`
- run_03 里不再出现 `FAILURE_INIT_FAILED`，只剩少量 `FAILURE_OPTIMIZER_FAILED`

结论是：

- 这次最小修复确实把 drone_2 的启动等待压下来了
- 问题仍然存在少量 A* / optimizer 失败，但已经不再是“长时间卡在首轨迹初始化”
- 目前更像是规划器在高约束场景下的正常短暂试探，而不是启动链路本身的结构性阻塞

## 10. 本次验证运行的位置

这次已经通过的验证是 run_02，对应输出都在工作区下：

- 运行根目录：[benchmark_artifacts/validation/drone0_sync_run_02](../benchmark_artifacts/validation/drone0_sync_run_02)
- 本次 session 目录：[benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159](../benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159)
- manifest：[benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/auto_20260407_173159_manifest.json](../benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/auto_20260407_173159_manifest.json)
- summary：[benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/auto_20260407_173159_summary.json](../benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/auto_20260407_173159_summary.json)
- drone_0 bag：[benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/drone_0_vel10_20260407_173159.bag](../benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/drone_0_vel10_20260407_173159.bag)
- drone_1 bag：[benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/drone_1_vel10_20260407_173159.bag](../benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/drone_1_vel10_20260407_173159.bag)
- drone_2 bag：[benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/drone_2_vel10_20260407_173159.bag](../benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/drone_2_vel10_20260407_173159.bag)
- drone_0 主 CSV：[benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/drone_0_auto_20260407_173159.csv](../benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/drone_0_auto_20260407_173159.csv)
- drone_1 主 CSV：[benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/drone_1_auto_20260407_173159.csv](../benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/drone_1_auto_20260407_173159.csv)
- drone_2 主 CSV：[benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/drone_2_auto_20260407_173159.csv](../benchmark_artifacts/validation/drone0_sync_run_02/auto_20260407_173159/drone_2_auto_20260407_173159.csv)

对应的 ROS 日志目录是：

- roslaunch 主日志：[~/.ros/log/30d57880-3263-11f1-98e8-a9d14e4a3eb7/roslaunch-guanwen-Alpha-17-C7VF-16389.log](../.ros/log/30d57880-3263-11f1-98e8-a9d14e4a3eb7/roslaunch-guanwen-Alpha-17-C7VF-16389.log)
- benchmark manager：[~/.ros/log/30d57880-3263-11f1-98e8-a9d14e4a3eb7/benchmark-benchmark_manager-40.log](../.ros/log/30d57880-3263-11f1-98e8-a9d14e4a3eb7/benchmark-benchmark_manager-40.log)
- swarm commander：[~/.ros/log/30d57880-3263-11f1-98e8-a9d14e4a3eb7/swarm_dynamic_commander_v2-39.log](../.ros/log/30d57880-3263-11f1-98e8-a9d14e4a3eb7/swarm_dynamic_commander_v2-39.log)
- drone_0 planner stdout：[benchmark_artifacts/validation/drone0_sync_run_02/planner_logs/drone0_sync_run_02/drone_0_ego_planner_v2.log](../benchmark_artifacts/validation/drone0_sync_run_02/planner_logs/drone0_sync_run_02/drone_0_ego_planner_v2.log)
- drone_1 planner stdout：[benchmark_artifacts/validation/drone0_sync_run_02/planner_logs/drone0_sync_run_02/drone_1_ego_planner_v2.log](../benchmark_artifacts/validation/drone0_sync_run_02/planner_logs/drone0_sync_run_02/drone_1_ego_planner_v2.log)
- drone_2 planner stdout：[benchmark_artifacts/validation/drone0_sync_run_02/planner_logs/drone0_sync_run_02/drone_2_ego_planner_v2.log](../benchmark_artifacts/validation/drone0_sync_run_02/planner_logs/drone0_sync_run_02/drone_2_ego_planner_v2.log)

本次 run_02 的关键数值：

- drone_0 规划器启动后 3 秒位移：5.29 m
- drone_1 规划器启动后 3 秒位移：2.64 m
- drone_2 规划器启动后 3 秒位移：1.42 m
- drone_0 启动后 3 秒累计偏航转角：约 0.038 rad，未出现 360 度绕转
- drone_0 全程累计偏航转角：约 0.244 rad，未出现多圈旋转


3. launch 增加 odom remap

- 在 swarm_uav_runtime_instance_v2.launch 中，为 traj_server_v2 增加：
  - ~odom -> odom

这样 traj_server_v2 就不再从 0 rad 冷启动，而是从 UAV 当前真实朝向起步，同时首个控制周期也不会继承旧轨迹的时间状态。

## 10. run_09 验证结果

为验证修复是否生效，重新复跑同一 case：

- benchmark_artifacts/research_profiles/stage_d/d_rigid_clearance_balance/run_09

session：

- benchmark_artifacts/research_profiles/stage_d/d_rigid_clearance_balance/run_09/auto_20260407_144157

### 10.1 健康检查结果

- runtime-health 通过
- run_spec 状态为 completed

说明本次代码修改没有破坏 Stage D 既有的 headless 运行链路。

### 10.2 启动 yaw 对齐结果

从 run_09 的 drone_0 bag 首批 /position_cmd 可见：

- 首批命令 yaw 约为 2.618 rad

这已经和 drone_0 的场景初始朝向一致，不再是过去那种“内部 yaw 状态先从 0 开始，再向真实朝向拉回”的冷启动模式。

这意味着本轮修复确实消除了最关键的 yaw 初始化错误。

### 10.3 drone_0 启动段真实运动

run_09 的主 CSV 起始段显示，drone_0 在进入命令控制后的最早若干采样内已经持续发生平移：

- x 从 12.9887 持续下降到 12.9492
- y 从 -7.4994 持续增加到 -7.4762
- z 从 1.1504 持续上升到 1.1642

对应 odom bag 起始段同样可见连续位移，因此修复后依然不存在“位置不动只在原地打转”的情况。

### 10.4 聚合性能对比

run_04 中：

- drone_0 tracking_error_p95 = 0.7316
- drone_0 speed_mean = 1.3305

run_09 中：

- drone_0 tracking_error_p95 = 0.6286
- drone_0 speed_mean = 1.3473

说明在同一 case 下，修复后 drone_0 没有变差，启动后整体表现还有小幅改善。

### 10.5 仍然存在但尚未本轮修复的问题

run_09 的事件时间仍显示三架机不是完全同步进入命令阶段：

- drone_0 首次 6 -> 4: 25.144 s
- drone_1 首次 6 -> 4: 26.079 s
- drone_2 首次 6 -> 4: 26.694 s

这说明多机启动时序差仍然存在，drone_0 仍会更早进入主动轨迹阶段，所以在同一 wall-clock 时刻对比三架机时，它依旧更容易被看到处于“更早的控制阶段”。

但这个问题已经与“yaw 从错误初值冷启动”分离开来：

- 本轮已修掉的是朝向初始化错误和首次时间步初始化错误
- 尚未修的是多机 barrier 级别的真正同步启动

## 11. 更新后的最终结论

截至本轮修复，drone_0 启动异常应分成两层看：

1. RViz 表达误导

- 已通过 replay_path_publisher.py + 新 RViz 配置解决

2. 启动段真实 yaw 瞬态过大

- 已通过 traj_server_v2 从真实 odom 初始化 yaw、并重置新轨迹首步时间状态解决主要根因

当前剩余问题不是“drone_0 特有坏掉”，而是：

- 多机进入轨迹控制仍非完全同步

如果后续要继续收敛启动一致性，下一步应优先处理：

- 让三架机在 takeoff 完成后进入统一 barrier，再同时放行首条主动轨迹

而不是再回头怀疑 odom 或 RViz 本身。