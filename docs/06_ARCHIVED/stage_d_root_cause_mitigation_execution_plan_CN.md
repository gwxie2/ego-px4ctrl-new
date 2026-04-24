# Stage D 10m/s 根因扩展排查与可回退实施计划

## 1. 目标

本轮目标不是继续用放大 A* 搜索池的方式硬顶过 Stage D 10m/s，而是把问题拆成三类并分别验证：

1. 指标口径是否把时戳差误当成真实控制阻塞。
2. V2 规划与执行链是否存在同步阻塞和轨迹供给断流。
3. 地图输入、A* 搜索窗口和优化器失败链是否共同放大了 `drone_2` 的早期失稳。

所有实验都必须满足以下边界：

- 默认行为不变。
- 新功能只通过 launch 参数显式开启。
- 能通过关闭参数或删除新增 case 回退。
- 先保留 V2 主线，再用 V1 对照确认问题是否是 V2 特有。

## 2. 已确认事实

### 2.1 `control_lag_ms` 不是“真实控制线程卡死时长”

[src/clean_uav_core/scripts/benchmark_manager.py](src/clean_uav_core/scripts/benchmark_manager.py) 当前用的是 `odom_stamp` 与 `command_stamp` 的绝对差。它可以反映消息时间基不一致、命令陈旧或消息流断续，但不能直接证明控制线程真的被阻塞了同等时长。

因此后续所有 Phase 1 结论都必须结合以下证据同时看：

- `PositionCommand` 是否持续发布
- odom 与 command 的 header 时间是否来自同一时间基
- planner replan 期间是否出现明显停流
- MAVROS / PX4 是否报告 time jump 或 offboard 异常

### 2.2 V2 的重点嫌疑点不是“时间戳没改 now”

[src/plan_manage_v2/src/traj_server.cpp](src/plan_manage_v2/src/traj_server.cpp) 已经默认用 `ros::Time::now()` 给 `PositionCommand` 打时间戳。因此 Phase 1 的真正重点不是重复补一个已经存在的 `now()`，而是把它参数化并与旧轨迹短时外推一起验证。

### 2.3 V2 当前确实缺少 generic 的 replan 间隙轨迹延续能力

当前 V2 `traj_server` 在轨迹结束后没有通用的短时延续路径；heartbeat 丢失时会直接回到 hover。对于 10m/s 场景，这意味着 replanning 如果稍慢，就可能出现命令供给空窗。

### 2.4 V1 对照必须补到 10m/s

[src/clean_uav_core/config/benchmark_research/campaign_matrix_v2.yaml](src/clean_uav_core/config/benchmark_research/campaign_matrix_v2.yaml) 当前只有 V1 1.2m/s 和 5.0m/s 参考项，缺少 V1 10.0m/s。这个缺口必须补上，否则无法判断问题到底是 V2 架构问题，还是 planner / sim 的共同极限。

## 3. 本轮实施顺序

### 3.1 第一批：低侵入入口改动

本批次只做三件事：

1. 新增本实施文档。
2. 在 campaign 中补入 V1 10.0m/s 对照项。
3. 在 V2 `traj_server` 增加沙盒参数：
   - `traj_server/enable_stale_traj_extrapolation`
   - `traj_server/stale_traj_extrapolation_timeout`
   - `traj_server/retime_position_cmd_now`
4. 在 V2 A* 增加只读诊断参数：
  - `optimization/astar_debug_logging`

这些参数的设计要求：

- 默认关闭旧轨迹外推，保持现有主线行为。
- `retime_position_cmd_now` 默认保持当前行为，不引入行为倒退。
- 默认关闭 A* 诊断日志，避免日常运行刷屏。
- 外推只允许短时间生效，作为 replanning 间隙的缓冲，而不是长期替代新轨迹。

### 3.2 第二批：只读诊断增强

在第一批入口落下后，后续优先考虑以下只读诊断：

- A* pool center / start / end index / out-of-pool 方向日志
- optimizer 失败原因细分
- replan 期间 `PositionCommand` 发布连续性统计
- V1 5.0 / 10.0 与 V2 5.0 / 10.0 的统一对照

这批诊断在证据不足前，不直接推进异步线程化和大规模地图瘦身。

### 3.3 第三批：YAML 可追踪实验分支

在第二批只读诊断补齐后，新增单独的 Stage D A/B case，而不是覆写既有 best case：

- baseline 继续使用 `d_rigid_clearance_balance`
- 新增 `d_rigid_clearance_balance_gap_probe` 作为实验分支
- 只通过 YAML `runtime_overrides` 打开以下开关：
  - `planner_astar_debug_logging`
  - `traj_server_enable_stale_traj_extrapolation`
  - `traj_server_stale_traj_extrapolation_timeout`
  - `traj_server_retime_position_cmd_now`
  - benchmark 命令连续性阈值

这样可以保证：

- baseline 语义不漂移
- run_spec 中能直接记录本次试验到底打开了哪些诊断/缓冲开关
- 回退时只需删除新增 case 或把对应 override 关掉

## 4. 首批实现细节

### 4.1 V1 10.0m/s 对照 case

在 Stage A 中新增一条 V1 10.0m/s case，复用现有 [src/clean_uav_core/config/benchmark_research/stage_a_speed_ramping_v2.yaml](src/clean_uav_core/config/benchmark_research/stage_a_speed_ramping_v2.yaml) 的 `speed_10p0` profile，仅通过 campaign 配置加入：

- `version: v1`
- `profile: speed_10p0`
- `launch_output: src/clean_uav_core/launch/benchmark_stage_a_v1_speed_10p0.launch`
- `launch_file: benchmark_stage_a_v1_speed_10p0.launch`

这样不会修改现有主线 launch，只是新增一个研究入口。

### 4.2 `traj_server` 短时外推沙盒

在 [src/plan_manage_v2/src/traj_server.cpp](src/plan_manage_v2/src/traj_server.cpp) 中加入可选逻辑：

- 若轨迹已结束但距结束时间仍在短时窗口内，则使用末端状态做有限时长外推：
  - $p(t)=p_T + v_T \Delta t + \frac{1}{2} a_T \Delta t^2$
  - $v(t)=v_T + a_T \Delta t$
  - $a(t)=a_T$
  - $j(t)=0$
- 若超过窗口，保持现有逻辑，不继续推演。
- 若 planner heartbeat 丢失，但外推窗口仍未结束，也允许这段短时缓冲继续输出，避免立即回到 hover。

这里的设计目的不是替代 replanning，而是给 replan 成功前的短暂空窗一个连续控制输入。

### 4.3 `PositionCommand` 时间戳参数化

虽然当前代码已经默认使用 `ros::Time::now()`，仍然要把它参数化，原因有两个：

1. 让实验能明确区分“当前行为”与“历史时间基”对 benchmark 指标的影响。
2. 让后续 replay / 复盘时可以明确知道当前试验到底用了哪种 header 时间策略。

## 5. 回退边界

本轮任何实现都必须满足：

- 删除新增文档不影响运行。
- 删除新增 campaign case 不影响既有 benchmark。
- 将 `traj_server/enable_stale_traj_extrapolation` 设为 `false` 即回到当前主线行为。
- `traj_server/retime_position_cmd_now` 默认值保持现有行为，不改变当前稳定路径。

## 6. 验证口径

首批改动后，验证只看三类结果：

1. 配置层：新增 case 是否能被 campaign 正常生成。
2. 编译层：`traj_server.cpp` 是否能通过相关包编译。
3. 行为层：默认参数下行为不变；显式打开旧轨迹外推时，replan 间隙是否减少命令断流。

如果第一批改动通过，再继续做 A* / optimizer / grid_map 侧的诊断增强；如果第二批改动通过，再进入带 YAML 开关的 Stage D A/B 实验分支。

## 7. 当前实施状态

- 已完成：本实施文档落地。
- 已完成：V1 10.0m/s 对照入口、V2 `traj_server` 沙盒参数与短时外推实现、A* 边界诊断链。
- 已完成：benchmark 命令连续性统计、planner failure detail 落盘、Stage D YAML 实验分支接线。
- 已完成：`test_swarm_top_level_runtime_health.sh` 从瞬时 `replan_info` live topic 校验切换为 benchmark 主 CSV / replan CSV 落盘校验，避免 Stage D/V2 在真实已有 replan 数据时被非 latch 话题误判失败。
- 已完成：`d_rigid_clearance_balance_gap_probe` `run_04` 全链路实跑、summary 落盘、campaign index 与分析图刷新。
- 暂不做：默认参数重调、A* pool 继续增大、MINCO 核心深改、多线程大重构。

### 7.1 `gap_probe run_04` 相对 baseline `run_09` 的初步结论

本轮 A/B 选取：

- baseline: `d_rigid_clearance_balance/run_09`
- probe: `d_rigid_clearance_balance_gap_probe/run_04`

结论不是“全面更优”，而是把原来混在一起的现象拆成了更可解释的 trade-off：

- `drone_2` 安全侧改善明显：`min_safety_margin_m` 从 `0.0536` 提升到 `0.1243`，`safety_violation_count` 从 `334` 降到 `165`。
- `drone_2` 规划侧反而变差：`replan_success_rate` 从 `0.663` 降到 `0.403`，`planner_latency_p95_ms` 从 `9` 升到 `15`，失败细节集中为 `fine_collision_unresolved: 46`。
- `drone_0` 的 `control_lag_p95_ms` 与 `jerk_integral` 明显下降，但 `tracking_error_p95` 和 `safety_violation_count` 略有变差，说明 gap-probe 更像把控制输入变连续了，而不是把整体轨迹质量同时拉高。
- `drone_1` 出现显著退化：`tracking_error_p95` 从 `0.719` 升到 `12.059`，`min_safety_margin_m` 从 `0.456` 降到 `0.111`，`safety_violation_count` 从 `47` 升到 `255`。这提示当前实验分支并不适合作为全机统一默认策略。
- 新增连续性指标已成功把“时戳差”和“真实命令断续”分开：`run_04` 三架机都出现了 `command_age_warn_sample_count`，其中 `drone_0/1/2` 分别为 `167/167/104`；`command_interval_warn_count` 分别为 `1/2/1`。

因此本轮更准确的判断是：

- `gap_probe` 证明了连续性诊断和 failure detail 链条已经可用。
- `gap_probe` 不应直接替代 `d_rigid_clearance_balance` 成为新的默认 best case。
- 后续应优先围绕 `drone_2` 的 `fine_collision_unresolved` 做更细的窗口/代价/约束排查，而不是继续把所有改动整体打包推进。