# broadcast_bspline 与多机互避机制说明（中文）

此文档说明 cleanroom_ws 中用于多机通信与互避的关键机制、当前默认配置的限制，以及如何在 Phase-2 环境下启用真实的轨迹广播与互避。

## 概览

- 核心话题：`broadcast_bspline`（在代码中以 `planning/broadcast_bspline_from_planner` / `planning/broadcast_bspline_to_planner` 的形式出现）
- 关键实现位置：
  - 轨迹广播与接收：`cleanroom_ws/src/ego_planner/src/ego_replan_fsm.cpp`（`BroadcastBsplineCallback` & 发布点）
  - 互避代价：`cleanroom_ws/src/bspline_opt/src/bspline_optimizer.cpp`（`calcSwarmCost`）
  - launch 重映射：`cleanroom_ws/src/ego_planner/launch/advanced_param.xml` 与 `cleanroom_ws/src/ego_planner/launch/advanced_param_xtdrone.xml`，以及 `cleanroom_ws/src/clean_uav_core/launch/phase1_minimal_demo.launch` 和 `phase2_dual_uav_stack.launch` 中的参数映射。

## 工作机制

1. 当本机的 `ego_planner` 生成新的 bspline 轨迹时，会通过本地发布器发布 `planning/broadcast_bspline_from_planner`（节点内部话题），该话题通常会在 launch 中被 remap 到某个全局（或命名空间下的）`planning_broadcast_topic`。
2. 同时，`ego_planner` 订阅 `planning/broadcast_bspline_to_planner`（也会 remap 到相同或不同的 `planning_broadcast_topic`），用于接收队友发布的轨迹消息。
3. 接收到队友轨迹后，`BroadcastBsplineCallback` 会把接收到的轨迹解码并写入 `swarm_trajs_buf_`，随后调用碰撞检测函数（`checkCollision`）。若检测到可能冲突，会触发本机进行 `REPLAN_TRAJ`（重新规划）。
4. 在轨迹优化器侧，`BsplineOptimizer::calcSwarmCost` 会将已知队友轨迹在时间上与本机候选轨迹逐点对比，采用椭球距离度量（竖直方向允许更小距离），对靠近的部分施加二次惩罚并计算梯度，促使最终轨迹增大间距以实现互避。

## 当前默认配置与限制（重要）

- 在 `phase2_dual_uav_stack.launch` 与 `phase1_minimal_demo.launch` 中，`planning_broadcast_topic` 默认按**每机命名空间隔离**（例如 `/uav0/broadcast_bspline`、`/uav1/broadcast_bspline`）。
- 由于每台 `ego_planner` 既发布到其命名空间下的 broadcast topic，又用同一命名空间下的 topic 订阅，这造成**消息只在本机内循环**（发布者的 `drone_id` 被用作过滤），从而无法看到其它机的轨迹。换言之，默认配置下互避逻辑无法被激活。

## 如何启用跨机互避（建议步骤）

1. 统一广播话题：将两机的 `planning_broadcast_topic` 指向同一个全局话题，例如 `/swarm/broadcast_bspline`（在 `phase2_dual_uav_stack.launch` 中对 `uav0` 和 `uav1` 的 include 均设置相同值）。
2. 确保时间同步或允许小延迟：`BroadcastBsplineCallback` 会丢弃时间差距过大的消息（阈值示例：0.25s），在多机仿真/网络环境中需保证时间同步或增大可接受窗口。
3. 调整 `optimization/swarm_clearance` 参数以匹配期望的安全距离（launch 或参数服务器中设置，默认在 demo 中通常为 0.5m）。
4. 在较大规模的测试中考虑：降低广播频率、合并/稀疏轨迹点以减轻网络与计算压力。

## 关键文件引用（供快速定位）

- `cleanroom_ws/src/ego_planner/src/ego_replan_fsm.cpp` — `BroadcastBsplineCallback`（处理远端轨迹并触发重规划）。
- `cleanroom_ws/src/bspline_opt/src/bspline_optimizer.cpp` — `calcSwarmCost`（互避代价与梯度计算）。
- `cleanroom_ws/src/clean_uav_core/launch/phase1_minimal_demo.launch` 与 `phase2_dual_uav_stack.launch` — 默认 remap 与 `planning_broadcast_topic` 参数。 

## 小结

`broadcast_bspline` 是 cleanroom_ws 设计中用于实现轨迹级别协同与互避的机制；但在 Phase-2 的默认 launch 配置下，由于命名空间隔离导致两机轨迹无法互通，互避未被激活。若希望在双机或多机仿真中启用互避，请将广播话题统一为一个全局话题，并根据网络与时间同步条件调整相关参数。

（文档到此，若需要我可以把示例 launch 修改为使用 `/swarm/broadcast_bspline` 并提交变更。）
