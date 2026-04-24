# 多旋翼集群基准实验综合报告

**报告日期**：2026-04-11  
**实验平台**：XTDrone + PX4 SITL + Gazebo + MAVROS + EGO-Planner V2 + px4ctrl  
**集群规模**：3 架 iris 无人机  
**数据来源**：benchmark_artifacts/research_profiles/campaign_index.json 与 campaign_index.csv  
**参考报告**：[Phase A–D 报告](phase_a_d_benchmark_teacher_report_CN.md)、[Stage E 5m/s 报告](stage_e_5mps_benchmark_analysis_report_CN.md)

## 1. 报告范围

本报告把当前已经稳定落盘的基准结果合并成一份可直接汇报的总览，覆盖三类信息：

1. Phase A–C 的速度、稳定性和时序权衡结论。
2. Stage D 的 10m/s 极限速度探索结果。
3. Stage E 的 5m/s 感知退化结果，以及补齐后的 10m/s 扩展项。

原则上，所有结论都以 campaign index 为准，不再以单次日志印象替代真实索引。

## 2. 结论先行

当前这批结果已经把系统的主要边界划分清楚了。

- Phase A–C 说明：中速场景下，单纯追求时间优先会迅速把跟踪和安全推到不可接受区间，平滑优先和适度感知放宽更可取。
- Stage D 说明：10m/s 不是简单的速度放大，而是把问题推成规划时延、重规划成功率和 jerk 积分的联合约束问题。
- Stage E 说明：5m/s 下感知退化会优先放大控制滞后和跟踪误差，clean 是当前最稳的运行基线，但它仍不是“无风险”状态。

综合来看，系统已经从“能不能跑”进入“在什么压力类型下还能维持可接受行为”的阶段。

## 3. 分阶段总览

| 阶段 | 主要压力 | 当前代表结果 | 实际含义 |
| --- | --- | --- | --- |
| Phase A | 速度梯度压力 | 5m/s 已能完成任务，但控制滞后显著抬升 | 系统在中速档开始暴露控制与规划耦合问题 |
| Phase B | 感知恢复与安全裕度 | perception_clearance_plus 是最优方案，成功率 0.9935，安全间距 0.344 m | 提升感知覆盖和 clearance 比单纯加快重规划更有效 |
| Phase C | 时间-平滑权衡 | smooth_priority 优于 time_priority，后者 tracking error P95 达 18.833 m，违规 421 次 | 时间优先不可作为默认策略 |
| Stage D | 10m/s 极限速度 | d_rigid_clearance_balance run_01 最均衡 | headless 模式下才有可用结果，且仍需在时延/抖动上做权衡 |
| Stage E | 5m/s 感知退化 | e_5p0_clean run_04 是当前最佳基线 | mild 和 extreme 都明显抬升控制压力 |

## 4. Stage D 10m/s 结果

Stage D 的目标不是找一个“最漂亮”的数值，而是确认 10m/s 场景下什么配置还能稳定活下来。当前索引里最值得拿出来汇报的是下面三组。

| case | run | 状态 | 重规划成功率 | 最小安全间距 | 控制滞后 P95 | 跟踪误差 P95 | jerk 积分 | 终止原因 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| d_rigid_clearance_balance | run_01 | completed | 0.9206 | 0.2894 m | 7658.67 ms | 0.6934 m | 135.35 | goal_reached |
| d_rigid_control | run_01 | completed | 0.7846 | 0.1295 m | 6690.00 ms | 9.9218 m | 128.98 | stop_reason 未单独提取 |
| d_rigid_lambda_light | run_01 | completed | 0.9218 | 0.1355 m | 10573.33 ms | 11.2273 m | 170.76 | 高时延、低裕度 |

### 4.1 Stage D 解读

- d_rigid_clearance_balance 是当前最均衡的 10m/s 候选。它没有把控制滞后压到最低，但在安全间距、成功率和 tracking error 之间取得了更好的平衡。
- d_rigid_control 更偏保守，但 tracking error 已经明显变差，说明单纯守住控制并不能自动换来好的轨迹质量。
- d_rigid_lambda_light 的重规划成功率很高，但 lag 和 jerk 同时抬升，属于“能跑但代价太大”的分支。

结论：10m/s 场景下，最重要的不是把某个单项指标调到极值，而是避免让 lag 和 jerk 一起失控。当前最值得继续沿用的是 d_rigid_clearance_balance 这类折中方案。

## 5. Stage E 5m/s 结果

Stage E 的重点是感知退化，不是速度极限。这里最关键的现象是：clean 能稳定完成，mild 和 extreme 都把控制压力明显抬高，但 failure channel 不完全相同。

| case | run | 状态 | 总飞行时长 | 跟踪误差 P95 | 控制滞后 P95 | 规划器延迟 P95 | 重规划成功率 | 最小安全间距 | 违规次数 | jerk 积分 | 终止原因 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| e_5p0_clean | run_04 | completed | 13.36 s | 1.871 m | 1351.33 ms | 7.00 ms | 0.9275 | 0.000 m | 43 | 34.09 | goal_reached |
| e_5p0_mild | run_09 | completed | 16.31 s | 4.562 m | 2661.41 ms | 12.33 ms | 0.8816 | 0.193 m | 76 | 45.56 | goal_reached |
| e_5p0_extreme | run_03 | completed | 15.66 s | 11.860 m | 2937.54 ms | 14.67 ms | 0.9074 | 0.125 m | 19 | 21.28 | emergency_stop, goal_reached |
| e_5p0_mild | run_06 | completed | 未记录 | 未记录 | 未记录 | 未记录 | 0.7379 | 0.257 m | 38 | 未记录 | emergency_stop |

### 5.1 Stage E 解读

- clean 是当前最稳的基线，但它并不等于“安全裕度充足”。0.000 m 的最小安全间距和 43 次违规说明系统只是完成了任务，并没有留下宽松余量。
- mild 已经明显把控制滞后和跟踪误差推高，run_09 虽然最终 goal_reached，但代价明显升高。
- extreme 的跟踪误差最差，说明感知退化到一定程度后，系统首先损失的是轨迹跟踪质量，而不是立刻损失重规划成功率。
- run_06 是一个更差的 mild 尝试，已经进入 emergency_stop，说明 mild 档位本身就处在临界带附近。

结论：Stage E 当前的可汇报基线是 e_5p0_clean run_04，但它只能算“当前最稳”，不能算“已经充分安全”。

## 6. Phase A–C 的压缩结论

这部分已有完整数值和讨论，请直接参考 [Phase A–D 报告](phase_a_d_benchmark_teacher_report_CN.md)。为了把总报告闭合，这里只保留最重要的归纳。

| 阶段 | 最重要的结论 | 推荐配置 |
| --- | --- | --- |
| Phase A | 5m/s 已能完成任务，但控制滞后会显著抬升 | 5m/s 基线需继续保留 |
| Phase B | 提升感知覆盖与 clearance 比单纯提速更有效 | perception_clearance_plus |
| Phase C | time_priority 不可作为默认策略 | smooth_priority |

## 7. Stage E 10m/s 结果

三条 10m/s 扩展线已经补齐并重新索引，但它们并不构成“稳定完成”的结论，只说明系统在更高压力下仍能落出完整闭环产物。

| case | run | 状态 | 总飞行时长 | 跟踪误差 P95 | 控制滞后 P95 | 规划器延迟 P95 | 重规划成功率 | 最小安全间距 | 违规次数 | jerk 积分 | 终止原因 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| e_10p0_clean | run_07 | completed | 30.45 s | 0.755 m | 19604.33 ms | 34.67 ms | 0.6656 | 0.0508 m | 211 | 198.41 | max_session_duration |
| e_10p0_mild | run_05 | completed | 30.91 s | 15.560 m | 14411.50 ms | 1.67 ms | 0.3305 | 0.0130 m | 1278 | 93.56 | max_session_duration |
| e_10p0_extreme | run_04 | completed | 20.86 s | 15.189 m | 3883.89 ms | 41.00 ms | 0.5236 | 0.0242 m | 154 | 70.92 | emergency_stop |

### 7.1 10m/s 解读

- clean 是三者中最均衡的 10m/s 结果，但控制滞后和安全违规依旧很高，session 也只是以 max_session_duration 收口，并没有做到 goal_reached。
- mild 的跟踪误差和违规次数都非常高，说明感知退化在 10m/s 下已经把系统推到了明显不可接受的区间。
- extreme 以 emergency_stop 收口，虽然平均控制滞后低于 clean 和 mild，但这更像是提前中止，而不是性能更优。

结论：10m/s 现在可以作为“已补齐的压力测试结果”引用，但不能作为稳定基线引用。

## 8. 最终建议

1. Phase A–C 的推荐基线沿用 smooth_priority 与 perception_clearance_plus 的结论，不要回到 time_priority。
2. Stage D 的 10m/s 推荐继续以 d_rigid_clearance_balance 为主线做微调，而不是盲目压低 clearance 或一味增大 lambda。
3. Stage E 的当前运行基线仍然是 e_5p0_clean run_04，但 10m/s 已经补齐，可用于说明系统在高压下的边界行为。
4. 10m/s 结果可以纳入正式汇报，但只能作为压力测试和边界样本，不能当作稳定推荐配置。

## 9. 归档说明

本报告的数值基础来自最新的 campaign index。对应的原始数据、汇总表和分析产物应作为后续复核入口：

- benchmark_artifacts/research_profiles/campaign_index.json
- benchmark_artifacts/research_profiles/campaign_index.csv
- benchmark_artifacts/research_profiles/pareto_frontier.json
- benchmark_artifacts/research_profiles/parameter_sensitivity.json
- benchmark_artifacts/research_profiles/risk_statistics.json

如需查看更细的阶段分析，请分别参考 [Phase A–D 报告](phase_a_d_benchmark_teacher_report_CN.md) 与 [Stage E 5m/s 报告](stage_e_5mps_benchmark_analysis_report_CN.md)。