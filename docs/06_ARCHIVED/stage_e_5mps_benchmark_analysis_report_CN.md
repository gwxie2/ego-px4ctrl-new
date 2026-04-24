# Stage E 5m/s 基准测试分析报告

**报告日期**：2026-04-11
**实验阶段**：stage_e / 5m/s 组
**运行版本**：v2
**比较基线**：smooth_priority
**感知配置**：clean / mild / extreme

## 1. 报告范围

本报告总结 stage_e 中 5m/s 三组真实运行结果，覆盖以下三个 case：

- [e_5p0_clean](../benchmark_artifacts/research_profiles/stage_e/e_5p0_clean/run_04/run_spec.json)
- [e_5p0_mild](../benchmark_artifacts/research_profiles/stage_e/e_5p0_mild/run_06/run_spec.json)
- [e_5p0_extreme](../benchmark_artifacts/research_profiles/stage_e/e_5p0_extreme/run_03/run_spec.json)

对应的 summary 产物分别为：

- [clean summary](../benchmark_artifacts/research_profiles/stage_e/e_5p0_clean/run_04/auto_20260411_104756/auto_20260411_104756_summary.json)
- [mild summary](../benchmark_artifacts/research_profiles/stage_e/e_5p0_mild/run_06/auto_20260411_110727/auto_20260411_110727_summary.json)
- [extreme summary](../benchmark_artifacts/research_profiles/stage_e/e_5p0_extreme/run_03/auto_20260411_111000/auto_20260411_111000_summary.json)

运行顺序与当前状态记录在 [stage_e_execution_order_CN.md](../benchmark_artifacts/research_profiles/stage_e/stage_e_execution_order_CN.md)。统一索引已生成于 benchmark_artifacts/research_profiles/campaign_index.json 和 benchmark_artifacts/research_profiles/campaign_index.csv，可直接用于后续分析。

## 2. 结论先行

5m/s 组已经形成清晰的层级关系：clean 是当前最稳的基线，mild 开始明显放大控制时延与重规划压力，extreme 进一步把问题推到安全边界和动力学边界上。

三个 case 都成功生成了 manifest、summary、主 CSV、replan CSV、event CSV 和 rosbag，说明 benchmark 闭环已经可用。就性能本身而言，5m/s 并不是“无风险通过”，而是“可以跑完，但已经能看出退化层对系统时序和安全裕度的影响”。

## 3. 汇总表

| case | session stop_reason | 会话结果 | 代表性指标 |
| --- | --- | --- | --- |
| clean | goal_reached | 完整完成 | 平均 tracking_error_mean 约 0.42 m，平均 control_lag_mean 约 302 ms |
| mild | emergency_stop | 产物完整，但会话提前停止 | 平均 tracking_error_mean 约 3.06 m，平均 control_lag_mean 约 1176 ms |
| extreme | emergency_stop | 产物完整，但会话提前停止 | 平均 tracking_error_mean 约 3.65 m，平均 control_lag_mean 约 1292 ms |

## 4. 分 case 分析

### 4.1 clean：当前 5m/s 的最佳基线

clean 组是 5m/s 下最接近“可稳定复用”的基线。

- 三架无人机都完成了目标；session 级 stop_reason 为 goal_reached。
- 跟踪误差保持在较低水平，三机的 tracking_error_mean 分别约为 0.35 m、0.47 m、0.43 m。
- 控制时延整体可控，三机的 control_lag_mean 约为 193 ms 到 414 ms。
- 规划输出基本连续，planner 端重规划成功率在前两架无人机上达到 100%，第三架无人机降到 78.3%。

需要注意的是，clean 并不代表“完全没有问题”。

- drone_2 仍然是限制项：min_obstacle_clearance_m 约 0.261 m，safety_violation_count 为 26，且出现过一次 recovery 事件。
- drone_1 的控制滞后 P95 已接近 2 s，说明系统在 5m/s 下已经开始出现明显的时序抖动，只是还没有把任务打断。

结论：clean 可以作为后续对照组和回归基线，但它已经处在“可运行但不轻松”的区间，不建议再把它理解成完全安全裕度充足的状态。

### 4.2 mild：感知退化开始显著影响时序

mild 组是最能体现 sensor degradation 对系统节奏影响的一组。

- session stop_reason 为 emergency_stop，说明系统未能在完整时间窗内维持稳定收敛。
- 三机的 tracking_error_mean 上升到约 1.81 m、3.55 m、3.81 m。
- 三机的 control_lag_mean 全部超过 1 s，且最高达到约 1.26 s。
- drone_2 的重规划成功率降到 44.4%，并且出现 6 次 init_failed，说明退化后初值搜索已经开始明显失效。

mild 组最重要的信号不是“飞不起来”，而是“飞得起来，但命令节奏和规划稳定性都被拉长了”。这会直接放大后续的碰撞风险和控制误差。

结论：mild 更适合作为负载/鲁棒性压力测试，不适合作为当前 5m/s 的默认推荐运行档。

### 4.3 extreme：安全边界和动力学边界同时吃紧

extreme 组仍然成功生成了完整产物，但系统代价更高。

- session stop_reason 同样为 emergency_stop。
- drone_0 的 tracking_error_mean 升到约 4.91 m，control_lag_mean 超过 2 s，说明在极端退化下前向跟踪误差已经非常大。
- drone_2 的 min_obstacle_clearance_m 仅约 0.125 m，actuator_saturation_ratio 达到 14.6%，并出现 1 次 emergency recovery 和 1 次 interaction recovery。
- drone_2 的 replan_success_rate 仍能保持在 72.2%，但失败原因更偏向 optimizer_failed 和 init_failed 的组合，而不只是单纯的感知丢失。

extreme 的特点是：它不是单纯“更慢”，而是把系统推到安全裕度和执行器余量都更紧的状态。换句话说，算法还能跑，但已经明显接近可接受边界。

结论：extreme 可以保留为研究中的压力组，但不应被当作稳定工作点。

## 5. 横向结论

1. 5m/s 已经证明 stage_e 的感知退化链路是可运行的，benchmark 闭环完整。
2. clean 是唯一一个 session 级 goal_reached 的 5m/s case，说明它是当前最合适的基线。
3. mild 和 extreme 都会显著拉高 control_lag，并把 drone_2 推向更差的安全裕度。
4. 退化对系统的影响不是线性的：mild 主要拉高时序压力，extreme 则更容易把问题转化成安全和执行器余量问题。
5. 如果后续要继续推进 10m/s，建议沿用 clean 作为基线，不要直接从 mild 或 extreme 外推。

## 6. 建议的下一步

- 统一索引已经生成，后续分析可以直接使用 benchmark_artifacts/research_profiles/campaign_index.json 和 benchmark_artifacts/research_profiles/campaign_index.csv。
- 10m/s 的 clean / mild / extreme 组已经补齐并纳入 [综合报告](swarm_benchmark_comprehensive_report_CN.md)，后续若继续分析，建议直接以综合报告为入口。
- 如果要继续做参数分析，优先看 drone_2 的重规划失败模式，因为它是 5m/s 组里最早出现边界问题的节点。

## 7. 结论

5m/s 下，系统已经从“能不能跑”进入“在什么退化程度下还能稳住”的阶段。clean 是当前推荐基线，mild 是明显的时序压力组，extreme 则是安全边界压力组。

这组结果对后续 10m/s 的意义在于：5m/s 已经把退化链路的主要失效模式暴露出来了，10m/s 的补齐结果也已证实同一类问题会被进一步放大。下一步不该再只看是否完成，而要重点看 position_cmd 及时性、控制滞后和 drone_2 的重规划稳定性。