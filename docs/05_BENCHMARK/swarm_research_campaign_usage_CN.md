# Swarm Research Campaign 使用说明

## 1. 文档目的

本文专门说明当前这套“非侵入式 swarm benchmark research campaign”已经做了什么、怎么用、会产出什么，以及接下来建议按什么顺序推进实验。

这份文档对应的实现目标不是替换现有 benchmark 主链路，而是在保留现有 V1/V2 顶层架构、planner、px4ctrl 和 benchmark manager 的前提下，通过 YAML 配置、批量执行脚本和离线分析脚本，把参数研究流程系统化。

当前实现重点解决四件事：

1. 用 YAML 管理四个研究阶段的参数档位。
2. 用统一脚本生成专用 launch 并调用已验证过的 runtime health 链路。
3. 为每次实验写入 run_spec，保留参数与运行元数据。
4. 在实验后自动汇总 campaign index，并生成 Pareto、风险统计、参数敏感度三类离线分析结果。

当前正在新增的覆盖记忆与虚拟目标能力，已经单独拆成两阶段实施方案，见 [docs/coverage_memory_virtual_target_implementation_CN.md](coverage_memory_virtual_target_implementation_CN.md)。

---

## 2. 当前已经完成的工作

### 2.1 四阶段研究配置

已新增四个阶段化 YAML：

- [src/clean_uav_core/config/benchmark_research/stage_a_speed_ramping_v2.yaml](src/clean_uav_core/config/benchmark_research/stage_a_speed_ramping_v2.yaml)
- [src/clean_uav_core/config/benchmark_research/stage_b_stability_recovery_v2.yaml](src/clean_uav_core/config/benchmark_research/stage_b_stability_recovery_v2.yaml)
- [src/clean_uav_core/config/benchmark_research/stage_c_time_smooth_tradeoff_v2.yaml](src/clean_uav_core/config/benchmark_research/stage_c_time_smooth_tradeoff_v2.yaml)
- [src/clean_uav_core/config/benchmark_research/stage_d_extreme_10ms_v2.yaml](src/clean_uav_core/config/benchmark_research/stage_d_extreme_10ms_v2.yaml)

设计含义如下：

- Stage A：速度梯度基线，覆盖 V2 从 1.2m/s 到 10.0m/s，并保留少量 V1 对照。
- Stage B：针对 1.2m/s 病态表现做“止血式”稳定性修复扫描。
- Stage C：在 5m/s 固定档对比 time-priority 与 smooth-priority。
- Stage D：在 10m/s 极限档联合扫描控制刚性、长程预判和安全冗余。

### 2.2 campaign 总清单

已新增 campaign 清单：

- [src/clean_uav_core/config/benchmark_research/campaign_matrix_v2.yaml](src/clean_uav_core/config/benchmark_research/campaign_matrix_v2.yaml)

这个文件是 research campaign 的入口配置，定义了：

- 输出根目录
- 默认重复次数
- 是否在每次运行前清理环境
- 是否等待 session 完整结束
- 每个 phase/case 对应的版本、profile、launch 输出路径和 launch override

### 2.3 执行器脚本

已新增：

- [tools/benchmark_research/run_single_profile.sh](tools/benchmark_research/run_single_profile.sh)
- [tools/benchmark_research/run_benchmark_campaign.py](tools/benchmark_research/run_benchmark_campaign.py)

两者职责分工：

- run_single_profile.sh：负责单次 case 的统一执行包装，最终复用 [test_swarm_top_level_runtime_health.sh](test_swarm_top_level_runtime_health.sh)。
- run_benchmark_campaign.py：负责读取 campaign YAML，生成 launch，创建 run_spec，并循环执行 phase/case。

### 2.4 索引与分析脚本

已新增：

- [tools/benchmark_research/collect_campaign_index.py](tools/benchmark_research/collect_campaign_index.py)
- [tools/pareto_frontier_analysis.py](tools/pareto_frontier_analysis.py)
- [tools/risk_statistics_analysis.py](tools/risk_statistics_analysis.py)
- [tools/parameter_sensitivity_analysis.py](tools/parameter_sensitivity_analysis.py)

其中：

- collect_campaign_index.py：扫描所有 run_spec 和 session summary，生成总表。
- pareto_frontier_analysis.py：基于总飞行时间与平均 jerk 积分画 Pareto frontier。
- risk_statistics_analysis.py：根据安全违例、重规划成功率、误差和控制滞后做风险分层统计。
- parameter_sensitivity_analysis.py：对关键参数与某个指定指标做相关性敏感度分析。

### 2.5 对现有 runtime health 的最小扩展

为支持 research 生成的专用 launch，已给 [test_swarm_top_level_runtime_health.sh](test_swarm_top_level_runtime_health.sh) 增加：

- `--launch-file`

这样就不需要覆盖原有的标准顶层 launch，也不需要新写一套并行的执行链。

### 2.6 新增的两阶段能力边界

下面这部分是当前要新增的能力边界，和现有 research campaign 保持解耦：

- **Phase 1**：在单独的 Python helper 内完成覆盖网格、FoV 投影、虚拟目标检测、实时指标发布和 session 级报表导出。
- **Phase 2**：把 helper 产物接入 campaign index、离线分析脚本和文档体系，必要时再补消息定义。

这两阶段方案的详细说明见 [docs/coverage_memory_virtual_target_implementation_CN.md](coverage_memory_virtual_target_implementation_CN.md)。

---

## 3. 当前目录与产物约定

### 3.1 配置目录

- [src/clean_uav_core/config/benchmark_research](src/clean_uav_core/config/benchmark_research)

存放阶段 YAML 和 campaign matrix。

### 3.2 执行脚本目录

- [tools/benchmark_research](tools/benchmark_research)

存放 run_single_profile 与 run_benchmark_campaign、collect_campaign_index。

### 3.3 研究产物目录

- [benchmark_artifacts/research_profiles](benchmark_artifacts/research_profiles)

每个 case 默认会写到：

- `benchmark_artifacts/research_profiles/<phase>/<case>/run_XX/`

每个 run 目录当前可能包含：

- `run_spec.json`
- 一个或多个 session 子目录
- session 内的 manifest、summary、CSV、bag

campaign 级汇总产物位于：

- [benchmark_artifacts/research_profiles/campaign_index.json](benchmark_artifacts/research_profiles/campaign_index.json)
- [benchmark_artifacts/research_profiles/campaign_index.csv](benchmark_artifacts/research_profiles/campaign_index.csv)
- [benchmark_artifacts/research_profiles/pareto_frontier.json](benchmark_artifacts/research_profiles/pareto_frontier.json)
- [benchmark_artifacts/research_profiles/pareto_frontier.png](benchmark_artifacts/research_profiles/pareto_frontier.png)
- [benchmark_artifacts/research_profiles/risk_statistics.json](benchmark_artifacts/research_profiles/risk_statistics.json)
- [benchmark_artifacts/research_profiles/risk_statistics.csv](benchmark_artifacts/research_profiles/risk_statistics.csv)
- [benchmark_artifacts/research_profiles/risk_statistics.png](benchmark_artifacts/research_profiles/risk_statistics.png)
- [benchmark_artifacts/research_profiles/parameter_sensitivity.json](benchmark_artifacts/research_profiles/parameter_sensitivity.json)
- [benchmark_artifacts/research_profiles/parameter_sensitivity.png](benchmark_artifacts/research_profiles/parameter_sensitivity.png)

如果后续加入覆盖记忆模块，建议额外产出：

- `coverage_metrics.json`
- `target_detection.json`
- `coverage_vs_time.png`
- `coverage_memory_summary.json`

---

## 4. 核心使用方式

## 4.1 前置环境

每个新终端都先执行：

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh
```

如果只运行 Python 脚本，当前实现也可以直接用：

```bash
/usr/bin/python3 <script>
```

但凡真正涉及 ROS、roslaunch、runtime health，仍然建议遵守仓库既有要求，从已 source 环境的终端发起。

## 4.2 只做命令链检查

如果你只想看即将执行什么，而不真正起 Gazebo / ROS：

```bash
/usr/bin/python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_a \
  --case v2_speed_1p2 \
  --dry-run
```

用途：

- 验证 campaign 配置是否能正确解析
- 验证生成器和 runtime wrapper 的参数拼接是否正确

## 4.3 只生成 launch 与 run_spec

如果你想先生成专用 launch 和元数据，不实际启动实验：

```bash
/usr/bin/python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_a \
  --case v2_speed_1p2 \
  --generate-only
```

用途：

- 预审生成后的 launch 内容
- 检查 run_spec 中解析后的参数是否符合预期

## 4.4 实际运行一个 case

当前最推荐的起步 case 是 stage A 的 1.2m/s V2 档：

```bash
/usr/bin/python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_a \
  --case v2_speed_1p2
```

它会自动完成：

1. 读取 campaign matrix。
2. 生成专用 launch。
3. 在对应 run 目录写入 run_spec。
4. 调用 run_single_profile.sh。
5. 由 run_single_profile.sh 继续调用 runtime health。
6. 在成功后把 run_spec 状态更新为 completed，并回写 session 目录与 summary 路径。

## 4.5 运行整个阶段

如果单 case 稳定，可直接跑一个完整阶段，例如 stage A：

```bash
/usr/bin/python3 tools/benchmark_research/run_benchmark_campaign.py --phase stage_a
```

当前不建议一开始就直接跑 stage D，因为它是 10m/s 极限档，失败概率更高，更适合在低速档确认链路和参数记录都可靠之后再启动。

## 4.6 生成索引与分析结果

每跑完一批实验后，执行：

```bash
/usr/bin/python3 tools/benchmark_research/collect_campaign_index.py
/usr/bin/python3 tools/pareto_frontier_analysis.py
/usr/bin/python3 tools/risk_statistics_analysis.py
/usr/bin/python3 tools/parameter_sensitivity_analysis.py
```

如果想一行执行，也可以：

```bash
/usr/bin/python3 tools/benchmark_research/collect_campaign_index.py && \
/usr/bin/python3 tools/pareto_frontier_analysis.py && \
/usr/bin/python3 tools/risk_statistics_analysis.py && \
/usr/bin/python3 tools/parameter_sensitivity_analysis.py
```

---

## 5. run_spec 与 index 的含义

### 5.1 run_spec.json

每个 run 都会写一个 run_spec，主要用于保留“这次实验当时到底是怎么配出来的”。

关键字段包括：

- `phase_id`
- `case_name`
- `version`
- `profile`
- `runtime_overrides`
- `resolved_parameters`
- `status`
- `summary_path`
- `session_dirs`

当前状态语义：

- `planned`：已创建 run_spec，但还没进入真实运行阶段。
- `generated`：只生成了 launch 和 run_spec，没有真正执行。
- `completed`：runtime health 完成，已经找到 session summary。
- `failed`：执行失败，并保留退出码。

### 5.2 campaign_index

campaign_index 是跨 run 的扁平化总表，便于后续用 pandas 或 Excel 快速筛选。

当前 `collect_campaign_index.py` 已经会在检测到覆盖记忆 helper 产物时，把这些字段一并纳入；没有这些文件时，相关列会保持空值，原有 summary / rosbag 字段不受影响。

覆盖相关字段包括：

- 累计覆盖率
- 探索率
- 冗余因子
- Time_to_First_Detection
- coverage / target / history / plot 相关 artifact 路径
- 首次检测目标与命中 UAV 的标识
- coverage 侧 jerk 与重规划汇总

当前聚合字段包括：

- `target_speed_mps`
- `total_flight_time_sec`
- `avg_tracking_error_p95_m`
- `avg_control_lag_p95_ms`
- `avg_planner_latency_p95_ms`
- `avg_replan_success_rate`
- `min_safety_margin_m`
- `total_safety_violation_count`
- `avg_jerk_integral`
- `stop_reasons`
- `coverage_cumulative_coverage_pct`
- `coverage_exploration_rate_m2_s`
- `coverage_redundancy_factor`
- `coverage_time_to_first_detection_sec`
- `coverage_metrics_path`
- `coverage_target_detection_path`

---

## 6. 当前分析脚本的解读方式

## 6.1 Pareto frontier

当前实现把：

- 总飞行时间
- 平均 jerk 积分

作为两目标平衡面。

解释方式：

- 越靠左：总飞行时间越短。
- 越靠下：平均 jerk 积分越小，通常代表轨迹更平滑、动作更不激烈。
- 位于 frontier 上的点：在当前样本集中，没有其他 case 同时在这两个指标上都更优。

## 6.2 风险统计

当前实现用了一个保守的规则分层：

- high：存在安全违例，或重规划成功率过低。
- medium：没有明显安全违例，但误差或控制滞后偏大。
- low：相对稳定。

这不是最终科研定义，只是现阶段用于快速筛查坏样本的第一版规则。

## 6.3 参数敏感度

当前实现使用 Pearson 相关性做初步排序。

这意味着：

- 结果更适合作为“下一轮重点排查方向”，而不是最终因果结论。
- 当样本量很小，或者参数变化范围很窄时，相关性会不稳定。
- 只有在 stage A/B/C/D 累积了足够多 completed run 之后，这张图才真正有分析价值。

---

## 7. 当前已验证到什么程度

截至目前，这套 research campaign 已完成以下验证：

1. 新脚本静态检查通过。
2. stage A 单 case dry-run 已验证命令链可通。
3. generate-only 已验证 launch 生成和 run_spec 落盘。
4. index 与分析脚本已验证在“无 completed run”时不会崩溃，而是平滑输出占位结果。

这意味着：

- 脚手架已经可以用。
- 但实验数据质量仍然要通过真实运行继续验证。

换句话说，当前真正还缺的是“真实 completed run 数据”，而不是脚本本身。

---

## 8. 推荐的后续实验步骤

建议按下面顺序推进，不要一上来直接跑全矩阵。

### 第一步：低风险真实样例

先跑：

- `stage_a / v2_speed_1p2`

目标：

- 确认真实 run 可以从 generated 走到 completed
- 确认 session 目录内有非空主 CSV、summary、bag
- 确认 campaign_index 能读到真实数值

### 第二步：扩展到 stage A 的低中速档

建议下一批跑：

- `v2_speed_3p0`
- `v2_speed_5p0`
- `v1_reference_speed_1p2`
- `v1_reference_speed_5p0`

目标：

- 建立一条可解释的速度梯度基线
- 先看 V2 与 V1 在低中速下的差别
- 检查 5m/s 档是否已经出现明显 planner latency、safety margin 或 actuator stress 问题

### 第三步：再进入 Stage B

当 stage A 的低中速结果可信后，再运行 stage B：

- 病态基线
- perception aligned
- obstacle guarded
- 高频重规划
- recovery combo

目标：

- 找出 1.2m/s 异常行为到底对哪些参数最敏感

### 第四步：做 Stage C 与 Stage D

- Stage C：研究时间权重与平滑性权重之间的折中关系
- Stage D：研究 10m/s 极限档的可行边界

其中 Stage D 应该最后做，因为它最容易触发失败、半开会话、极端控制负荷和安全边界问题。

---

## 9. 推荐的实验后检查清单

每完成一次真实 run，建议至少检查：

1. run_spec 的 `status` 是否为 `completed`。
2. session 目录里是否存在 `*_summary.json`。
3. 主 CSV 是否不止表头。
4. 是否没有残留 `.bag.active`。
5. `campaign_index.json` 中该条目是否有真实数值，而不是空字段。

如果这五条里有一条不满足，该 run 就不建议直接拿去做论文或报告结论。

---

## 10. 一条最小可复现实验命令

如果后续只想最小成本复现当前入口，直接用下面这条：

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh
/usr/bin/python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_a \
  --case v2_speed_1p2
```

跑完后立刻更新总表与分析图：

```bash
/usr/bin/python3 tools/benchmark_research/collect_campaign_index.py && \
/usr/bin/python3 tools/pareto_frontier_analysis.py && \
/usr/bin/python3 tools/risk_statistics_analysis.py && \
/usr/bin/python3 tools/parameter_sensitivity_analysis.py
```

这就是当前 research campaign 的最小闭环。