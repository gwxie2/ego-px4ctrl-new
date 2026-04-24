# Swarm Benchmark 当前情况分析与性能调参建议（2026-04-03）

## 1. 文档目的

本文面向当前 V1 / V2 集群 benchmark 工作的阶段性分析，回答三个问题：

1. 当前 benchmark 体系到底是否已经跑通。
2. 这次拿到的数据能够说明什么，不能说明什么。
3. 下一步应该优先调哪些参数，怎样避免无效试错。

本文基于以下已完成的真实产物：

- V1 完整会话：`benchmark_artifacts/20260403_top_level_final/v1/auto_20260403_104104/`
- V2 完整会话：`benchmark_artifacts/20260403_top_level_final/v2/auto_20260403_104514/`
- 最终对比报告：`benchmark_artifacts/20260403_top_level_final/report/advanced_benchmark_report.pdf`
- 最终汇总 JSON：`benchmark_artifacts/20260403_top_level_final/report/advanced_benchmark_summary.json`

---

## 2. 结论先行

### 2.1 已经完成的部分

- V1 / V2 已经在 `swarm_top_level_v1.launch` 与 `swarm_top_level_v2.launch` 入口上完成统一 benchmark 闭环。
- 现在不只是“能启动”，而是已经能稳定生成：主 CSV、event CSV、replan CSV、summary JSON、正式 rosbag、PDF 报告。
- benchmark 运行链已经经过一轮真实修正，关键问题包括：
  - benchmark 主采样 topic 订阅路径错误；
  - V2 自动起 session 条件过窄；
  - 会话完成后重复自动拉起第二个半截 session；
  - 报表脚本在空主 CSV 条件下容易误导判断。

### 2.2 当前实验结论的边界

- 当前这批数据**可以作为 benchmark 体系已打通的基线结论**。
- 当前这批数据**不能直接作为 10m/s 高速 benchmark 结论**。

原因很关键：

- 本次真实运行时，`swarm_top_level_v1.launch` 和 `swarm_top_level_v2.launch` 的 `planner_max_vel` 默认值都是 `1.2`。
- 同时 benchmark 元数据里 `benchmark_default_vmax` 默认值仍是 `10.0`。
- 因此当前生成的 bag 文件名里有 `vel10`，summary 里也写了 `v_max: 10.0`，但这**不是本次实际规划速度上限**。

换句话说，本轮结果更准确的表述应当是：

> 已完成 V1 / V2 benchmark 基础设施验证，并在当前 `planner_max_vel=1.2` 的保守基线下拿到第一批可比较的真实数据。

### 2.3 当前版本对比结论

- 在当前保守参数下，V1 整体稳定性略好于 V2。
- V2 在当前配置下表现出更高的速度均值，但代价是更差的 replanning 成功率和更低的最小安全裕度。
- 两个版本当前都以 `emergency_stop` 结束会话，说明当前配置还没有达到“稳定完成整段任务”的状态。

---

## 3. 本轮真实运行参数与结果语境

### 3.1 本轮有效顶层默认参数

本轮 benchmark 对应的实际顶层默认参数如下：

#### V1

- `planner_max_vel = 1.2`
- `planner_max_acc = 2.8`
- `planner_max_jerk = 4.0`
- `swarm_clearance = 0.5`
- `hover_percent = 0.58`
- `mass = 1.2`
- `benchmark_default_vmax = 10.0`

#### V2

- `planner_max_vel = 1.2`
- `planner_max_acc = 2.8`
- `planner_max_jerk = 4.0`
- `swarm_clearance = 0.35`
- `hover_percent = 0.58`
- `mass = 1.2`
- `benchmark_default_vmax = 10.0`

### 3.2 这意味着什么

- V1 与 V2 当前**并不是在完全同一安全阈值下**比较，因为 `swarm_clearance` 默认值不同。
- `safety_violation_count` 也因此不能直接横向比较绝对值，因为它跟阈值绑定。
- 当前更可信的横向比较指标是：
  - `tracking_error_p95`
  - `control_lag_p95_ms`
  - `planner_latency_p95_ms`
  - `min_safety_margin_m`
  - `replan_success_rate`
  - `stop_reason`

---

## 4. 核心量化结果

### 4.1 汇总表

| 指标 | V1 | V2 | 解释 |
| --- | --- | --- | --- |
| 运行入口 | `swarm_top_level_v1.launch` | `swarm_top_level_v2.launch` | 已统一为顶层入口 |
| 主 CSV 是否有真实数据 | 是 | 是 | V1 `drone_0` 416 行，V2 `drone_0` 305 行 |
| 会话是否完整闭环 | 是 | 是 | 均有 manifest / summary / bag |
| 平均 `tracking_error_p95` | 约 `0.169 m` | 约 `0.180 m` | V1 略优 |
| 平均 `control_lag_p95` | 约 `83.7 ms` | 约 `87.3 ms` | 两者同量级，V2 略高 |
| 平均 `planner_latency_p95` | `1.0 ms` | 约 `2.67 ms` | V1 明显更轻 |
| 最差 `min_safety_margin_m` | `0.3205 m` | `0.2712 m` | V2 风险更高 |
| 平均 `replan_success_rate` | 约 `94.7%` | 约 `74.3%` | V2 明显更弱 |
| 最差单机 `replan_success_rate` | `88.0%` | `39.47%` | V2 `drone_1` 是当前主要问题点 |
| 会话结束原因 | `emergency_stop` | `emergency_stop` | 当前仍未到“稳定收官”阶段 |

### 4.2 单机观察重点

#### V1

- `drone_0`
  - `tracking_error_p95 = 0.1460 m`
  - `min_safety_margin_m = 0.3401 m`
  - `replan_success_rate = 96.15%`
- `drone_1`
  - `tracking_error_p95 = 0.1542 m`
  - `min_safety_margin_m = 0.3205 m`
  - `replan_success_rate = 100%`
- `drone_2`
  - `tracking_error_p95 = 0.2083 m`
  - `speed_max = 1.3751 m/s`
  - `replan_success_rate = 88.0%`
  - 最终触发 `emergency_stop`

#### V2

- `drone_0`
  - `tracking_error_p95 = 0.1790 m`
  - `planner_latency_p95 = 2.0 ms`
  - `replan_success_rate = 100%`
- `drone_1`
  - `tracking_error_p95 = 0.1646 m`
  - `planner_replan_interval_mean_ms = 386.16`
  - `replan_success_rate = 39.47%`
  - 当前 V2 最明显的不稳定点
- `drone_2`
  - `tracking_error_p95 = 0.1951 m`
  - `min_safety_margin_m = 0.2712 m`
  - `safety_violation_count = 5`
  - 当前 V2 最危险的安全边界点

---

## 5. 当前状态判断

### 5.1 从工程状态看

当前系统已经进入如下阶段：

- 不再是“功能拼装阶段”；
- 也不再是“只会冒烟”的阶段；
- 已进入“可以反复采集真实 benchmark 数据并基于数据调参”的阶段。

这是一个很重要的节点，因为它意味着后续的任何性能讨论都可以绑定真实 CSV / bag / summary，而不是只依赖日志直觉。

### 5.2 从性能状态看

当前仍应判断为：

> benchmark 体系成熟度已达到可用；
> 高速性能本身尚未达到可汇报为“已完成优化”的程度。

原因有三点：

1. 真实规划速度上限还是 `1.2 m/s`，当前还只是保守档基线。
2. 两个版本都以 `emergency_stop` 结束，说明安全边界还未被稳定控制。
3. V2 出现了 `drone_1` 低重规划成功率和 `drone_2` 低安全裕度两个明显风险点。

### 5.3 从对比结论看

当前最稳妥的汇报表述应该是：

- V1 在保守基线下呈现出更轻的规划负载与更高的整体稳定性。
- V2 在同一基线下表现出更高的平均速度，但在重规划成功率与安全裕度上暴露出更明显的问题。
- 因此，V2 当前更像“潜力版本”，而不是“已经全面优于 V1 的版本”。

---

## 6. 调参建议

以下建议按优先级排列，优先处理能避免“错误结论”的问题，再处理真正的性能问题。

### 6.1 P0：先对齐 benchmark 元数据与真实速度参数

#### 建议

- 将 `benchmark_default_vmax` 与真实实验使用的 `planner_max_vel` 对齐。

#### 原因

- 当前 `benchmark_default_vmax = 10.0`，而真实 `planner_max_vel = 1.2`。
- 这会导致：
  - bag 文件名带有 `vel10`，容易让人误解为 10m/s 档实验；
  - failure prediction 的解释语境不清晰；
  - 对导师或外部读者不够严谨。

#### 建议动作

- 保守基线实验时，将 `benchmark_default_vmax` 改为 `1.2`。
- 中速档实验时，将其同步改为 `2.0`、`3.0` 或 `5.0`。
- 禁止再出现“实际速度 1.2，但报告写 vel10”的情况。

### 6.2 P1：先把 V2 的安全边界拉回到可控区

#### 建议

- 将 V2 `swarm_clearance` 从 `0.35` 提高到 `0.45`，必要时到 `0.50`。
- 同步保证 `benchmark_safety_margin_threshold` 跟随该值变化。

#### 原因

- 当前 V2 `drone_2` 的最小安全裕度只有 `0.2712 m`。
- 当前 V2 的默认避碰间距本身就比 V1 更激进。
- 如果在这个基础上直接推高速档，很容易把问题放大成 emergency stop 或近碰风险。

#### 适用结论

- 这一步优先级高于单纯追求更高速度。
- 如果这一步不做，高速结果很难被解释为“算法性能问题”还是“安全参数本身过松”。

### 6.3 P1：压低 V2 的重规划压力，先把成功率拉回到 90% 以上

#### 建议

- 首先不要加速，先试一轮更保守的 V2 动力学上限：
  - `planner_max_acc: 2.8 -> 2.2 ~ 2.4`
  - `planner_max_jerk: 4.0 -> 3.5 ~ 4.0`
- 如果需要继续保守，可把 `planner_max_vel` 暂时维持在 `1.2` 不动。

#### 原因

- 当前 V2 `drone_1` 的 `replan_success_rate = 39.47%`，已经不属于“偶发不稳定”，而是结构性高风险信号。
- 在这种情况下继续推高速档，只会让定位问题、算力问题和避碰问题耦合到一起，无法判断根因。

#### 目标

- 先把 V2 最差单机重规划成功率拉回 `>= 90%`。
- 再讨论速度上探，否则数据价值有限。

### 6.4 P2：控制层做小步修正，不要大改

#### 建议

- `hover_percent` 允许在 `0.56 ~ 0.60` 小步校准。
- `Kv` 可以从 `2.0` 试到 `2.2`，但每次只改一档。
- `Kp` 暂不建议大动，除非出现明显拖尾或跟踪过软。

#### 原因

- 当前两版 `actuator_ratio_mean` 都在 `0.72` 左右，说明控制输出偏高但还没有进入饱和。
- `actuator_saturation_ratio = 0.0`，说明现在不是“已经打满”，而是“长期工作点偏高”。
- 这时最怕一次性同时改 `hover_percent`、`Kp`、`Kv`，会让结果无法归因。

#### 判断方法

- 如果改 `hover_percent` 后平均推力占比下降，但 tracking error 没恶化，说明有收益。
- 如果改 `Kv` 后 control lag 下降且无明显振荡，可以保留。
- 如果 tracking error 变好但 jerk 积分明显上升，则说明轨迹更硬，需谨慎。

### 6.5 P2：建立真正的高速升级梯度，而不是一步切到 10m/s

建议采用四级升档，而不是直接跳到 `10.0`：

#### 阶段 A：当前基线

- `planner_max_vel = 1.2`
- 目标：确认全链稳定、artifact 稳定、V2 成功率可接受

#### 阶段 B：低风险提速

- `planner_max_vel = 2.0`
- `planner_max_acc = 3.2`
- 目标：验证 tracking error 和 safety margin 是否仍可控

#### 阶段 C：中速 benchmark

- `planner_max_vel = 3.0`
- `planner_max_acc = 4.0`
- 目标：开始观察两版算法差异是否扩大

#### 阶段 D：高压档

- `planner_max_vel = 5.0`
- 只有在前三级满足门槛后才继续

### 6.6 建议的升档门槛

每次提速前，至少满足以下条件：

- 所有无人机 `stop_reason` 不应是 `emergency_stop`，或者 emergency stop 只允许极少数一次性偶发。
- 最差单机 `replan_success_rate >= 95%`。
- 最差单机 `min_safety_margin_m >= 当前 swarm_clearance + 0.05`。
- `control_lag_p95_ms < 100 ms`。
- `actuator_ratio_mean < 0.75` 且 `actuator_saturation_ratio = 0`。

如果不满足，就不应该继续升档。

---

## 7. 参数修改入口建议

### 7.1 快速实验入口

适合直接在 launch 命令里覆盖：

- `planner_max_vel`
- `planner_max_acc`
- `planner_max_jerk`
- `swarm_clearance`
- `benchmark_default_vmax`

示例：

```bash
source tools/source_phase1_env.sh
./test_swarm_top_level_runtime_health.sh \
  --version v2 \
  --output-dir benchmark_artifacts/manual_trials/v2_case_a \
  --wait-for-session-complete \
  -- \
  planner_max_vel:=2.0 \
  planner_max_acc:=3.2 \
  swarm_clearance:=0.45 \
  benchmark_default_vmax:=2.0
```

### 7.2 稳定配置入口

如果目标是形成长期 profile，优先写入：

- `src/clean_uav_core/config/swarm_config.yaml`

适合纳入 profile 的参数包括：

- `max_vel`
- `max_acc`
- `max_jerk`
- `planning_horizon`
- `replan_time`
- `lambda_smooth`
- `grid_map_obstacles_inflation`
- `swarm_clearance`
- `swarm_weight`

### 7.3 控制参数入口

控制器相关参数入口：

- `src/clean_uav_core/config/phase1_px4ctrl_no_rc.yaml`
- `src/px4ctrl/config/ctrl_param_fpv.yaml`

优先级建议：

- 先改 `hover_percent`
- 再小步改 `Kv`
- 最后才考虑 `Kp`

---

## 8. 当前建议的下一轮实验组合

### 方案 A：V2 稳定性修复优先

- `planner_max_vel = 1.2`
- `planner_max_acc = 2.2`
- `swarm_clearance = 0.45`
- `benchmark_default_vmax = 1.2`

目标：先把 V2 最差单机成功率拉高，观察 emergency stop 是否减少。

### 方案 B：公平对比基线

- V1 / V2 同时设：
  - `planner_max_vel = 2.0`
  - `planner_max_acc = 3.2`
  - `swarm_clearance = 0.5`
  - `benchmark_default_vmax = 2.0`

目标：做一轮真正可解释的公平对比，不让安全阈值差异干扰结论。

### 方案 C：V2 专项修复后再试中速档

- 在方案 A 连续通过后，再推到：
  - `planner_max_vel = 3.0`
  - `planner_max_acc = 4.0`
  - `swarm_clearance = 0.5`

目标：验证 V2 是否在更高速度段体现出真正优势，而不是只在低速段出现不稳定。

---

## 9. 本文结论

当前 benchmark 项目已经从“系统搭建阶段”推进到了“可基于真实数据做调参闭环”的阶段，这是本轮最大的进展。

但就性能本身而言，当前最重要的工作不是直接追逐 10m/s，而是：

1. 对齐 benchmark 元数据和真实速度参数。
2. 先把 V2 的稳定性与安全边界拉回到可解释区。
3. 再进行分级升档和公平对比。

只有这样，后续的高速结论才会既可信，又适合对导师和外部评审汇报。