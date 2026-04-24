# 基准实验文档快速参考索引

## 两部分文档构成

### 📊 Part 1: 汇报文档
**[docs/phase_a_d_benchmark_teacher_report_CN.md](phase_a_d_benchmark_teacher_report_CN.md)** — 446 行，~23 KB

**适用场景**：向老师/领导汇报整体结果

| 章节 | 内容 | 适合人群 |
|------|------|---------|
| §1. 研究背景 | 四个 Phase 的目标和递进逻辑 | 管理层、新人 |
| §2. 系统架构 | 规划–控制–仿真三层数据流 | 系统集成者 |
| §3. RViz 回放配置 | rosbag 可视化说明 + 快速启动命令 | 演示/验证人员 |
| §4. Phase A 分析 | 速度梯度压力测试结果 + 表格 | **所有人** |
| §5. Phase B 分析 | 8 个稳定性方案对比 + 最优方案 | **所有人** |
| §6. Phase C 分析 | 时间–平滑权衡详解 | 规划算法研究者 |
| §7. Phase D 分析 | **★ 极限速度探索 + 14 个方案完整对照表** | **所有人** |
| §8. 跨阶段综合 | 系统能力边界、参数敏感性总结 | 系统设计者 |
| §9. 扩展至 10 m/s 结论 | 可行性、差距、建议行动 | 决策层 |
| §10. 归档结构 | 数据位置 + 分析工具命令 | 数据维护者 |

---

### 📖 Part 2: 补充详解文档
**[docs/benchmark_metrics_and_design_details_CN.md](benchmark_metrics_and_design_details_CN.md)** — 764 行，~30 KB

**适用场景**：深入理解每个指标的含义和每个方案的设计原理

| 章节 | 内容 | 适合人群 |
|------|------|---------|
| §1.1–1.9 指标详解 | 每个指标的完整定义、参考范围、改善方式 | **深度理解者必读** |
| §1.1 重规划成功率 | 定义 + 参考范围 + 关系矩阵 + 改善方式 | 规划调试 |
| §1.2 最小安全间距 | 物理含义 + 临界值分析 + 改善方式 | 安全分析 |
| §1.3 跟踪误差 P95 | 与控制器性能的关系 + 物理来源 | 控制调优 |
| §1.4 控制滞后 P95 | 13 个可能的恶化来源 | 系统实时性诊断 |
| §1.5–1.6 其他指标 | jerk / violation / optimizer_failure | 细节控制 |
| §2 各 Stage 方案设计 | **每个方案为什么这样设计，参数怎么改** | **参数调优必读** |
| §2.1 Stage A 设置 | 为什么用 1.2/3.0/5.0，发现了什么 | baseline 理解 |
| §2.2 Stage B 8 方案 | baseline → high_freq → perception_series | 稳定性调优参考 |
| §2.3 Stage C 3 方案 | time_priority 为何失败、smooth 为何成功 | 规划器权重选择 |
| §2.4 Stage D 完整迭代 | **kv 分支为何死掉、clearance 微调的甜蜜点** | **高速调优必读** |
| §3 参数相关性矩阵 | 一个参数改变会影响哪些指标 | 快速诊断 |
| §4 调优工作流 | 从问题诊断到参数选择的系统方法 | 下一次调优的 SOP |
| §5 场景权重建议 | 物流/巡检/研究三种场景的指标优先级 | 应用选型 |

---

## 快速导航地图

### 🎯 "我想看…"

| 要求 | 跳转位置 |
|------|---------|
| **整体结果汇总表** | 汇报 §4–7 中的对照表 |
| **10 m/s 为什么选 headless** | 汇报 §7.2 |
| **为什么 d_rigid_clearance_balance 是最优的** | 补充 §2.4 或 汇报 §7.5 |
| **参数改一个，其他指标咋变** | 补充 §3（相关性矩阵）|
| **怎样快速诊断 rosbag** | 汇报 §3（RViz 回放） |
| **每个指标如何改善** | 补充 §1.1–§1.9 的"改善方式"列表 |
| **下次调优从哪里开始** | 补充 §4（工作流）|
| **为什么 Stage B 的 lambda_light 看起来成功率高但还是不推** | 补充 §2.2（lambda 系列分析） |
| **kv 参数为什么没用** | 补充 §2.4（kv 分支失败原因） |
| **我们离"完美"还差多少** | 汇报 §9.2（与理想状态差距） |

---

## 核心数据表总结

### 最关键的三个表

#### 表 1: Phase A 速度梯度
```
速度    成功率  间距    滞后         问题
1.2 m/s 0.9412 0.397m  84.7ms      充裕（基准）
3.0 m/s 0.8506 0.316m  86.0ms      略差
5.0 m/s 1.0000 0.319m  1825.0ms ⚠️  滞后爆炸（发现性能悬崖）
```

#### 表 2: Phase B 最优方案（8 方案中选）
```
方案                  成功率  间距    滞后      说明
baseline              0.9375 0.223m  85ms      不稳定
high_freq_replan      0.9905 0.079m  7549ms    成功率高但滞后炸
✅ perception_clearance_plus  0.9935 0.344m  7368ms  ★ 最优
```

#### 表 3: Phase D 核心对照
```
方案                        成功率  间距    跟踪P95 滞后    说明
d_rigid_control（基线）     0.7846 0.130m  9.92m   6690ms  参考点
d_lambda_kv_mid             0.8299 0.099m  10.28m  12216ms ❌ 排除
d_rigid_clearance_micro     0.8869 0.091m  11.33m  10405ms 中间点
✅ d_rigid_clearance_balance 0.9206 0.289m  0.693m  7659ms  ★ 最优（+17% 成功率，−93% 跟踪误差！）
🟢 d_rigid_clearance_shallow 0.9091 0.269m  0.774m  7053ms  备选（低滞后）
```

---

## 推荐阅读顺序

### 💼 如果你是老师/管理者
1. 汇报 §1（背景）
2. 汇报 §9（结论与差距）
3. 汇报 §7（极速探索，有表）
4. 补充 §5（应用场景权重）

**预计时间**：20 分钟

---

### 🔬 如果你是控制/规划研究者
1. 汇报 §2（系统架构）
2. 补充 §1.1–1.4（关键指标深度）
3. 汇报 §7.3–7.6（Phase D 完整迭代）
4. 补充 §2.4（调优甜蜜点分析）
5. 补充 §4（工作流）

**预计时间**：1 小时

---

### ⚙️ 如果你要做下一轮参数调优
1. 补充 §4（问题诊断流程）
2. 补充 §3（参数相关性矩阵）
3. 补充 §2.4 中的"参数对比表"（选择下一个要改的参数）
4. 参考汇报 §7.3 中的"参数迭代历程"（看前人踩过的坑）

**预计时间**：30 分钟准备，然后开始实验

---

### 🎓 如果你要复现某个特定实验
1. 汇报 §3（RViz 回放命令）
2. 汇报 §10（归档结构）
3. 运行 replay 脚本：
   ```bash
   tools/benchmark_research/replay_benchmark_rosbag.sh \
     --input benchmark_artifacts/research_profiles/stage_d/d_rigid_clearance_balance/run_01
   ```
4. 对比 rosbag 中的话题（`/drone_N/odom`、`/drone_N/position_cmd` 等）与汇报表格中的指标

**预计时间**：10 分钟

---

## 指标速查表

| 指标英文名 | 缩写 | 中文 | 参考范围（10m/s） | 好/坏 | 详解位置 |
|-----------|------|------|-----------------|------|---------|
| Replan Success Rate | SR | 重规划成功率 | > 0.90 | ↑好 | 补充 §1.1 |
| Min Safety Margin | MSM | 最小安全间距 | > 0.25 m | ↑好 | 补充 §1.2 |
| Tracking Error P95 | TE P95 | 跟踪误差 P95 | < 1.0 m | ↓好 | 补充 §1.3 |
| Control Lag P95 | LAG P95 | 控制滞后 P95 | < 8000 ms | ↓好 | 补充 §1.4 |
| Jerk Integral | JERK | 抖动积分 | < 150 | ↓好 | 补充 §1.5 |
| Safety Violation | VIOL | 违规次数 | < 200 | ↓好 | 补充 §1.6 |
| Optimizer Failure | OPT_FAIL | 优化失败 | < 5% | ↓好 | 补充 §1.7 |
| Collision Trigger | COL_TRIG | 紧停次数 | 0 | ↓好 | 补充 §1.8 |

---

## 文件位置速索

| 内容 | 文件路径 |
|------|---------|
| 汇报文档 | `docs/phase_a_d_benchmark_teacher_report_CN.md` |
| 补充文档 | `docs/benchmark_metrics_and_design_details_CN.md` |
| RViz 回放配置 | `src/clean_uav_core/rviz/benchmark_rosbag_replay.rviz` |
| 回放脚本 | `tools/benchmark_research/replay_benchmark_rosbag.sh` |
| 所有实验数据索引 | `benchmark_artifacts/research_profiles/campaign_index.json` |
| 实验数据（CSV版） | `benchmark_artifacts/research_profiles/campaign_index.csv` |
| 参数矩阵定义 | `src/clean_uav_core/config/benchmark_research/campaign_matrix_v2.yaml` |
| 具体配置示例 | `src/clean_uav_core/config/benchmark_research/stage_d_extreme_10ms_v2.yaml` |

---

## 常见问题速答

#### Q1: 为什么 Stage D 必须用 headless？
→ 汇报 §7.2 表格 / 补充 §2.4（解释 lambda 分支为何失效，反证 headless 的必要性）

#### Q2: d_rigid_clearance_balance 和 d_rigid_clearance_shallow 怎么选？
→ 汇报 §7.6 / 补充 §2.4（双候选对照）→ 总结：balance 安全性优先，shallow 若要低滞后

#### Q3: Stage C 为什么 time_priority 会这么差？
→ 补充 §2.3（跟踪误差从 0.589 m 跳到 18.833 m 的物理原因）

#### Q4: 怎样快速复现一个 rosbag？
→ 汇报 §3（一行命令）+ RViz 看 drone 轨迹和障碍地图

#### Q5: 下次调优从哪个参数开始？
→ 补充 §4（工作流）→ 补充 §3（相关性矩阵）→ 补充 §2.4（参数对比表）

#### Q6: 所有 14 个 Stage D 方案的设计意图分别是什么？
→ 补充 §2.4（完整的微调历程表）

#### Q7: 怎样判断某个配置是否"好"？
→ 补充 §5（根据应用场景选权重）

---

## 两文档之间的交叉引用

| 汇报中的位置 | 对应补充文档 |
|-------------|-------------|
| 汇报 §7（完整 Stage D 表）| 补充 §2.4（每一行方案的设计详解） |
| 汇报 §8.2（参数敏感性） | 补充 §3（参数相关性矩阵） |
| 汇报 §9（建议行动） | 补充 §4（工作流 SOP） |
| 汇报 §3（RViz 回放） | 补充 无（纯操作指南） |

---

*快速参考索引 — Last Updated: 2026-04-07*
