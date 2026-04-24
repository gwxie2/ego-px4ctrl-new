# 05_BENCHMARK - 性能测试与参数研究

本目录包含**性能基准测试、参数调优和性能分析**的相关文档。

## 📚 目录内容

### [benchmark_documentation_quick_reference_CN.md](benchmark_documentation_quick_reference_CN.md)
**大小**：9K | **阅读时间**：10-15 分钟

**Benchmark 文档导航索引** — 快速定位你需要的资料。

**内容**：
- 两部分 Benchmark 文档的分类和用途
- 快速导航地图
- 适用场景速查表

**何时阅读**：
- 不知道有什么 benchmark 资源
- 第一次接触系统的性能测试框架
- 需要快速找到特定的指标或参数说明

**核心分类**：
```
Part 1：汇报文档（面向决策层和非专家）
  └─ phase_a_d_benchmark_teacher_report_CN.md
     ├─ 四个 Phase 的研究目标
     ├─ 完整的性能对比表
     └─ 系统能力边界总结

Part 2：深度解读（面向技术人员）
  └─ benchmark_metrics_and_design_details_CN.md
     ├─ 每个指标的完整定义和改善方式
     ├─ 每个方案为什么这样设计
     └─ 参数调优的系统方法
```

---

### [benchmark_metrics_and_design_details_CN.md](benchmark_metrics_and_design_details_CN.md) ⭐ 性能优化必读
**大小**：32K | **阅读时间**：60-90 分钟

**最详细的性能指标和参数设计文档**。

**核心内容**：
- **9 大性能指标详解**：
  - 重规划成功率
  - 最小安全间距
  - 跟踪误差 P95
  - 控制滞后
  - 其他关键指标

- **参数相关性矩阵**：一个参数改变会影响哪些指标

- **调优工作流**：从问题诊断到参数选择的系统方法

- **各 Stage 方案设计**：
  - Stage A（速度梯度基线）：为什么选 1.2/3.0/5.0 m/s
  - Stage B（稳定性恢复）：8 个方案对比
  - Stage C（时间-光滑权衡）：time_priority vs smooth_priority
  - Stage D（极限速度）：14 个方案完整迭代

**何时阅读**：
- 需要理解性能指标（tracking_error、replan_count 等）
- 计划改进系统性能
- 要选择合适的参数组合
- 面临性能瓶颈需要诊断原因

**重点内容**：⭐ 第 §3 章"参数相关性矩阵"和 §4 章"调优工作流"

**例子**：如果你看到 `tracking_error` 过大：
```
1. 查表：tracking_error ← Kp/Kv 增益、planning_horizon、max_vel
2. 诊断：是 PID 增益低还是规划不合理？
3. 决策：逐步增加 Kp，或增大 planning_horizon
4. 验证：运行 benchmark 对比新旧参数的效果
```

---

### [swarm_research_campaign_usage_CN.md](swarm_research_campaign_usage_CN.md)
**大小**：16K | **阅读时间**：30-40 分钟

**研究 Campaign 框架的完整使用说明**。

**功能**：
- 用 YAML 配置管理四个研究阶段的参数档位
- 自动生成专用 launch 并调用验证过的 runtime health 链路
- 为每次实验记录参数和元数据
- 运行后自动汇总统计和生成离线分析

**使用场景**：
- 系统化的参数研究（而不是一次性的单个调试）
- 对标测试（重复运行同一配置多次，评估稳定性）
- 参数敏感度分析（识别关键参数）
- 跨方案对比（Stage A vs B vs C vs D）

**何时阅读**：
- 计划进行参数研究和对标测试
- 要生成性能报告和对比表
- 需要系统化地扫描参数空间

**快速开始**：
```bash
# 运行一个基准 case（3 个独立 run）
python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_g \
  --case g11_horizon15 \
  --num_runs 3

# 查看结果
cat benchmark_artifacts/research_profiles/stage_g/g11_horizon15/campaign_index.json
```

**输出结构**：
```
benchmark_artifacts/research_profiles/stage_g/g11_horizon15/
├── run_01/
│   ├── run_spec.json             # 参数配置快照
│   ├── metrics.csv               # 时间序列指标
│   ├── health_check.json         # 是否成功、错误摘要
│   └── full_rosbag.bag           # 完整数据，支持离线分析
├── run_02/
├── run_03/
└── campaign_index.json           # 3 个 run 的均值、标准差、min/max
```

**后续分析**：
```bash
# 参数敏感度
python3 tools/benchmark_research/parameter_sensitivity_analysis.py \
  --campaign_dir benchmark_artifacts/.../g11_horizon15

# Pareto 前沿（flight_time vs jerk_integral）
python3 tools/benchmark_research/pareto_frontier_analysis.py \
  --campaign_dir benchmark_artifacts/.../g11_horizon15

# 风险评分
python3 tools/benchmark_research/risk_statistics_analysis.py \
  --campaign_dir benchmark_artifacts/.../g11_horizon15
```

---

## 📊 性能指标速查表

| 指标 | 单位 | 正常范围 | 含义 |
|------|------|--------|------|
| `tracking_error` | m | 0.1-0.5 | 实际位置与期望位置的偏差，越小越好 |
| `replan_count` | count | 5-20 | 任务期间的总重规划次数，太高说明环境复杂 |
| `replan_latency` | ms | 50-200 | 单次规划耗时，与场景复杂度相关 |
| `flight_time` | s | 60-120 | 完成任务的总耗时，受速度和路径影响 |
| `jerk_integral` | (m/s³)·s | 1-10 | 轨迹的加加速度积分，反映舒适度 |
| `min_safety_dist` | m | >0.3 | 最小安全间距（与障碍物），应该 > 通货膨胀距离 |
| `optimizer_failure` | count | 0 | 轨迹优化失败次数，应该为 0 |
| `collision_events` | count | 0 | 碰撞事件，应该为 0 |

---

## 🎯 快速参考

**我想理解"tracking_error"是什么** → [benchmark_metrics_and_design_details_CN.md](benchmark_metrics_and_design_details_CN.md)

**我想找个快速链接到各种资料** → [benchmark_documentation_quick_reference_CN.md](benchmark_documentation_quick_reference_CN.md)

**我要做参数研究** → [swarm_research_campaign_usage_CN.md](swarm_research_campaign_usage_CN.md)

**我要对标测试两个参数配置** → 阅读 [swarm_research_campaign_usage_CN.md](swarm_research_campaign_usage_CN.md) 再运行 campaign

---

## 📈 研究工作流

### Step 1：理解性能指标
阅读 [benchmark_metrics_and_design_details_CN.md](benchmark_metrics_and_design_details_CN.md) 的 §1 章

### Step 2：确定问题
根据当前的指标对比，识别最大的瓶颈（accuracy / speed / smoothness）

### Step 3：选择参数组合
参考 [benchmark_metrics_and_design_details_CN.md](benchmark_metrics_and_design_details_CN.md) 的 §3 相关性矩阵，选择可能有帮助的参数

### Step 4：运行对标测试
```bash
# 运行原始配置（baseline）
python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_g --case g11_horizon15 --num_runs 3

# 运行改进配置
# （修改 src/clean_uav_core/config/... 然后）
python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_g --case my_new_config --num_runs 3
```

### Step 5：对比分析
```bash
# 手工对比两个 campaign_index.json，或用脚本
python3 tools/benchmark_research/pareto_frontier_analysis.py \
  --campaign_dir benchmark_artifacts/.../g11_horizon15

python3 tools/benchmark_research/parameter_sensitivity_analysis.py \
  --campaign_dir benchmark_artifacts/.../my_new_config
```

### Step 6：迭代或确认
- 如果有改进，考虑进一步调优
- 如果没改进甚至更差，回滚并尝试其他参数
- 如果达到目标，更新基线配置

---

## 📋 Benchmark 运行检查清单

在运行任何 benchmark campaign 前：

- [ ] 基础系统能正常运行吗？（smoke test 通过）
- [ ] 修改的参数值在合理范围内吗？
- [ ] 我清楚这个参数会影响什么指标吗？
- [ ] 我有足够的磁盘空间吗？（每个 run 可能产生 100-500 MB 数据）
- [ ] 我的机器能支持所需的 CPU/内存吗？（特别是多机场景）
- [ ] 我知道这次 campaign 的预计运行时间吗？

---

**推荐接下来阅读**：[06_ARCHIVED](../06_ARCHIVED/)（如果需要历史参考）或 [主文档索引](../00_DOCUMENTATION_INDEX.md)
