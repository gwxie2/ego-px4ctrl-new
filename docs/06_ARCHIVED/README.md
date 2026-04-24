# 06_ARCHIVED - 存档文档（历史参考）

本目录包含**日期标记的研究报告和过期文档**，仅供**历史参考**和**问题排查**。

## ⚠️ 重要提示

**不建议新手或日常开发者阅读这个目录**。这里的文档：

1. 可能包含**过时的参数值**（已被新的更优方案替代）
2. 可能记录已**修复的问题**（不再发生）
3. 可能涉及**旧版本的系统设计**（已重构）
4. 文件名中包含**日期戳**（表示特定时间点的快照）

这个目录被保留的原因是：
- 保留完整的历史记录和演进过程
- 当面对"之前出现过的问题"时，可以查阅相同问题的解决方案
- 追踪系统的改进和败笔背后的原因

---

## 📚 存档文件分类

### 性能研究报告（按阶段）

#### Phase A / B / C / D 研究
这些是速度梯度、稳定性、时间-光滑权衡和极限速度的详细研究报告。

**何时查阅**：
- 研究历史上为什么选择了特定参数
- 对某个参数的改动有疑惑，想看之前的实验记录
- 学习之前的参数调优思路（虽然可能已落后）

**包含文件**：
- `phase_a_d_benchmark_teacher_report_CN.md` (28K) — 整体汇报
- `phase_a_d_benchmark_cpu_freq_detailed_CN.md` (13K) — CPU 频率分析
- `stage_a_speed_arrival_time_comparison_CN.md` — Speed ramping 基线
- `stage_d_root_cause_mitigation_execution_plan_CN.md` — 极限速度修复
- `stage_e_5mps_benchmark_analysis_report_CN.md` — 5m/s 分析

### 多机集群研究

#### 6 机集群的调试报告
这些是早期多机系统的问题分析和调试过程。

**何时查阅**：
- 遇到"多机间通信延迟"或"deadlock"问题
- 相同的故障特征能在这里找到原始诊断

**包含文件**：
- `swarm_6uav_ablation_tuning_CN.md` (10K) — 消融实验
- `swarm_6uav_deadlock_and_mavros_warning_analysis_CN.md` (14K) — Deadlock 和 MAVROS 警告分析
- `drone0_replay_diagnosis_and_fix_CN.md` (17K) — 单机回放诊断

### Benchmark 框架演进

#### 多个版本的 Benchmark 文档
这些是 benchmark 框架的不同版本和演进记录。

**为什么有这么多？**
- 框架在不断完善，每个版本都记录了进展
- 某些版本可能有不同的目标受众（学术汇报 vs 工程应用）

**何时查阅**：
- 理解 benchmark 框架为什么这样设计
- 学习性能测试的方法论
- 找到特定阶段的完整性能数据

**包含文件**：
- `swarm_benchmark_implementation_and_quick_reference_CN.md` — 实现说明
- `swarm_benchmark_advisor_briefing_CN.md` — 顾问汇报版
- `swarm_benchmark_comprehensive_report_CN.md` — 综合报告
- `swarm_benchmark_status_analysis_and_tuning_CN.md` — 状态分析
- `swarm_benchmark_rosbag_offline_replay_and_campaign_update_CN.md` — Rosbag 回放指南
- `swarm_benchmark_visual_quick_reference_CN.md` — 可视化快速参考

### 集群系统重构与实现

#### V1 → V2 的演进
这些文档记录了系统从 V1 到 V2 的重大重构。

**何时查阅**：
- 想理解 V1 和 V2 的区别
- 追踪某个重要功能的演变过程

**包含文件**：
- `swarm_refactor_implementation_plan_CN.md` (12K) — V2 重构计划
- `swarm_v2_sandbox_CN.md` (5.7K) — V2 沙盒测试
- `swarm_trajectory_sharing_fix_and_ab_CN.md` (5.4K) — 轨迹共享修复
- `swarm_yaml_workflow_CN.md` (7.3K) — YAML 工作流
- `swarm_search_development_progress_CN.md` (17K) — 搜索开发进度

### Phase 3 的健康报告

#### VINS 集成验证
这些是 Phase 3 VINS 集成的验证报告。

**何时查阅**：
- 理解 VINS 集成的历史和挑战
- 遇到 VINS 相关问题，查看之前的诊断

**包含文件**：
- `PHASE3_HEALTH_REPORT_20260315_CN.md` — Phase 3 工作日志
- `PHASE3_HEALTH_REPORT_RESTART_2RUNS_20260315_CN.md` — 重启验证报告

### 已完成的功能和修复

#### 其他归档文档
- `px4ctrl_and_system_changes_CN.md` — PX4Ctrl 改动历史
- `depth_noise_schematic_CN.md` — 深度传感器噪声（已有图片版）
- `phase1_dependency_inventory.md` — 依赖清单（已过时）
- `launch_phase*.md` — 旧版 launch 说明（信息已迁移）

---

## 🔍 如何查找历史问题

假设你遇到了一个问题，想看之前有没有人遇到过：

### 场景 1：多机间的通信延迟很大
```
查找步骤：
1. 首先看 03_MULTIUAV_PHASES/phase2_runtime_log_reading_guide_CN.md
2. 如果问题类似"6 机场景特别卡"，查看：
   06_ARCHIVED/swarm_6uav_deadlock_and_mavros_warning_analysis_CN.md
3. 尝试相同的诊断步骤和解决方案
```

### 场景 2：跟踪误差在某个速度档突然增大
```
查找步骤：
1. 查看 05_BENCHMARK/benchmark_metrics_and_design_details_CN.md
2. 看历史上在哪个 Stage 遇到过相同症状：
   06_ARCHIVED/stage_d_root_cause_mitigation_execution_plan_CN.md
3. 学习当时是怎么修复的
```

### 场景 3：不知道某个参数是怎么确定的
```
查找步骤：
1. 查看 05_BENCHMARK/swarm_research_campaign_usage_CN.md
2. 查看相应 Stage 的研究报告：
   06_ARCHIVED/phase_a_d_benchmark_teacher_report_CN.md
3. 找到该参数在哪个 Stage 被测试过
```

---

## 📋 存档文件查找表

| 文件名 | 日期 | 内容 | 查找关键词 |
|--------|------|------|-----------|
| PHASE3_HEALTH_REPORT_*.md | 2026-03-15 | VINS 集成验证 | VINS / Phase 3 |
| swarm_6uav_deadlock_*.md | 历史 | 多机 deadlock | 多机 / 卡顿 / deadlock |
| phase_a_d_benchmark_*.md | 历史 | 性能研究全汇报 | 性能 / 基准 / 对标 |
| stage_*_*.md | 历史 | 各 Stage 的详细分析 | 特定阶段 / 特定速度 |
| swarm_benchmark_*.md | 历史 | Benchmark 框架演进 | Benchmark / 实现 |

---

## ⚠️ 使用存档文档时的注意事项

1. **参数可能已过时** — 不要直接复制存档中的参数值，先看最新的 05_BENCHMARK 和 HANDOVER_README

2. **问题可能已修复** — 存档中的 bug 报告可能已在最新版本中解决

3. **系统架构可能已变** — V1 的设计在 V2 中可能完全不同

4. **对比最新文档** — 如果存档中说的和最新文档矛盾，以最新文档为准

---

## 🗂️ 完整文件列表

```
06_ARCHIVED/
├── 性能研究
│   ├── phase_a_d_benchmark_teacher_report_CN.md
│   ├── phase_a_d_benchmark_cpu_freq_detailed_CN.md
│   ├── stage_a_speed_arrival_time_comparison_CN.md
│   ├── stage_d_root_cause_mitigation_execution_plan_CN.md
│   └── stage_e_5mps_benchmark_analysis_report_CN.md
├── 多机集群
│   ├── swarm_6uav_ablation_tuning_CN.md
│   ├── swarm_6uav_deadlock_and_mavros_warning_analysis_CN.md
│   └── drone0_replay_diagnosis_and_fix_CN.md
├── Benchmark 框架
│   ├── swarm_benchmark_implementation_and_quick_reference_CN.md
│   ├── swarm_benchmark_advisor_briefing_CN.md
│   ├── swarm_benchmark_comprehensive_report_CN.md
│   ├── swarm_benchmark_status_analysis_and_tuning_CN.md
│   ├── swarm_benchmark_rosbag_offline_replay_and_campaign_update_CN.md
│   └── swarm_benchmark_visual_quick_reference_CN.md
├── V2 重构
│   ├── swarm_refactor_implementation_plan_CN.md
│   ├── swarm_v2_sandbox_CN.md
│   ├── swarm_trajectory_sharing_fix_and_ab_CN.md
│   ├── swarm_yaml_workflow_CN.md
│   └── swarm_search_development_progress_CN.md
├── Phase 3 VINS
│   ├── PHASE3_HEALTH_REPORT_20260315_CN.md
│   └── PHASE3_HEALTH_REPORT_RESTART_2RUNS_20260315_CN.md
└── 其他
    ├── px4ctrl_and_system_changes_CN.md
    ├── depth_noise_schematic_CN.md
    ├── phase1_dependency_inventory.md
    ├── launch_phase1_minimal_demo.md
    ├── launch_phase2_dual_uav_stack.md
    └── launch_phase2_px4_multi_sim.md
```

---

**提示**：如果你在这个目录中花费了太多时间，请回到 [主文档索引](../00_DOCUMENTATION_INDEX.md) 重新开始！😊
