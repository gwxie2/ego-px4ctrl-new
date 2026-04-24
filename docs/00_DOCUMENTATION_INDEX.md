# ego-px4ctrl-new 文档完整索引

> **最后更新**：2026-04-24  
> **目的**：为所有开发者提供清晰的文档导航，避免重复阅读和误读过时文档。

---

## 📖 阅读层次说明

为了帮助你快速找到需要的内容，我们将文档分为四个层次：

| 层次 | 适用群体 | 速度 | 说明 |
|------|---------|------|------|
| **【必须读】** 🔴 | 所有用户 | 15-30 分钟 | 运行系统和理解基础的最小必须知识 |
| **【可以读】** 🟡 | 特定场景用户 | 1-2 小时 | 针对你们的使用场景（多机/性能优化等）的深入指南 |
| **【扩展读】** 🟢 | 研究开发者 | 2-4 小时 | 高级功能、性能优化、参数研究等深度内容 |
| **【排查读】** 🔵 | 遇到问题时 | 10-30 分钟 | 故障诊断、问题排查、错误日志解读 |

---

## 🎯 快速开始（推荐阅读顺序）

### 【必须读】矩阵 —— 所有用户必须
**预计时间：15-30 分钟**

**无论你是谁，下面这些文档都必须读：**

1. **[HANDOVER_README.md](../HANDOVER_README.md)** （根目录，必读）
   - ✅ 10 分钟快速复现（第 2 节）
   - ✅ 关键命令清单（第 4 节）
   - ✅ 调参红线和已知问题（第 5, 7 节）
   - ⏱️ **建议时间**：10-15 分钟

2. **系统基础理论**（15-20 分钟）
   - [系统架构设计](./01_FUNDAMENTALS/system_architecture_CN.md) — 5 层分解、数据流
   - [仓库目录结构](./01_FUNDAMENTALS/directory_structure_CN.md) — 代码在哪里

3. **第一次运行**（根据你的操作系统和环境，选一个）
   - [新手入门指南](./01_FUNDAMENTALS/beginner_guide_CN.md) — 完整的环境配置（仅 Linux/Ubuntu 新手需要，有经验者跳过）

---

### 【可以读】矩阵 —— 按你的场景选择
**预计时间：1-2 小时**

根据你的使用场景，选择相关的文档深入学习：

**场景 A：我要快速跑通单机演示**
- [Phase 1 运行链设计](./01_FUNDAMENTALS/phase1_runtime_chain_design.md)
- [无人机位置定义](./02_CONFIGURATION/uav_position_goal.md)

**场景 B：我要升级到多机系统**
- [Phase 2 多机架构](./03_MULTIUAV_PHASES/phase2_architecture_and_principles_CN.md)
- [运行日志分析](./03_MULTIUAV_PHASES/phase2_runtime_log_reading_guide_CN.md)

**场景 C：我要调整控制或规划参数**
- [PX4Ctrl 参数详解](./02_CONFIGURATION/phase1_px4ctrl_config_explanation_CN.md)
- [起飞调参经验](./02_CONFIGURATION/phase1_takeoff_tuning_CN.md)

**场景 D：我要为性能优化进行对标测试**
- [Benchmark 文档导航](./05_BENCHMARK/benchmark_documentation_quick_reference_CN.md)
- [Benchmark 快速参考](./05_BENCHMARK/swarm_research_campaign_usage_CN.md)

---

## 📚 完整文档分类（按阅读层次）

### 第一部分：【必须读】基础理论（01_FUNDAMENTALS）

这是系统设计的基石，包含架构、设计思想和初期规划。

| 文档 | 层次 | 大小 | 用途 |
|------|------|------|------|
| [system_architecture_CN.md](./01_FUNDAMENTALS/system_architecture_CN.md) | 🔴 必须读 | 14K | ⭐ 系统 5 层分解、数据流、消息格式定义 |
| [directory_structure_CN.md](./01_FUNDAMENTALS/directory_structure_CN.md) | 🔴 必须读 | 13K | ⭐ 仓库目录树、各目录作用说明 |
| [beginner_guide_CN.md](./01_FUNDAMENTALS/beginner_guide_CN.md) | 🟡 可以读 | 11K | ROS 环境配置、本地编译、第一个 demo（仅新手） |
| [phase1_runtime_chain_design.md](./01_FUNDAMENTALS/phase1_runtime_chain_design.md) | 🟡 可以读 | 2.7K | Phase 1 设计目标、无RC模式说明 |

**必须读说明**：
- system_architecture 和 directory_structure 是全局理解的基础
- 新手需要全部读，有经验开发者可快速浏览

---

### 第二部分：【可以读】配置与调参（02_CONFIGURATION）

涉及无人机的参数配置、起飞调试、位置初始化。

| 文档 | 层次 | 大小 | 用途 |
|------|------|------|------|
| [phase1_px4ctrl_config_explanation_CN.md](./02_CONFIGURATION/phase1_px4ctrl_config_explanation_CN.md) | 🟡 可以读 | 6.3K | 📋 px4ctrl 全参数手册（mass、hover_percentage、Kp/Kv等） |
| [uav_position_goal.md](./02_CONFIGURATION/uav_position_goal.md) | 🟡 可以读 | 2.3K | 📍 Phase 1 无人机起始位置和目标位置定义格式 |
| [phase1_takeoff_tuning_CN.md](./02_CONFIGURATION/phase1_takeoff_tuning_CN.md) | 🟢 扩展读 | 3.6K | 🎯 起飞调参经验和最佳实践（实机部署）|

**何时阅读**：
- 需要调整控制参数：阅读 phase1_px4ctrl_config_explanation_CN.md
- 配置新的无人机起始位置：阅读 uav_position_goal.md
- 准备实机部署：阅读 phase1_takeoff_tuning_CN.md

---

### 第三部分：【可以读】多机与高阶功能（03_MULTIUAV_PHASES）

涵盖 Phase 2/3/4 的多机协作、VINS 集成、动态目标等高阶功能。

| 文档 | 层次 | 大小 | 用途 |
|------|------|------|------|
| [phase2_architecture_and_principles_CN.md](./03_MULTIUAV_PHASES/phase2_architecture_and_principles_CN.md) | 🟡 可以读 | 7.1K | 🤝 多机数据流、命名空间、同步机制 |
| [phase2_runtime_log_reading_guide_CN.md](./03_MULTIUAV_PHASES/phase2_runtime_log_reading_guide_CN.md) | 🔵 排查读 | 7.6K | 🔍 如何读懂日志、定位多机故障 |
| [phase4_dynamic_goal_execution_CN.md](./03_MULTIUAV_PHASES/phase4_dynamic_goal_execution_CN.md) | 🟢 扩展读 | 3.4K | 🚀 动态目标发布和执行机制 |
| [PHASE3_VINS_CONTEXT_CN.md](./03_MULTIUAV_PHASES/PHASE3_VINS_CONTEXT_CN.md) | 🟢 扩展读 | 6.4K | 📡 VINS 与 EGO-Planner 融合方案 |

**何时阅读**：
- 从单机升级到多机：必读 phase2_architecture_and_principles_CN.md
- 遇到多机 Debug 困难：阅读 phase2_runtime_log_reading_guide_CN.md（【排查读】）
- 集成 VINS/外部定位：阅读 PHASE3_VINS_CONTEXT_CN.md
- 实现动态目标跟踪：阅读 phase4_dynamic_goal_execution_CN.md

---

### 第四部分：【扩展读】功能模块（04_FEATURES）

包含可选的高阶功能和实验性功能。

| 文档 | 层次 | 大小 | 用途 |
|------|------|------|------|
| [vlm_bridge_usage_CN.md](./04_FEATURES/vlm_bridge_usage_CN.md) | 🟢 扩展读 | 9.1K | 🎥 将摄像头像素投影到世界坐标、目标检测 |
| [vins_drift_monitor_usage_CN.md](./04_FEATURES/vins_drift_monitor_usage_CN.md) | 🟢 扩展读 | 5.4K | 📊 VINS 漂移监控和实时诊断 |
| [sensor_degradation_middleware_CN.md](./04_FEATURES/sensor_degradation_middleware_CN.md) | 🟢 扩展读 | 3.8K | 🔧 传感器噪声模拟、老化测试 |
| [coverage_memory_virtual_target_implementation_CN.md](./04_FEATURES/coverage_memory_virtual_target_implementation_CN.md) | 🟢 扩展读 | 4.0K | 🗺️ 搜索覆盖记忆和虚拟目标机制 |
| [BROADCAST_BSPLINE_AND_SWARM_CN.md](./04_FEATURES/BROADCAST_BSPLINE_AND_SWARM_CN.md) | 🟢 扩展读 | 4.2K | 📡 B样条广播和群体轨迹共享 |

**何时阅读**（这些都是可选功能）：
- 研究视觉目标识别：阅读 vlm_bridge_usage_CN.md
- 集成 VINS 并需要监控：阅读 vins_drift_monitor_usage_CN.md
- 做鲁棒性测试：阅读 sensor_degradation_middleware_CN.md
- 多机搜索任务：阅读 coverage_memory_virtual_target_implementation_CN.md
- 多机队形飞行：阅读 BROADCAST_BSPLINE_AND_SWARM_CN.md

---

### 第五部分：【扩展读 + 必须读组合】基准与研究（05_BENCHMARK）

关于性能测试、基准方案和参数调优的文档。

| 文档 | 层次 | 大小 | 用途 |
|------|------|------|------|
| [benchmark_documentation_quick_reference_CN.md](./05_BENCHMARK/benchmark_documentation_quick_reference_CN.md) | 🟡 可以读 | 9K | 📑 Benchmark 文档导航索引 |
| [benchmark_metrics_and_design_details_CN.md](./05_BENCHMARK/benchmark_metrics_and_design_details_CN.md) | 🟢 扩展读 | 32K | 📊 ⭐ 详细的性能指标定义和参数组合设计 |
| [swarm_research_campaign_usage_CN.md](./05_BENCHMARK/swarm_research_campaign_usage_CN.md) | 🟢 扩展读 | 16K | 🔬 如何运行研究 campaign、参数扫描和离线分析 |

**何时阅读**：
- 第一次接触 benchmark：阅读 benchmark_documentation_quick_reference_CN.md（【可以读】）
- 需要理解性能指标：阅读 benchmark_metrics_and_design_details_CN.md（【扩展读】）
- 计划做参数研究：阅读 swarm_research_campaign_usage_CN.md（【扩展读】）

---

### 第六部分：【排查读】存档与参考（06_ARCHIVED）

这些是日期标记的研究报告和过期文档，仅供**历史参考和故障排查**。

**何时查看**（【排查读】）：
- 遇到相同的历史问题
- 学习之前的解决方案和失败案例
- 追踪参数改动的原因

包含：
- 性能研究报告（Phase A/B/C/D）
- 多机集群调试记录
- Benchmark 框架演进历史
- Phase 3 健康报告

📌 **重要提示**：不建议新手或日常开发者阅读，除非遇到特定问题需要查阅历史记录。
- 模拟恶劣传感器环境：阅读 sensor_degradation_middleware_CN.md
- 开发多机协作搜索：阅读 coverage_memory_virtual_target_implementation_CN.md

---

### 第五部分：基准与研究（05_BENCHMARK）

关于性能测试、基准方案和参数调优的文档。

| 文档 | 大小 | 适用人群 | 用途 |
|------|------|--------|------|
| [benchmark_documentation_quick_reference_CN.md](./05_BENCHMARK/benchmark_documentation_quick_reference_CN.md) | 9K | 所有人 | 📑 Benchmark 文档导航索引 |
| [benchmark_metrics_and_design_details_CN.md](./05_BENCHMARK/benchmark_metrics_and_design_details_CN.md) | 32K | 性能优化 | 📊 ⭐ 详细的性能指标定义和参数组合设计 |
| [swarm_research_campaign_usage_CN.md](./05_BENCHMARK/swarm_research_campaign_usage_CN.md) | 16K | Research | 🔬 如何运行研究 campaign、参数扫描和离线分析 |

**何时阅读**：
- 不知道有什么 benchmark 资源：阅读 benchmark_documentation_quick_reference_CN.md
- 需要理解性能指标（tracking_error、replan_count 等）：⭐ 必读 benchmark_metrics_and_design_details_CN.md
- 计划进行参数研究和对标测试：阅读 swarm_research_campaign_usage_CN.md

---

### 第六部分：存档文档（06_ARCHIVED）

这些是日期标记的研究报告和过期的阶段报告，仅供参考。**不建议新手阅读**，除非研究特定的历史问题。

该目录下的文生是为了：
1. 保留历史记录和研究过程
2. 参考之前的解决方案和失败案例
3. 追踪系统演进过程

**内容包括**：
- Phase A/B/C/D 的研究报告（速度梯度、稳定性恢复、时间-光滑权衡、极限速度）
- Phase 3 健康报告（VINS 集成验证）
- 6 机集群的调试和分析报告
- 轨迹共享修复历史

**阅读建议**：
- 只有在面对"之前出现过的问题"时再查阅
- 不作为正式文档，仅供历史参考
- 内容可能包含过期的参数值和已修复的问题

---

### 第七部分：资源文件（images/）

系统架构、传感器配置等的图表和示意图。

| 文件 | 用途 |
|------|------|
| depth_noise_schematic.png | 深度传感器噪声模型示意 |
| real_depth_reconstruction_schematic.png | 深度重构原理示意 |
| real_depth_reconstruction_shape.png | 深度重构三维形状示例 |

---

## 🔑 关键文档使用场景

### 场景 1：我是新手，想从零开始学习这个项目

```
阅读顺序：
1. 👉 README_NEW.md （项目根目录）
2. 👉 HANDOVER_README.md （项目根目录）
3. 📖 01_FUNDAMENTALS/system_architecture_CN.md
4. 📖 01_FUNDAMENTALS/beginner_guide_CN.md
5. 📖 02_CONFIGURATION/uav_position_goal.md
预计时间：1-2 小时
```

### 场景 2：环境已配置，我想快速跑通第一个 demo

```
快速跑通：
1. cd /home/guanwen/XTDrone/ego-px4ctrl-new
2. source tools/source_phase1_env.sh
3. catkin_make -j4
4. ./test_swarm_smoke.sh
5. 参考 HANDOVER_README.md 第 3 节的基线命令

如果失败，再查看 HANDOVER_README.md 第 11 节（Debug 排查）
预计时间：30 分钟
```

### 场景 3：我需要调整控制参数（悬停高度、响应速度等）

```
关键文档：
👉 02_CONFIGURATION/phase1_px4ctrl_config_explanation_CN.md - 参数详解
👉 HANDOVER_README.md 第 12 节 - 配置文件详解
参考：HANDOVER_README.md 第 5 节（调参红线）
预计时间：30 分钟理解 + 实际调参时间
```

### 场景 4：我要从单机升级到多机系统

```
关键文档：
👉 03_MULTIUAV_PHASES/phase2_architecture_and_principles_CN.md - 多机架构
👉 02_CONFIGURATION/uav_position_goal.md - 多无人机位置配置
👉 03_MULTIUAV_PHASES/phase2_runtime_log_reading_guide_CN.md - 多机 debug
参考：HANDOVER_README.md 第 10-11 节
预计时间：1-2 小时学习 + 实际调试时间
```

### 场景 5：我在做性能优化和参数研究

```
关键文档：
👉 05_BENCHMARK/benchmark_metrics_and_design_details_CN.md - 性能指标解读
👉 05_BENCHMARK/swarm_research_campaign_usage_CN.md - 如何运行研究
参考：HANDOVER_README.md 第 9 节（脚本使用）
预计时间：2-4 小时学习工具链 + 实际研究时间
```

### 场景 6：遇到了一个 bug，不知道如何排查

```
关键资源：
👉 HANDOVER_README.md 第 11 节 - Debug 排查指南和常见问题
👉 03_MULTIUAV_PHASES/phase2_runtime_log_reading_guide_CN.md - 日志分析
👉 HANDOVER_README.md 第 13 节 - 日志分析和故障定位
预计时间：10-30 分钟（取决于问题复杂度）
```

---

## 📋 文档维护说明

### 谁应该更新这套文档

- **系统架构改动** → 更新 01_FUNDAMENTALS
- **参数改变** → 更新 02_CONFIGURATION 和相关章节
- **新功能上线** → 新增 04_FEATURES 章节
- **bug 修复** → 在 HANDOVER_README 的 Known Issues 记录
- **新研究成果** → 整理后放入 05_BENCHMARK

### 如何判断文档是否过期

文档中包含以下特征说明可能过期：

```
⚠️ 过期标记：
- 日期在文件名中（如 *_20260315_*）
- 内容说"据测试..."但没有给出测试条件
- 参数值与当前代码不一致
- 步骤描述与实际用户遇到的流程不符
```

---

## 🗂️ 目录结构说明

```
docs/
├── 00_DOCUMENTATION_INDEX.md              ← 你在这里
├── 01_FUNDAMENTALS/
│   ├── system_architecture_CN.md
│   ├── directory_structure_CN.md
│   ├── beginner_guide_CN.md
│   └── phase1_runtime_chain_design.md
├── 02_CONFIGURATION/
│   ├── phase1_px4ctrl_config_explanation_CN.md
│   ├── uav_position_goal.md
│   └── phase1_takeoff_tuning_CN.md
├── 03_MULTIUAV_PHASES/
│   ├── phase2_architecture_and_principles_CN.md
│   ├── phase2_runtime_log_reading_guide_CN.md
│   ├── phase4_dynamic_goal_execution_CN.md
│   └── PHASE3_VINS_CONTEXT_CN.md
├── 04_FEATURES/
│   ├── vlm_bridge_usage_CN.md
│   ├── vins_drift_monitor_usage_CN.md
│   ├── sensor_degradation_middleware_CN.md
│   ├── coverage_memory_virtual_target_implementation_CN.md
│   └── BROADCAST_BSPLINE_AND_SWARM_CN.md
├── 05_BENCHMARK/
│   ├── benchmark_documentation_quick_reference_CN.md
│   ├── benchmark_metrics_and_design_details_CN.md
│   └── swarm_research_campaign_usage_CN.md
├── 06_ARCHIVED/                          ← 存档，仅供参考
│   └── ... （历史研究报告）
└── images/
    ├── depth_noise_schematic.png
    ├── real_depth_reconstruction_schematic.png
    └── real_depth_reconstruction_shape.png
```

---

## 📞 获取帮助

如果你在这份索引中找不到答案：

1. **查看 HANDOVER_README.md** 的对应章节（更新频率最高）
2. **在相关 .md 文件中搜索** 关键词（Ctrl+F）
3. **查看代码注释** 和 launch 文件的参数说明
4. **在 issues 中搜索** 类似问题或创建新 issue

---

**最后修订**：2026-04-24  
**维护者**：ego-px4ctrl-new 开发团队
