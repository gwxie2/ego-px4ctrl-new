# ego-px4ctrl-new 文档完整索引

> **最后更新**：2026-04-24  
> **目的**：为所有开发者提供清晰的文档导航，避免重复阅读和误读过时文档。

---

## 🎯 快速开始（推荐阅读顺序）

如果你是项目新接手者或新开发者，建议按以下顺序阅读：

1. **项目总览**（5 分钟）
   - [系统架构设计](./01_FUNDAMENTALS/system_architecture_CN.md) — 了解 5 层分解的系统设计
   - [仓库目录结构](./01_FUNDAMENTALS/directory_structure_CN.md) — 了解代码组织

2. **快速上手**（10 分钟）
   - [新手入门指南](./01_FUNDAMENTALS/beginner_guide_CN.md) — 环保配、编译、第一次运行
   - **[HANDOVER_README.md](../HANDOVER_README.md)** — 交接手册，包含 10 分钟快速复现、命令清单、调参红线（必读！）

3. **深入学习**（30-60 分钟）
   - [Phase 1 运行链设计](./01_FUNDAMENTALS/phase1_runtime_chain_design.md) — 了解单机系统如何工作
   - [PX4Ctrl 参数详解](./02_CONFIGURATION/phase1_px4ctrl_config_explanation_CN.md) — 理解控制器参数
   - [无人机位置定义](./02_CONFIGURATION/uav_position_goal.md) — Phase 1 启动位置和目标配置

4. **准备多机实验**
   - [Phase 2 多机架构](./03_MULTIUAV_PHASES/phase2_architecture_and_principles_CN.md)
   - [运行日志分析](./03_MULTIUAV_PHASES/phase2_runtime_log_reading_guide_CN.md)

---

## 📚 完整文档分类

### 第一部分：基础理论（01_FUNDAMENTALS）

这是系统设计的基石，包含架构、设计思想和初期规划。

| 文档 | 大小 | 适用人群 | 用途 |
|------|------|--------|------|
| [system_architecture_CN.md](./01_FUNDAMENTALS/system_architecture_CN.md) | 14K | 所有人 | ⭐ 系统 5 层分解、数据流、消息格式定义 |
| [directory_structure_CN.md](./01_FUNDAMENTALS/directory_structure_CN.md) | 13K | 所有人 | ⭐ 仓库目录树、各目录作用说明 |
| [beginner_guide_CN.md](./01_FUNDAMENTALS/beginner_guide_CN.md) | 11K | 新手 | ROS 环境配置、本地编译、第一个 demo |
| [phase1_runtime_chain_design.md](./01_FUNDAMENTALS/phase1_runtime_chain_design.md) | 2.7K | 系统设计者 | Phase 1 设计目标、无RC模式说明 |

**何时阅读**：
- Linux/ROS 新手：必读全部
- 有经验开发者：快速浏览前两个，重点关注 phase1_runtime_chain_design

---

### 第二部分：配置与调参（02_CONFIGURATION）

涉及无人机的参数配置、起飞调试、位置初始化。

| 文档 | 大小 | 适用人群 | 用途 |
|------|------|--------|------|
| [phase1_px4ctrl_config_explanation_CN.md](./02_CONFIGURATION/phase1_px4ctrl_config_explanation_CN.md) | 6.3K | 控制调参 | 📋 px4ctrl 全参数手册（mass、hover_percentage、Kp/Kv等） |
| [uav_position_goal.md](./02_CONFIGURATION/uav_position_goal.md) | 2.3K | 所有人 | 📍 Phase 1 无人机起始位置和目标位置定义格式 |
| [phase1_takeoff_tuning_CN.md](./02_CONFIGURATION/phase1_takeoff_tuning_CN.md) | 3.6K | 实机部署 | 🎯 起飞调参经验和最佳实践 |

**何时阅读**：
- 需要调整控制参数：必读 phase1_px4ctrl_config_explanation_CN.md
- 配置新的无人机起始位置：阅读 uav_position_goal.md
- 准备实机部署：阅读 phase1_takeoff_tuning_CN.md

---

### 第三部分：多机与高阶功能（03_MULTIUAV_PHASES）

涵盖 Phase 2/3/4 的多机协作、VINS 集成、动态目标等高阶功能。

| 文档 | 大小 | 适用人群 | 用途 |
|------|------|--------|------|
| [phase2_architecture_and_principles_CN.md](./03_MULTIUAV_PHASES/phase2_architecture_and_principles_CN.md) | 7.1K | 多机开发 | 🤝 多机数据流、命名空间、同步机制 |
| [phase2_runtime_log_reading_guide_CN.md](./03_MULTIUAV_PHASES/phase2_runtime_log_reading_guide_CN.md) | 7.6K | Debug | 🔍 如何读懂日志、定位多机故障 |
| [phase4_dynamic_goal_execution_CN.md](./03_MULTIUAV_PHASES/phase4_dynamic_goal_execution_CN.md) | 3.4K | 高级用户 | 🚀 动态目标发布和执行机制 |
| [PHASE3_VINS_CONTEXT_CN.md](./03_MULTIUAV_PHASES/PHASE3_VINS_CONTEXT_CN.md) | 6.4K | VINS 集成者 | 📡 VINS 与 EGO-Planner 融合方案 |

**何时阅读**：
- 从单机升级到多机：必读 phase2_architecture_and_principles_CN.md
- 遇到多机 Debug 困难：阅读 phase2_runtime_log_reading_guide_CN.md
- 集成 VINS/外部定位：阅读 PHASE3_VINS_CONTEXT_CN.md
- 实现动态目标跟踪：阅读 phase4_dynamic_goal_execution_CN.md

---

### 第四部分：功能模块（04_FEATURES）

包含可选的高阶功能和实验性功能。

| 文档 | 大小 | 适用人群 | 用途 |
|------|------|--------|------|
| [vlm_bridge_usage_CN.md](./04_FEATURES/vlm_bridge_usage_CN.md) | 9.1K | 视觉研究 | 🎥 将摄像头像素投影到世界坐标、目标检测 |
| [vins_drift_monitor_usage_CN.md](./04_FEATURES/vins_drift_monitor_usage_CN.md) | 5.4K | VINS 用户 | 📊 VINS 漂移监控和实时诊断 |
| [sensor_degradation_middleware_CN.md](./04_FEATURES/sensor_degradation_middleware_CN.md) | 3.8K | 鲁棒性测试 | 🔧 传感器噪声模拟、老化测试 |
| [coverage_memory_virtual_target_implementation_CN.md](./04_FEATURES/coverage_memory_virtual_target_implementation_CN.md) | 4.0K | 多机搜索 | 🗺️ 搜索覆盖记忆和虚拟目标机制 |
| [BROADCAST_BSPLINE_AND_SWARM_CN.md](./04_FEATURES/BROADCAST_BSPLINE_AND_SWARM_CN.md) | 4.2K | 高级用户 | 📡 B样条广播和群体轨迹共享 |

**何时阅读**：
- 研究视觉目标识别：阅读 vlm_bridge_usage_CN.md
- 集成 VINS 并需要监控：阅读 vins_drift_monitor_usage_CN.md
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
