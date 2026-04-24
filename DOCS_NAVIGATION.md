# 📚 文档系统导航表

> **提示**：本项目拥有两个互补的文档系统。请根据您的语言偏好和学习场景选择合适的入口。

---

## 系统选择矩阵

| 维度 | **docs/** | **docs_new/** |
|------|-----------|----------------|
| **语言** | 英文为主，中文混合 | 纯中文 |
| **受众** | 国际团队、学术发表 | 中文开发者、国内团队 |
| **特点** | 按逻辑功能分类 7 大类 | 按学习路径分类 10 大类 |
| **推荐场景** | 需要系统参考、API 查询 | 快速入门、实战操作 |

---

## 📌 快速导航

### 🌟 首次使用？（10 分钟快速开始）

**必读** → 选择您的语言：
- 🇬🇧 **英文用户**：[docs/00_DOCUMENTATION_INDEX.md](docs/00_DOCUMENTATION_INDEX.md) 
  - 查看 【必须读】矩阵 → system_architecture_CN.md
  - 时间：15-30 分钟

- 🇨🇳 **中文用户**：[docs_new/README.md](docs_new/README.md#-快速开始)
  - 查看 【快速开始】→ 00_快速开始.md
  - 时间：10 分钟

---

### 🎯 按角色快速定位

#### 👨‍💻 开发者（需要理解系统架构）

**docs/ 推荐路径：**
1. 🔴 [system_architecture_CN.md](docs/01_FUNDAMENTALS/system_architecture_CN.md) - 系统 5 层分解（30 分钟）
2. 🔴 [directory_structure_CN.md](docs/01_FUNDAMENTALS/directory_structure_CN.md) - 目录组织（15 分钟）
3. 🟡 [phase1_px4ctrl_config_explanation_CN.md](docs/02_CONFIGURATION/phase1_px4ctrl_config_explanation_CN.md) - 参数详解（45 分钟）
4. 🟢 [phase2_architecture_and_principles_CN.md](docs/03_MULTIUAV_PHASES/phase2_architecture_and_principles_CN.md) - 多机系统（60 分钟）

**docs_new/ 推荐路径：**
1. [项目简介](docs_new/01_系统概述/01_项目简介.md) + [系统架构](docs_new/01_系统概述/02_系统架构.md) - 整体认知（30 分钟）
2. [环境搭建](docs_new/02_基础使用/01_环境搭建.md) + [常用命令](docs_new/02_基础使用/03_常用命令.md) - 实战操作（40 分钟）
3. [核心组件](docs_new/03_核心组件/) - 深入模块（按需阅读 1-2 小时）

#### 🚀 新手上手（只想快速跑通）

**首选 docs_new/**（中文优先）：
1. [00_快速开始](docs_new/00_快速开始.md) - 10 分钟仿真（必做）
2. [环境搭建](docs_new/02_基础使用/01_环境搭建.md) - 完整配置（30 分钟）
3. [快速入门](docs_new/02_基础使用/02_快速入门.md) - 常用操作（20 分钟）
4. 遇到问题 → [常见问题排查](docs_new/02_基础使用/04_常见问题排查.md)

#### 🔍 调试/排查（系统运行异常）

**首选排查文档** 🔵 【排查读】：
- **docs/**：
  - [phase2_runtime_log_reading_guide_CN.md](docs/03_MULTIUAV_PHASES/phase2_runtime_log_reading_guide_CN.md) - 日志阅读（30 分钟）
  - [06_ARCHIVED/](docs/06_ARCHIVED/README.md) - 历史问题库（按关键词搜索）

- **docs_new/**：
  - [常见问题排查](docs_new/02_基础使用/04_常见问题排查.md) - 快速排除（15 分钟）
  - [故障排除](docs_new/09_故障排除/) - 深度调参（按需）

#### 📊 性能优化/研究（深度二次开发）

**推荐 docs/ 扩展读文档** 🟢 【扩展读】：
- [04_FEATURES/README.md](docs/04_FEATURES/README.md) - 浏览所有高级功能
- [swarm_trajectory_sharing_fix_and_ab_CN.md](docs/04_FEATURES/swarm_trajectory_sharing_fix_and_ab_CN.md) - 轨迹共享机制
- [vins_drift_monitor_usage_CN.md](docs/04_FEATURES/vins_drift_monitor_usage_CN.md) - 视觉融合
- 研究论文逻辑 → 参考 [05_BENCHMARK/](docs/05_BENCHMARK/README.md)

---

## 📖 阅读层次说明

### 🔴 【必须读】 — 15-30 分钟
- **目标用户**：所有开发者、新手必读
- **内容**：系统架构、基础概念、快速启动
- **场景**：了解项目全貌、快速上手

### 🟡 【可以读】 — 1-2 小时
- **目标用户**：参与开发、想深入某模块的开发者
- **内容**：核心组件详解、配置参数说明、单阶段深度
- **场景**：需要修改代码、理解参数含义

### 🟢 【扩展读】 — 2-4 小时
- **目标用户**：高级开发者、研究者、性能优化人员
- **内容**：多机系统、集群算法、性能基准、新特性
- **场景**：二次开发、性能优化、研究工作

### 🔵 【排查读】 — 10-30 分钟
- **目标用户**：遇到问题的开发者、调试人员
- **内容**：问题排查、调参技巧、日志阅读
- **场景**：系统调试、问题排除、性能调优

---

## 📂 核心文件速查

### 项目必知（docs/）

| 文件 | 分类 | 说明 |
|------|------|------|
| [system_architecture_CN.md](docs/01_FUNDAMENTALS/system_architecture_CN.md) | 🔴 必读 | 系统 5 层架构：任务→规划→控制→仿真→物理 |
| [directory_structure_CN.md](docs/01_FUNDAMENTALS/directory_structure_CN.md) | 🔴 必读 | 仓库目录与文件组织逻辑 |
| [phase1_px4ctrl_config_explanation_CN.md](docs/02_CONFIGURATION/phase1_px4ctrl_config_explanation_CN.md) | 🟡 可读 | 114 个 px4ctrl 参数详解 |
| [phase1_runtime_chain_design.md](docs/01_FUNDAMENTALS/phase1_runtime_chain_design.md) | 🟡 可读 | Phase 1 启动链路设计 |
| [phase2_architecture_and_principles_CN.md](docs/03_MULTIUAV_PHASES/phase2_architecture_and_principles_CN.md) | 🟡 可读 | Phase 2 多机协同原理 |
| [phase2_runtime_log_reading_guide_CN.md](docs/03_MULTIUAV_PHASES/phase2_runtime_log_reading_guide_CN.md) | 🔵 排查 | 日志阅读与问题诊断 |

### 实战指南（docs_new/）

| 文件 | 分类 | 说明 |
|------|------|------|
| [00_快速开始](docs_new/00_快速开始.md) | 🔴 必读 | 10 分钟仿真体验 |
| [环境搭建](docs_new/02_基础使用/01_环境搭建.md) | 🔴 必读 | 完整环境配置指南 |
| [快速入门](docs_new/02_基础使用/02_快速入门.md) | 🔴 必读 | 常用操作速查 |
| [常见问题排查](docs_new/02_基础使用/04_常见问题排查.md) | 🔵 排查 | 快速故障排除 |
| [Phase 2: 多机系统](docs_new/04_开发阶段/02_Phase2_多机系统.md) | 🟡 可读 | 多机协同实战 |
| [控制器调参](docs_new/09_故障排除/02_控制器调参.md) | 🔵 排查 | PID 调参技巧 |

---

## 🔗 重要链接导航

### 根目录文档
- [HANDOVER_README.md](HANDOVER_README.md) - 交接手册（详细快速参考，1000+ 行）
- [CLAUDE.md](CLAUDE.md) - AI 开发指南
- [README.md](README.md) - 项目快速启动（本文件）

### 脚本与工具
- [tools/source_phase1_env.sh](tools/source_phase1_env.sh) - 环境初始化脚本
- [generate_swarm_config.py](generate_swarm_config.py) - 多机配置生成
- [test_swarm_smoke.sh](test_swarm_smoke.sh) - 冒烟测试脚本

### 源代码入口
- **核心包**：[src/clean_uav_core/](src/clean_uav_core/) - 启动编排胶水层
- **控制器**：[src/px4ctrl/](src/px4ctrl/) - px4ctrl 飞控
- **规划器**：[src/ego_planner/](src/ego_planner/) - EGO-Planner 规划

---

## 💡 使用建议

1. **第一次接手？** 
   - 中文优先 → [docs_new/00_快速开始](docs_new/00_快速开始.md)
   - 英文环境 → [docs/00_DOCUMENTATION_INDEX.md](docs/00_DOCUMENTATION_INDEX.md)

2. **想深入代码？**
   - 先读 [这个系统架构](docs/01_FUNDAMENTALS/system_architecture_CN.md)
   - 所有参数配置 → [这个文件](docs/02_CONFIGURATION/phase1_px4ctrl_config_explanation_CN.md)

3. **系统出问题？**
   - 快速查 → [常见问题排查](docs_new/02_基础使用/04_常见问题排查.md)
   - 深度诊断 → [日志阅读指南](docs/03_MULTIUAV_PHASES/phase2_runtime_log_reading_guide_CN.md)

4. **要做性能优化？**
   - 阅读 [docs/04_FEATURES/](docs/04_FEATURES/) 所有 🟢 文档
   - 参考 [基准测试框架](docs/05_BENCHMARK/)

---

## 📞 反馈与补充

如果您发现：
- 文档缺失或过时 → 在 `docs/` 或 `docs_new/` 对应文件夹创建 issue
- 链接失效 → 检查 [docs/06_ARCHIVED/](docs/06_ARCHIVED/) 是否已移动
- 需要新增主题 → 参考对应分类 README.md 的格式添加

---

**最后更新**：2026-04-24  
**状态**：两系统层次划分完整，所有文档已标记分层级别
