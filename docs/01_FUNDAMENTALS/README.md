# 01_FUNDAMENTALS - 基础理论与系统架构

本目录包含系统设计的**基石文档**，涵盖整个项目的架构、设计思想和初期规划。

## 📚 目录内容

### [system_architecture_CN.md](system_architecture_CN.md) ⭐ 必读
**大小**：14K | **阅读时间**：20-25 分钟

这是整个系统的总体架构文档，涵盖：
- **5 层分层设计**：任务层 / 规划层 / 控制层 / 适配层 / 通信中间件 / 物理仿真
- **完整数据流图**：单机和多机场景
- **ROS Topic 和消息定义**：所有核心话题和格式说明
- **模块交互关系**：各组件之间如何通信

**何时阅读**：第一次接触项目，需要理解全局架构

**关键内容**：
```
系统管理层
├── EGO-Planner（规划）
├── px4ctrl（控制）
├── clean_uav_core（胶水）
└── MAVROS（中间件）
```

---

### [directory_structure_CN.md](directory_structure_CN.md)
**大小**：13K | **阅读时间**：15-20 分钟

代码仓库的目录树和各目录的用途说明。

**何时阅读**：
- 想找某个功能的源代码
- 需要了解项目文件组织
- 新增功能时选择合适的位置

**主要部分**：
- `src/` - ROS 包源码
- `tools/` - 工具和脚本
- `docs/` - 文档（就是这个目录！）
- `build/` 和 `devel/` - 编译产物（不要手工编辑）

---

### [beginner_guide_CN.md](beginner_guide_CN.md)
**大小**：11K | **阅读时间**：20-30 分钟

完整的新手入门指南，包括环境安装、编译和第一个 demo。

**何时阅读**：
- 第一次在本地部署项目
- Linux 或 ROS 新手
- 遇到编译错误需要排查环境

**涵盖内容**：
- Ubuntu 20.04 / ROS Noetic 安装
- Gazebo 和 PX4 SITL 配置
- 第一次编译（`catkin_make`）
- 运行第一个单机 demo

---

### [phase1_runtime_chain_design.md](phase1_runtime_chain_design.md)
**大小**：2.7K | **阅读时间**：5-10 分钟

Phase 1（单机）的设计目标和规划。

**何时阅读**：
- 理解 Phase 1 为什么这样设计
- 了解无 RC 模式（OFFBOARD）的工作原理
- 学习从规划层到控制层的完整链路

**核心概念**：
- 目标：单 UAV 的自主导航
- 链路：Gazebo → truth_odom → ego_planner → px4ctrl → MAVROS → PX4
- 不含：VINS、多机、遥控

---

## 🎯 快速导航

**我想理解整个系统** → [system_architecture_CN.md](system_architecture_CN.md)

**我想找代码在哪里** → [directory_structure_CN.md](directory_structure_CN.md)

**我是 Linux/ROS 新手** → [beginner_guide_CN.md](beginner_guide_CN.md)

**我想知道 Phase 1 的设计** → [phase1_runtime_chain_design.md](phase1_runtime_chain_design.md)

---

## 📖 阅读建议

### 最小化阅读（10-15 分钟快速上手）
1. system_architecture_CN.md 的"总体架构"和"数据流向图"两节
2. directory_structure_CN.md 的"主要目录说明"一节

### 标准阅读（30-45 分钟深入理解）
1. 完整阅读 system_architecture_CN.md
2. 完整阅读 directory_structure_CN.md
3. 浏览 phase1_runtime_chain_design.md

### 完整阅读（1-2 小时深度学习）
1. 完整阅读以上所有文档
2. 参考 system_architecture_CN.md 绘制数据流图
3. 对照代码和 launch 文件核实各个模块

---

## ⚠️ 常见问题

**Q：我是否需要理解所有 5 层架构？**  
A：不必。新手可以先关注中间 3 层（规划/控制/胶水），后续再学习通信和仿真层。

**Q：这些文档多久更新一次？**  
A：基础架构很稳定，通常 6-12 个月更新一次。如遇到架构改动会明确标注。

**Q：我想修改系统架构，从哪开始？**  
A：先在 system_architecture_CN.md 中清楚地描述你的改动，然后在代码中实现，最后更新这个文档。

---

**推荐接下来阅读**：[02_CONFIGURATION](../02_CONFIGURATION/) 或 [主文档索引](../00_DOCUMENTATION_INDEX.md)
