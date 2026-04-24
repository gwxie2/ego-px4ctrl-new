# 03_MULTIUAV_PHASES - 多机与高阶功能

本目录包含 **Phase 2/3/4 的多机协作、VINS 集成、动态目标** 等高阶功能的文档。

## 📚 目录内容

### [phase2_architecture_and_principles_CN.md](phase2_architecture_and_principles_CN.md) ⭐ 多机必读
**大小**：7.1K | **阅读时间**：20-25 分钟

多机系统的架构、数据流和同步机制。

**核心内容**：
- **多机数据流**：Gazebo → 多个无人机的 odom → 共享的规划和控制
- **命名空间隔离**：如何为每个 UAV 分配 `/iris_X/` / `/drone_X/` 命名空间
- **跨机通信**：trajectory sharing、goal synchronization、deadlock avoidance
- **启动顺序**：必须先启动仿真和 PX4，再启动规划和控制

**何时阅读**：
- 从单机升级到多机系统
- 多机之间通信有问题
- 某个 UAV 的命名空间路由错误

**关键概念**：
```
多机场景 = 单机 × N，但每个单机的命名空间需要隔离
Topic 示例：
  /iris_0/mavros/state
  /drone_0/odom
  /drone_0/position_cmd
  /drone_0/goal_set
```

---

### [phase2_runtime_log_reading_guide_CN.md](phase2_runtime_log_reading_guide_CN.md)
**大小**：7.6K | **阅读时间**：20-25 分钟

如何读懂和分析多机运行日志，快速定位故障。

**涵盖内容**：
- **日志输出格式**：各个节点的日志格式和关键字
- **故障诊断**：常见错误的日志特征和原因
- **性能指标**：从日志中提取关键指标（replan_count、tracking_error 等）
- **多机对比**：如何对比多个 UAV 的日志发现同步问题

**何时阅读**：
- 遇到多机 Debug 困难
- 需要分析飞行日志
- 怀疑某个 UAV 有问题但不确定原因

**常见日志签名**：
```
[ERROR] A* search failed → 规划器找不到可行方案
[WARN] replan triggered → 重规划过于频繁
[timeout] odom / imu / cmd → 某传感器或指令超时
```

---

### [PHASE3_VINS_CONTEXT_CN.md](PHASE3_VINS_CONTEXT_CN.md)
**大小**：6.4K | **阅读时间**：15-20 分钟

VINS（视觉惯导融合）在本系统中的集成方案。

**核心内容**：
- **VINS 的作用**：当 Gazebo ground truth 不可用时，用视觉+IMU 做自定位
- **融合架构**：如何将 VINS 的输出融合到 EGO-Planner
- **漂移补偿**：VINS 长期会有漂移，如何检测和补偿
- **fallback 机制**：VINS 失败时自动回到 truth odom

**何时阅读**：
- 集成外部视觉定位系统
- 从仿真环境迁移到真实环境（无 Gazebo ground truth）
- 研究视觉定位的可靠性和漂移特性

**关键流程**：
```
Camera + IMU 数据
  ↓
VINS Front-end（特征提取和匹配）
  ↓
VINS Back-end（Bundle Adjustment）
  ↓
odom_msg（带协方差的位置和速度）
  ↓
EGO-Planner（作为替代输入源）
```

---

### [phase4_dynamic_goal_execution_CN.md](phase4_dynamic_goal_execution_CN.md)
**大小**：3.4K | **阅读时间**：10 分钟

动态目标发布和执行的机制。

**用途**：
- 在飞行过程中动态改变目标（例如追踪移动目标）
- 多个目标的排序和优先级处理
- 与规划器的集成（目标更新后触发 replan）

**何时阅读**：
- 需要实现"跟踪移动目标"的功能
- 多个目标的调度和分配
- 目标变更对规划延迟的影响

---

### [CLEANROOM_PHASE2_MULTI_UAV_PLAN_CN.md](CLEANROOM_PHASE2_MULTI_UAV_PLAN_CN.md)
**大小**：3.9K | **阅读时间**：10 分钟

Phase 2 多机系统的设计计划（偏向历史记录）。

**内容**：
- Phase 2 的设计目标和阶段划分
- 从单机到双机、再到三机的递进
- 关键的系统改动和风险点

**何时阅读**：
- 理解 Phase 2 为什么这样划分
- 需要参考历史决策的理由

---

## 🚀 多机快速启动指南

### 最小化启动（3 机演示）

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh
catkin_make -j4

# 终端 1：启动仿真（等待 Gazebo 完全加载，10-15 秒）
roslaunch clean_uav_core phase2_px4_multi_sim.launch vehicle_num:=3

# 终端 2（仿真完全加载后）：启动规划和控制
roslaunch clean_uav_core phase2_dual_uav_stack.launch

# 终端 3（可选）：监控进度
./monitor_swarm.sh
```

### 带基准测试的启动

```bash
python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_g --case g11_horizon15 --num_runs 1
```

---

## 📊 多机调试检查清单

| 项目 | 检查方法 | 预期结果 |
|------|---------|----------|
| 仿真加载 | `rostopic list \| grep iris` | 应该看到 iris_0, iris_1, iris_2 |
| MAVROS 连接 | `rostopic echo /iris_0/mavros/state -n 1` | connected: true |
| Odom 稳定 | `rostopic hz /drone_0/odom` | 应该是 ~50 Hz |
| 命令流 | `rostopic hz /drone_0/position_cmd` | 应该是 ~50 Hz（规划完成后） |
| 多机同期 | 对比 `/drone_0/goal_reached` 和 `/drone_1/goal_reached` | 应该在 N 秒内同时到达 |

---

## 🎯 快速参考

**我要升级到多机系统** → [phase2_architecture_and_principles_CN.md](phase2_architecture_and_principles_CN.md)

**多机 debug 很困难** → [phase2_runtime_log_reading_guide_CN.md](phase2_runtime_log_reading_guide_CN.md)

**我要集成 VINS** → [PHASE3_VINS_CONTEXT_CN.md](PHASE3_VINS_CONTEXT_CN.md)

**我要实现动态目标跟踪** → [phase4_dynamic_goal_execution_CN.md](phase4_dynamic_goal_execution_CN.md)

---

**推荐接下来阅读**：[04_FEATURES](../04_FEATURES/) 或 [05_BENCHMARK](../05_BENCHMARK/)
