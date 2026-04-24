# 04_FEATURES - 高阶功能模块

本目录包含**可选的高阶功能**和**实验性功能**的使用指南。这些功能不是核心流程，但可以显著增强系统能力。

## 📚 目录内容

### [vlm_bridge_usage_CN.md](vlm_bridge_usage_CN.md)
**大小**：9.1K | **阅读时间**：20-25 分钟

**VLM (Vision-Language Model) Bridge** 的使用指南。

**功能**：
- 将摄像头图像上的**像素坐标**投影到**世界坐标系**
- 支持手动点击或自动检测（Grounding DINO）
- 将检测到的目标位置发布给规划器

**使用场景**：
- 研究视觉目标识别和定位
- 实现"看到一个目标，自主飞过去"的功能
- 多机协作搜索中的视觉辅助

**何时阅读**：
- 需要整合摄像头和目标检测
- 要做视觉引导的自主导航
- 研究多模态感知（视觉 + 规划）的融合

**核心概念**：
```
摄像头像素 (u, v) + 深度 d
  ↓
相机内参投影
  ↓
相机坐标系下的 3D 点
  ↓
机体旋转矩阵变换
  ↓
世界坐标系下的目标位置
  ↓
发布给规划器作为新目标
```

---

### [vins_drift_monitor_usage_CN.md](vins_drift_monitor_usage_CN.md)
**大小**：5.4K | **阅读时间**：12-15 分钟

**VINS 漂移监控**工具的使用说明。

**功能**：
- 监测 VINS 输出的实时漂移量
- 识别定位系统何时开始漂移（长期误差累积）
- 与 ground truth 对比，评估 VINS 可靠性
- 实时诊断和告警

**使用场景**：
- VINS 集成验证
- 评估外部定位系统的长期稳定性
- 触发 fallback 机制（VINS 漂移过大时回到 truth odom）

**何时阅读**：
- 集成 VINS 后需要验证其可靠性
- 要从仿真迁移到实机（用 VINS 代替 Gazebo ground truth）
- 研究定位系统的鲁棒性

**关键指标**：
```
漂移量 = √((odom_x - truth_x)² + (odom_y - truth_y)²)
危险阈值：> 0.5m（需要告警）
临界值：> 1.0m（触发 fallback）
```

---

### [sensor_degradation_middleware_CN.md](sensor_degradation_middleware_CN.md)
**大小**：3.8K | **阅读时间**：10 分钟

**传感器降级中间件** — 在仿真中注入传感器噪声和故障。

**功能**：
- 模拟真实传感器的噪声、延迟、漂移
- 预设的降级档位：clean / nominal / mild / extreme
- 支持 odometry、IMU、深度图像等多种传感器

**使用场景**：
- 测试系统对恶劣传感器环境的鲁棒性
- 模拟实机环境中的噪声和偏差
- 评估不同降级档位下的性能

**何时阅读**：
- 需要模拟真实传感器噪声
- 做鲁棒性测试（系统在坏传感器下能不能工作）
- 从仿真到实机的过渡验证

**预设档位**：
```
Clean:     无噪声（仿真理想情况）
Nominal:   轻度噪声（仿真中等情况）
Mild:      中度噪声（实机普通环境）
Extreme:   重度噪声（恶劣环境，压力测试）
```

---

### [coverage_memory_virtual_target_implementation_CN.md](coverage_memory_virtual_target_implementation_CN.md)
**大小**：4.0K | **阅读时间**：12 分钟

**多机搜索覆盖和虚拟目标机制**。

**功能**：
- 维护全局**搜索覆盖地图**，防止多机重复探索
- 记录失败的搜索路段，后续无人机避开这些区域
- 动态生成**虚拟目标**来填补覆盖缝隙

**使用场景**：
- 多机协作搜索任务（e.g. 无人机群搜救、侦察）
- 最大化覆盖效率，最小化重复工作
- 分布式决策，无需集中式 commanding

**何时阅读**：
- 要实现多机群体搜索功能
- 需要覆盖率和效率指标追踪
- 研究分布式协调算法

**核心机制**：
```
全局覆盖栅格 (2D)
├── 已覆盖区（绿色）
├── 未覆盖区（红色） ← 优先搜索
└── 禁行区（黑色） ← 之前失败的路段

当一个 UAV 失败时：
  该路段标记为禁行
  其他 UAV 通过虚拟目标避开
```

---

### [BROADCAST_BSPLINE_AND_SWARM_CN.md](BROADCAST_BSPLINE_AND_SWARM_CN.md)
**大小**：4.2K | **阅读时间**：10-12 分钟

**B 样条轨迹广播和多机轨迹共享**机制。

**功能**：
- 高效地在多个 UAV 之间**共享规划的轨迹**
- 使用 B 样条参数化，大幅减少通信量
- 支持快速的轨迹数据传输和重新参数化

**使用场景**：
- 多机协作紧密队形飞行
- 减少通信开销（轨迹参数 << 路径点序列）
- 轨迹高保真共享（B 样条精确表示）

**何时阅读**：
- 要实现多机紧密队形控制
- 关心通信的带宽效率
- 研究分布式轨迹规划的通信底层

**优势**：
```
传统方法：发送 N 个路径点，每个 3D 坐标（3 × N × 4 字节）
B 样条：只发送控制点（通常 10-20 个），数据量减少 90%+
```

---

## 🎯 功能选型指南

| 功能 | 你的需求 | 优先级 |
|------|---------|--------|
| VLM Bridge | 需要视觉目标检测和导航 | ⭐⭐⭐ 中等 |
| VINS Monitor | 从仿真迁移到实机 | ⭐⭐⭐ 中等 |
| Sensor Degradation | 做鲁棒性测试 | ⭐⭐ 低（可选） |
| Coverage Memory | 多机搜索任务 | ⭐⭐⭐ 中等（按需） |
| B-spline Broadcast | 多机队形飞行 | ⭐⭐ 低（按需） |

---

## 🚀 快速启用

### 启用 VLM Bridge
```bash
roslaunch clean_uav_core phase2_dual_uav_stack.launch enable_vlm_bridge:=true
```

### 启用传感器降级
```bash
roslaunch clean_uav_core phase2_px4_multi_sim.launch sensor_degradation_mode:=mild
```

### 启用搜索覆盖追踪
```bash
python3 src/clean_uav_core/scripts/swarm_mission_manager.py \
  --enable_coverage_tracking
```

---

## ⚠️ 功能依赖关系

某些功能之间有依赖关系，请注意：

```
VINS Monitor  依赖  VINS 集成（Phase 3）
              依赖  External odometry / VINS Front-end

Coverage Memory 依赖  多机系统（Phase 2 或以上）
                  依赖  目标检测和定位（可选 VLM）

B-spline Broadcast 依赖  多机系统（Phase 2 或以上）
```

---

## 📋 启用功能检查清单

在启用任何高阶功能前：

- [ ] 基础系统（Phase 1 或 Phase 2）能正常运行吗？
- [ ] 我理解这个功能的作用吗？
- [ ] 我有足够的计算资源吗？（某些功能会增加 CPU 负载）
- [ ] 依赖的模块都已安装和配置吗？
- [ ] 我准备好处理级联故障吗？（功能故障可能影响整体）

---

**推荐接下来阅读**：[05_BENCHMARK](../05_BENCHMARK/) 或 [主文档索引](../00_DOCUMENTATION_INDEX.md)
