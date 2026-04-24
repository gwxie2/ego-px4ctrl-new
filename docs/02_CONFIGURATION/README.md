# 02_CONFIGURATION - 参数配置与调试指南

本目录包含**系统参数、配置文件和调试**相关的文档。

## 📚 目录内容

### [phase1_px4ctrl_config_explanation_CN.md](phase1_px4ctrl_config_explanation_CN.md) ⭐ 调参必读
**大小**：6.3K | **阅读时间**：20-25 分钟

PX4Ctrl 的完整参数手册和调参指南。

**核心参数包括**：
- `mass` - 无人机质量（影响推力模型）
- `hover_percentage` - 悬停油门百分比（**起飞生命线！**）
- `Kp0/1/2` 和 `Kv0/1/2` - 位置和速度环增益
- `msg_timeout.*` - 各传感器的超时时间

**何时阅读**：
- 无人机不能正确起飞
- 飞行轨迹有高频抖动或超调
- 需要调整飞行响应速度
- 准备从仿真迁移到实机

**关键警告**：
⚠️ `hover_percentage` 不是"可以随便试试"的参数——它直接决定无人机能否成功起飞。  
⚠️ 实机部署前，必须通过**低空悬停测试**来校准这个值。

---

### [uav_position_goal.md](uav_position_goal.md) 
**大小**：2.3K | **阅读时间**：5 分钟

Phase 1 无人机起始位置和目标位置的**配置格式说明**。

**用途**：
- 定义无人机的初始摆放位置（仿真中的起点）
- 定义无人机需要到达的目标位置
- 支持多无人机场景下的不同位置配置

**文件格式**（Markdown 格式）：
```markdown
## Phase1 Drone Definition (Three UAVs)

### Drone ID: iris_0
- Start Position: (-5, 0, 1)
- Goal Position: (5, 0, 1)

### Drone ID: iris_1
- Start Position: (0, -3, 1)
- Goal Position: (0, 3, 1)
```

**何时阅读**：
- 配置新的 Gazebo 场景
- 多无人机场景下设置不同起点和目标
- 运行 swarm_launch_generator.py 时需要理解输入格式

**关键说明**：
- 第一列：X 轴位置（前后，米)
- 第二列：Y 轴位置（左右，米）
- 第三列：Z 轴位置（高度，米）
- **原点** (0, 0, 0) 通常是 Gazebo 世界中心

---

### [phase1_takeoff_tuning_CN.md](phase1_takeoff_tuning_CN.md)
**大小**：3.6K | **阅读时间**：10-15 分钟

起飞过程的调参经验和最佳实践。

**涵盖内容**：
- 仿真中的起飞调参（hover_percentage 典型值 0.55-0.60）
- 实机起飞前的预检清单
- 电机解锁、推力测试的步骤
- 常见起飞失败的原因和对策

**何时阅读**：
- 无人机卡在地面无法起飞
- 起飞后瞬间过冲或下沉
- 准备从仿真迁移到真实硬件
- 需要进行电机响应性测试

**重点内容**：
- 悬停油门的标定方法（从保守值向上调整）
- 起飞阶段的安全限制（如起飞高度上限）
- 多机起飞的同期化问题

---

## 🔧 调参工作流

如果你需要調整参数,这是推荐的顺序:

### Step 1: 理解当前参数
```bash
# 查看当前的 PX4Ctrl 参数
cat src/clean_uav_core/config/phase1_px4ctrl_no_rc.yaml
```

### Step 2: 识别问题
根据现象确定需要调的参数：
- 起飞卡住？→ 调 `hover_percentage`
- 飞行抖动？→ 调 `Kp` / `Kv` 增益
- 响应缓慢？→ 增大增益或检查消息延迟
- 姿态不稳？→ 检查 IMU 校准或内环 PID

### Step 3: 修改参数
```bash
vim src/clean_uav_core/config/phase1_px4ctrl_no_rc.yaml
```

### Step 4: 编译和测试
```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh
catkin_make -j4
./test_swarm_smoke.sh  # 快速冒烟测试
```

### Step 5: 观察和迭代
- 观察起飞过程中的行为
- 查看控制器调试消息：`rostopic echo /drone_0/px4ctrl/debug`
- 逐步微调（每次改 5-10%）

### Step 6: 记录结果
在文档中记录：
- 改动了哪个参数
- 原值和新值
- 改动的效果（好或不好）

---

## 📊 参数对照表

| 参数 | 推荐值 | 范围 | 说明 |
|------|--------|------|------|
| `mass` | 1.5 kg | 1.0-2.0 | 影响重力补偿计算 |
| `hover_percentage` | 0.58 | 0.50-0.65 | 仿真通常 55-60%，实机需实测 |
| `Kp0/1` (x/y) | 10.0 | 5-20 | 水平面位置增益 |
| `Kp2` (z) | 20.0 | 10-40 | 竖直方向增益（通常较高） |
| `Kv0/1` (x/y) | 5.0 | 2-10 | 水平面速度增益 |
| `Kv2` (z) | 8.0 | 4-15 | 竖直方向速度增益 |
| `odom_timeout` | 1.0 s | 0.5-2.0 | 无 odom 消息时触发警报 |
| `imu_timeout` | 0.5 s | 0.2-1.0 | 无 IMU 消息时触发警报 |
| `cmd_timeout` | 1.0 s | 0.5-2.0 | 无位置指令时回到悬停 |

---

## ⚠️ 调参注意事项

| 禁止 | 推荐 |
|------|------|
| 一次改 50% 的参数值 | 每次改 5-10% |
| 同时改多个参数 | 一次只改一个参数 |
| 在高速（>5 m/s）下做激进调参 | 从低速（<2 m/s）开始 |
| 忽视 timeout 参数 | 根据实际硬件调整 timeout |
| 乱改 PID 系数而不记录 | 每次改动都写下"为什么改+改了啥+效果如何" |

---

## 🎯 快速参考

**我想调整起飞速度** → [phase1_takeoff_tuning_CN.md](phase1_takeoff_tuning_CN.md)

**我想调整飞行平稳度（PID 增益）** → [phase1_px4ctrl_config_explanation_CN.md](phase1_px4ctrl_config_explanation_CN.md)

**我要配置新的无人机位置** → [uav_position_goal.md](uav_position_goal.md)

**我要准备实机部署** → [phase1_takeoff_tuning_CN.md](phase1_takeoff_tuning_CN.md) + [phase1_px4ctrl_config_explanation_CN.md](phase1_px4ctrl_config_explanation_CN.md)

---

## 📋 调参检查清单

在修改任何参数之前，检查：

- [ ] 我理解这个参数的物理含义吗？
- [ ] 我知道改动它会影响哪些行为吗？
- [ ] 我的改动值在合理范围内吗？
- [ ] 我有备份了原参数值吗？
- [ ] 我准备好逐步迭代（而不是一次性大改）？

---

**推荐接下来阅读**：[03_MULTIUAV_PHASES](../03_MULTIUAV_PHASES/) 或 [主文档索引](../00_DOCUMENTATION_INDEX.md)
