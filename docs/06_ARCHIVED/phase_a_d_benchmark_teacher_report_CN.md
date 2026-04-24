# 多旋翼集群自主导航基准实验报告（Phase A–D）

**汇报日期**：2026-04-06  
**实验平台**：XTDrone + PX4 SITL + Gazebo + EGO-Planner-V2  
**集群规模**：3 架 iris 无人机  
**运行环境**：Ubuntu 20.04 / ROS Noetic / headless（无 GUI）  
**工作目录**：`/home/guanwen/XTDrone/ego-px4ctrl-new`

---

## 1. 研究背景与目标

本实验以 **EGO-Planner V2**（B 样条轨迹优化规划器）和 **px4ctrl**（级联 PID 飞控）为核心，在 PX4 SITL 仿真环境中构建了一套完整的三机协同自主导航测试流程。

实验由四个递进阶段构成：

| 阶段 | 英文标识 | 核心问题 |
|------|----------|----------|
| Phase A | Speed Ramping | 系统在不同飞行速度下的梯度压力响应 |
| Phase B | Stability Recovery | 低速复杂环境下的感知对齐与稳定性修复 |
| Phase C | Time–Smooth Trade-off | 时间效率与轨迹平滑之间的权衡分析 |
| Phase D | Extreme Speed 10 m/s | 极限速度场景下的系统边界探索与参数调优 |

所有实验均通过 `tools/benchmark_research/run_benchmark_campaign.py` 自动化运行，结果以 JSON/CSV 索引归档在 `benchmark_artifacts/research_profiles/`，并可通过 rosbag 回放复现。

---

## 2. 系统架构概述

```
┌──────────────────────────────────────────────────┐
│  规划层   EGO-Planner V2                         │
│           A* 初始路径 → B 样条优化 → 轨迹服务器   │
├──────────────────────────────────────────────────┤
│  控制层   px4ctrl                                │
│           级联 PID / 推力模型 / 无遥控模式        │
├──────────────────────────────────────────────────┤
│  胶水层   clean_uav_core                         │
│           启动编排 / 里程计桥 / 触发脚本          │
├──────────────────────────────────────────────────┤
│  中间件   MAVROS  (ROS ↔ MAVLink)               │
├──────────────────────────────────────────────────┤
│  仿真层   Gazebo + PX4 SITL                      │
└──────────────────────────────────────────────────┘
```

**关键数据流**：
```
Gazebo → /drone_N/odom → ego_planner_v2
                             ↓
                    /drone_N/position_cmd → px4ctrl → MAVROS → PX4
```

**rosbag 已录制话题**（每架无人机各一套）：
| 话题 | 类型 | 用途 |
|------|------|------|
| `/drone_N/odom` | `nav_msgs/Odometry` | 实际飞行轨迹（200 Hz） |
| `/drone_N/ego_planner_v2/grid_map/occupancy_inflate` | `sensor_msgs/PointCloud2` | 规划层感知到的膨胀障碍 |
| `/drone_N/position_cmd` | `quadrotor_msgs/PositionCommand` | 规划输出的位置指令 |
| `/iris_N/mavros/state` | `mavros_msgs/State` | 飞控连接与模式状态 |

> **注意**：深度相机原始图像（`/iris_N/realsense/...`）和规划轨迹可视化话题（`/drone_N/ego_planner_v2/optimal_list`）并未录入 rosbag，以控制文件体积。

---

## 3. RViz 回放配置

专用回放配置文件已创建：

```
src/clean_uav_core/rviz/benchmark_rosbag_replay.rviz
```

该配置基于 rosbag 实际录制话题，展示：
- **各无人机实际飞行轨迹**：Odometry 历史箭头（3 种颜色，保留 4000 帧）
- **规划层膨胀障碍地图**：PointCloud2（红/绿/蓝色系方块）
- **静态 TF**（可按需启用）

**快速启动回放**：

```bash
# 使用新 RViz 配置回放（脚本默认值已更新）
tools/benchmark_research/replay_benchmark_rosbag.sh \
  --input benchmark_artifacts/research_profiles/stage_d/d_rigid_clearance_balance/run_01

# 慢速回放（0.5×）
tools/benchmark_research/replay_benchmark_rosbag.sh \
  --input benchmark_artifacts/research_profiles/stage_a/v2_speed_5p0/run_01 \
  --rate 0.5

# 手动指定 RViz 配置
tools/benchmark_research/replay_benchmark_rosbag.sh \
  --input <run_dir> \
  --rviz-config src/clean_uav_core/rviz/benchmark_rosbag_replay.rviz

# 只回放不开 RViz（脚本方式分析）
tools/benchmark_research/replay_benchmark_rosbag.sh \
  --input <run_dir> \
  --no-rviz
```

> `position_cmd` 为自定义消息类型，标准 RViz 无法直接可视化，实际飞行轨迹通过 Odometry 历史箭头展示。

---

## 4. Phase A — 速度梯度压力测试

### 4.1 实验设置

- **目标**：评估 EGO-Planner V2 在 1.2 / 3.0 / 5.0 m/s 三个速度档位下的整体表现
- **场景**：相对稀疏的森林仿真环境
- **集群**：3 架无人机，同速，混合目标序列
- **关键配置**：`stage_a_speed_ramping_v2.yaml`，swarm_clearance 低速档 0.35 m

### 4.2 性能数据

| 速度 | 重规划成功率 | 最小安全间距 | 跟踪误差 P95 | 控制滞后 P95 | 抖动积分 | 违规次数 | 终止原因 |
|------|-------------|-------------|-------------|-------------|---------|---------|---------|
| 1.2 m/s | **0.9412** | **0.397 m** | 0.193 m | 84.7 ms | 4.7 | 0 | emergency_stop |
| 3.0 m/s | 0.8506 | 0.316 m | 0.541 m | 86.0 ms | 11.9 | 7 | emergency_stop |
| 5.0 m/s | **1.0000** | 0.319 m | 0.589 m | 1825 ms | 42.5 | 21 | goal_reached ✅ |

### 4.3 分析

1. **1.2 m/s**：系统容量充裕，零违规，最大安全间距。emergency_stop 由基准管理器的超时策略触发，并非真实碰撞。

2. **3.0 m/s**：追踪误差扩大 2.8×，间距略降，开始出现少量短时违规（7 次）。系统仍在可控范围内。

3. **5.0 m/s**：所有目标抵达成功，重规划成功率达到 100%，但**控制滞后骤升至 1825 ms**（20×），表明规划器与控制器之间的时序耦合开始成为性能瓶颈。抖动积分 42.5，轨迹质量下降明显。

> **Phase A 结论**：V2 规划器在 ≤3 m/s 下性能优秀；5 m/s 可达目标但伴随显著滞后累积。7 m/s 和 10 m/s 配置未通过 Stage A 阶段的 pipeline 验证（未在本批次运行）。

---

## 5. Phase B — 稳定性与感知恢复

### 5.1 实验设置

- **目标**：诊断并修复 EGO-Planner V2 在低速（1.2 m/s）复杂环境下的不稳定问题
- **背景**：初版配置（pathological_baseline）存在严重高频重规划死循环和小安全间距问题
- **速度**：固定 1.2 m/s，聚焦感知参数和避撞策略调优
- **关键配置路径**：`stage_b_stability_recovery_v2.yaml`，多个 profile

### 5.2 性能数据

| 方案 | 重规划成功率 | 最小安全间距 | 控制滞后 P95 | 抖动积分 | 违规次数 | 终止 |
|------|-------------|-------------|-------------|---------|---------|------|
| baseline_pathological（run1） | 0.9375 | 0.223 m | 85 ms | 5.0 | 10 | emergency_stop |
| baseline_pathological（run2） | 0.7053 | 0.068 m | 7339 ms | 48.1 | 34 | goal_reached |
| high_frequency_replan | 0.9905 | 0.079 m | 7549 ms | 47.3 | 85 | goal_reached |
| obstacle_guarded | 0.9285 | 0.150 m | 10997 ms | 65.1 | 171 | goal_reached |
| perception_aligned | 0.9808 | 0.103 m | 7479 ms | 47.6 | 33 | goal_reached |
| **perception_clearance_plus** | **0.9935** | **0.344 m** | 7368 ms | **46.2** | **14** | goal_reached ✅ |
| perception_dual_guard | 0.9358 | 0.251 m | 7941 ms | 34.1 | 60 | goal_reached |
| recovery_combo | 0.9735 | 0.256 m | 7672 ms | 46.7 | 155 | goal_reached |

### 5.3 分析

1. **基线 run1→run2 退化**：同一配置两次运行，run1 仅 run2 的一半时长便触发 emergency_stop，间距降至 0.068 m，说明 baseline 的 stochastic 因素很大。

2. **high_frequency_replan** 重规划成功率极高（0.9905），但间距 0.079 m 逼近临界，滞后 7549 ms。重规划频率提升带来的计算压力使整体时序更紧张。

3. **perception_clearance_plus**（最优方案）：
   - 感知范围扩展至 18×18 m，swarm_clearance 提升至 0.5 m
   - 成功率 0.9935，安全间距 0.344 m（全 B 阶段最大），违规仅 14 次
   - 这一配置成为后续高速阶段参数调优的参考基点

4. **obstacle_guarded** 虽然成功率尚可，但控制滞后高达 10997 ms，表明过度保守的局部地图更新导致规划频率受限。

> **Phase B 结论**：扩展感知范围 + 适度提高 clearance 是改善系统稳定性的最有效路径；单纯提高重规划频率不如调整感知参数收益高。最优方案 `perception_clearance_plus` 可作为低速标准配置。

---

## 6. Phase C — 时间–平滑权衡

### 6.1 实验设置

- **目标**：在 5 m/s 中速场景下量化时间效率优先 vs 轨迹平滑优先的权衡
- **速度**：5.0 m/s
- **配置路径**：`stage_c_time_smooth_tradeoff_v2.yaml`

### 6.2 性能数据

| 方案 | 重规划成功率 | 最小安全间距 | 跟踪误差 P95 | 控制滞后 P95 | 抖动积分 | 违规次数 |
|------|-------------|-------------|-------------|-------------|---------|---------|
| balanced_reference | 0.8410 | 0.111 m | 0.802 m | 2188 ms | 42.5 | 79 |
| smooth_priority | 0.8291 | 0.268 m | 0.589 m | 3299 ms | 45.3 | 28 |
| **time_priority** | 0.6975 | 0.063 m | **18.833 m** | 2893 ms | 67.9 | **421** |

### 6.3 分析

1. **time_priority**（时间优先）：跟踪误差爆炸至 18.833 m（P95），安全违规 421 次，接近临界。虽然成功抵达目标，但指令超前量过大，控制器根本无法追上，形成系统性滞后累积与安全风险。

2. **smooth_priority**（平滑优先）：间距改善至 0.268 m，违规仅 28 次，跟踪误差降低，但控制滞后升至 3299 ms（规划器输出变慢）。

3. **balanced_reference**：折中方案，抖动积分与 smooth_priority 相同（42.5 vs 45.3），但违规较多（79 vs 28），间距差（0.111 vs 0.268）。

> **Phase C 结论**：在 5 m/s 中速下，时间最优不可取（会导致跟踪崩溃）；平滑优先的安全性最好但规划延迟偏高。综合权衡建议选用 `smooth_priority` 作为中速标准配置。

---

## 7. Phase D — 极限速度 10 m/s 探索

### 7.1 挑战背景

将系统速度从 5 m/s 提升至 10 m/s 是一个质变（非量变）过程：

- 规划器时间窗口压缩 50%（从 5 m/s 时的预见距离减半）
- B 样条优化的动力学约束（$v_{\max}$、$a_{\max}$）几乎被完全激活
- 控制跟踪余量收窄，滞后从 ms 级变为 s 级将导致立即碰撞

**初期失败现象**：
- 参数原始设置下，`ego_planner_v2` 节点在极端约束下**段错误崩溃**（segfault）
- GUI/RViz 模式下规划器在 10 m/s 下无法产生 `position_cmd`（规划超时）
- 最初 3 次 Stage D 运行均以 `failed`（未达到 phase 完成条件）告终

### 7.2 为何选择 Headless（无 GUI）模式

| 问题 | GUI 模式 | Headless 模式 |
|------|---------|--------------|
| 规划器 CPU 余量 | Rviz 抢占约 15–25% CPU，规划循环抢占不到实时时隙 | CPU 完全给规划和控制 |
| `position_cmd` 产出 | 10 m/s 下频繁未产出（benchmark 超时触发 `failed`） | 正常产出，每次均通过 runtime-health 检查 |
| 运行确定性 | 系统调度抖动大，复现性差 | 行为可复现，利于参数对比 |
| Gazebo 渲染 | GUI 场景渲染与物理引擎共用线程，规划迟滞叠加 | 物理引擎可以更均匀地推进仿真时间 |

**结论**：对于 10 m/s 极限速度测试，**headless 是唯一经验证的稳定执行路径**；所有 Stage D 有效数据均来自 headless 运行。

### 7.3 参数迭代历程

Stage D 的调优以 `d_rigid_control`（保守基线）为起点，沿两条探索分支展开：

```
d_extreme_baseline (fail ×2)
        │
d_rigid_control（保守基线，成功）
        ├── lambda 分支 ─────────────────────────────────┐
        │    d_rigid_smooth_guard                        │
        │    d_rigid_lambda_light（成功率↑ 但滞后爆炸）  │
        │    d_lambda_kv_trim                            │
        │    d_lambda_kv_mid（明确排除）                 │
        │                                                │
        └── clearance/inflation 分支 ────────────────────┘
             d_safety_margin_biased（违规过多）
             d_rigid_clearance_plus（emergency_stop）
             d_rigid_clearance_light
             d_rigid_clearance_micro
             d_rigid_clearance_shallow
             d_rigid_clearance_balance（★ 最优）
```

**kv 分支（已收掉）**

- `d_lambda_kv_trim`（kv=2.6）：降低滞后但拉低成功率
- `d_lambda_kv_mid`（kv=2.7）：所有指标均劣于 kv_trim → **排除**

**clearance/inflation 分支（当前主线）**

小步长精调（clearance: 0.78→0.775 m，inflation: 0.29→0.2875 m）发现了显著的突破点。

### 7.4 完整 Stage D 结果对照表

| 方案 | 终止状态 | 成功率 | 最小间距 | 跟踪 P95 | 滞后 P95 | 抖动 | 违规 | 说明 |
|------|---------|--------|---------|---------|---------|------|------|------|
| extreme_baseline | ❌ failed | — | — | — | — | — | — | segfault / headless 前 |
| extreme_combo | ❌ failed | — | — | — | — | — | — | 规划超时 |
| predictive_long_range | ❌ failed | — | — | — | — | — | — | 规划超时 |
| extreme_baseline（relaxed） | ✅ | 0.7163 | 0.082 m | 10.39 m | 7321 ms | 144.5 | 335 | 首次通过，safety 差 |
| d_rigid_control | ✅ | 0.7846 | 0.130 m | 9.922 m | 6690 ms | 129.0 | 347 | 保守基线 |
| d_rigid_smooth_guard | ✅ | 0.7823 | 0.065 m | 10.78 m | 7798 ms | 111.5 | 346 | smooth守护无明显收益 |
| d_safety_margin_biased | ✅ | 0.9218 | 0.070 m | 28.10 m | 5201 ms | 113.8 | **687** | 跟踪崩溃，违规极多 |
| d_rigid_clearance_plus | ⚠️ emergency_stop | 0.6976 | 0.141 m | 1.208 m | 6639 ms | 105.2 | 211 | clearance 过大触发紧停 |
| d_rigid_lambda_light | ✅ | 0.9218 | 0.136 m | 11.23 m | 10573 ms | 170.8 | 395 | 成功率↑ 但滞后爆炸 |
| d_rigid_clearance_light | ✅ | 0.7630 | 0.085 m | 9.722 m | 6954 ms | 127.4 | 291 | 中间步骤 |
| d_lambda_kv_trim | ✅ | 0.8511 | 0.136 m | 10.55 m | 9276 ms | 116.5 | 403 | kv 分支参考点 |
| d_lambda_kv_mid | ✅ | 0.8299 | 0.099 m | 10.28 m | 12216 ms | 211.4 | 447 | ❌ 排除（kv 分支最差） |
| d_rigid_clearance_micro | ✅ | 0.8869 | 0.091 m | 11.33 m | 10405 ms | 184.9 | 219 | 步骤节点，间距偏低 |
| d_rigid_clearance_shallow | ✅ | 0.9091 | 0.269 m | 0.774 m | **7053 ms** | **131.7** | 161 | 低滞后候选 |
| **d_rigid_clearance_balance** | ✅ | **0.9206** | **0.289 m** | **0.693 m** | 7659 ms | 135.3 | **153** | ★ **当前最优** |

### 7.5 最优配置分析：`d_rigid_clearance_balance`

**关键参数**：

```yaml
benchmark_default_vmax: 10.0
px4ctrl_kv: 2.8            # 速度环增益
swarm_clearance: 0.775     # 集群避碰间距（米）
grid_map_obstacles_inflation: 0.2875  # 障碍膨胀半径（米）
# lambda_smooth 保持默认（未限制平滑权重）
```

**为何此配置有效**：

1. **swarm_clearance=0.775 m**：比 clearance_plus（0.8 m）略小，避免了"过度保守导致规划空间不足"而触发 emergency_stop 的问题；比 clearance_light（0.78 m）略大，给安全间距留出余量。

2. **inflation=0.2875 m**：轻微缩小障碍膨胀半径（相比常规值），在保持安全的前提下，为规划器释放了更多可通过的走廊宽度，显著降低了规划失败率（optimizer_failed 从 80 次降至 14 次）。

3. **跟踪误差从 9.9 m → 0.69 m（P95）**：这是最关键的突破。说明规划指令与实际飞行路径趋于一致，系统进入真正的实时跟踪状态。

**与基线对比**（`d_rigid_control`）：

| 指标 | d_rigid_control | d_rigid_clearance_balance | 改善幅度 |
|------|----------------|--------------------------|---------|
| 重规划成功率 | 0.7846 | **0.9206** | +17.3% |
| 最小安全间距 | 0.130 m | **0.289 m** | +122% |
| 跟踪误差 P95 | 9.922 m | **0.693 m** | -93% |
| 安全违规次数 | 347 | **153** | -56% |
| 规划优化器失败 | 80 | **14** | -83% |

### 7.6 双候选对照

| 指标 | d_rigid_clearance_balance | d_rigid_clearance_shallow |
|------|--------------------------|--------------------------|
| 重规划成功率 | **0.9206** | 0.9091 |
| 最小安全间距 | **0.289 m** | 0.269 m |
| 跟踪误差 P95 | **0.693 m** | 0.774 m |
| 控制滞后 P95 | 7659 ms | **7053 ms** |
| 抖动积分 | 135.3 | **131.7** |
| 安全违规数 | **153** | 161 |

**选择建议**：
- 首选 `d_rigid_clearance_balance`（安全性和成功率综合更优）
- 若系统对控制实时性有更高要求，`d_rigid_clearance_shallow` 滞后低 8.6%，可作备选

### 7.7 可重现性说明

目前最优配置 `d_rigid_clearance_balance` 已完成 run_01 验证，关键指标均超过预设阈值（成功率>0.9，违规<200，间距>0.25 m）。建议进行 run_02 复现验证以确认统计稳定性，再正式定为 10 m/s 主线配置。

### 7.8 10 m/s 算力负担与实机部署建议

在当前实验机 `Ryzen 9 7945HX`（16C/32T）上，`run_05` 已补充采样到 planner 栈的 CPU 占用，结果见 [auto_20260408_153538_summary.json](../benchmark_artifacts/validation/drone0_sync_run_05/auto_20260408_153538/auto_20260408_153538_summary.json)。

| 指标 | 数值 |
|------|------|
| planner 栈 CPU 平均占用 | 295.55% |
| planner 栈 CPU P95 | 379.10% |
| planner 栈 CPU 峰值 | 433.40% |
| planner 栈折算核心数均值 | 2.96 core |
| planner 栈折算核心数 P95 | 3.79 core |
| planner 栈折算核心数峰值 | 4.33 core |
| 主机总 CPU 平均占用 | 25.74% |
| 主机总 CPU P95 | 53.50% |
| 主机总 CPU 峰值 | 100.00% |

**解读**：

1. ego-planner 相关进程在 10 m/s 下的平均算力需求并不夸张，折算后约 3 个逻辑核心，P95 也在 4 个核心以内。对 7945HX 这类 16C/32T CPU 来说，planner 本身只占总算力的约 9%~14%，不构成单独的瓶颈。
2. 但系统总 CPU 在峰值时已经打满，说明真正的压力来自“Gazebo + ROS 调度 + 规划/控制 + 记录/通信”的整体竞争，而不是单纯的算法本体。
3. 这也解释了为什么 10 m/s 更依赖 headless 模式：一旦引入 GUI/RViz，CPU 余量会被进一步吃掉，planner 的启动抖动和重规划抖动更容易被放大。

**实机部署建议**：

- 单 UAV 实机：建议至少 8C/16T 的高频 CPU，32 GB 内存，SSD；如果只跑规划 + 控制 + 基础里程计，这个档位通常够用。
- 多 UAV 或 10 m/s 高速实机：建议 12C/24T 起步，优先 16C/32T 级别；对多机同时规划、VINS、日志记录、通信冗余更稳妥。
- 上机运行时尽量不要开 GUI/RViz，把可视化放到另一台观察机；主机只保留飞控、规划、感知和必要日志。
- 如果后续要把深度视觉/VINS/多机协同全部压在同一台伴随计算机上，建议按“至少预留 3~4 个逻辑核心的持续空闲”来配机。

**结论**：当前 10 m/s 主线配置对 `Ryzen 9 7945HX` 是“有余量可跑”的，不是纯算力卡死；后续优化重点仍应放在算法路径、初始化与系统调度上，而不是盲目追更高 CPU 频率。

### 7.9 CPU 频率补充观测与 1.2 m/s 对照

为了回应“CPU 运行频率是否也应纳入算力分析”，我又补跑了两次带频率采样的 session：一次是 10 m/s 主线配置的复跑，另一只是 1.2 m/s 专用基线复跑。

- 10 m/s 频率补采 summary： [auto_20260408_155900_summary.json](../benchmark_artifacts/validation/drone0_sync_run_07/auto_20260408_155900/auto_20260408_155900_summary.json)
- 1.2 m/s 频率补采 summary： [auto_20260408_155659_summary.json](../benchmark_artifacts/validation/stage_a_speed_1p2_run_02/auto_20260408_155659/auto_20260408_155659_summary.json)

| 指标 | 10 m/s | 1.2 m/s |
|------|------|------|
| planner 栈 CPU 平均占用 | 272.26% | 85.13% |
| planner 栈 CPU P95 | 343.50% | 169.10% |
| planner 栈 CPU 峰值 | 395.40% | 277.30% |
| planner 栈折算核心数均值 | 2.72 core | 0.85 core |
| system CPU 平均占用 | 24.30% | 15.53% |
| system CPU P95 | 43.60% | 32.10% |
| current frequency 均值 | 2332.03 MHz | 2183.66 MHz |
| current frequency P95 | 2716.12 MHz | 2540.63 MHz |
| current frequency 峰值 | 3473.74 MHz | 3755.60 MHz |
| peak-core current frequency 均值 | 4018.98 MHz | 4214.27 MHz |
| peak-core current frequency P95 | 4302.84 MHz | 4764.25 MHz |
| peak-core current frequency 峰值 | 4352.56 MHz | 4895.50 MHz |
| reported max frequency | 2500.0 MHz | 2500.0 MHz |

**解读**：

1. 1.2 m/s 下 planner 栈平均只占约 0.85 个逻辑核心，和 10 m/s 的 2.72 个核心相比，算力压力约低 3.2 倍，说明低速段对 CPU 余量非常宽松。
2. 两个速度档位下都能观察到明显的频率 boost，peak-core current frequency 都超过了 4.0 GHz，说明这台 7945HX 在实测里确实进入了高频加速状态。
3. `reported max frequency` 只显示 2500 MHz，但 per-core current frequency 已经能跑到 4.35-4.90 GHz，说明单看 nominal max 值会低估这台机器的实际瞬时算力；对这类 benchmark，必须同时看 utilization 和 frequency。
4. 10 m/s 的 current frequency 均值高于 1.2 m/s，说明更高速度确实会推高持续算力需求，但 boost 上限并没有同步线性抬升，这意味着系统更接近“调度和同步瓶颈”，而不是纯 CPU 主频瓶颈。

**结论**：频率可以观测，而且应该纳入部署判断。对当前 7945HX 来说，10 m/s 主线配置仍然可跑，但真正需要关注的是“持续占用 + boost 余量 + 多机调度抖动”这三个量一起看，而不是只看一个 CPU 百分比。

---

## 8. 综合跨阶段对比

### 8.1 系统能力边界图谱

| 阶段 | 速度 | 规划成功率 | 安全间距 | 跟踪精度 P95 | 系统状态 |
|------|------|-----------|---------|------------|---------|
| A – 1.2 m/s | 1.2 m/s | 0.9412 | 0.397 m | 0.193 m | 充裕 |
| A – 3.0 m/s | 3.0 m/s | 0.8506 | 0.316 m | 0.541 m | 良好 |
| A – 5.0 m/s | 5.0 m/s | 1.0000 | 0.319 m | 0.589 m | 达到，但滞后↑ |
| B – perception_clearance_plus | 1.2 m/s | 0.9935 | 0.344 m | 0.168 m | 最稳定低速配置 |
| C – smooth_priority | 5.0 m/s | 0.8291 | 0.268 m | 0.589 m | 最优中速配置 |
| **D – clearance_balance** | **10.0 m/s** | **0.9206** | **0.289 m** | **0.693 m** | **首个实用极速配置** |

### 8.2 关键参数敏感性总结

| 参数 | 作用方向 | Stage A/B 最优值 | Stage D 最优值 | 备注 |
|------|---------|-----------------|---------------|------|
| `benchmark_default_vmax` | 速度上限 | 1.2–5.0 m/s | 10.0 m/s | 关键驱动变量 |
| `swarm_clearance` | 集群避碰半径 | 0.35–0.5 m | 0.775 m | 高速下需要更大缓冲 |
| `grid_map_obstacles_inflation` | 障碍膨胀 | 0.3–0.48 m | 0.2875 m | 高速反而要缩小以保留走廊 |
| `px4ctrl_kv` | 速度环增益 | （Stage A/B 默认） | 2.8 | 太高导致振荡，太低跟踪差 |
| `planner_lambda_smooth` | 平滑权重 | — | 默认（不限制） | 过强会爆炸速度跟踪误差 |

### 8.3 安全违规来源分析

在 10 m/s 阶段，安全违规的三大来源：
1. **帧间跟踪超调**：控制器以高增益追赶被遗漏的位置命令，导致瞬时越界
2. **规划器重优化失败后的惰性轨迹**：optimizer_failed 期间系统沿最近一条有效轨迹飞行，位置偏差累积
3. **集群互相遮挡感知**：多机近距飞行时，一架无人机的传感器锥被同伴遮挡，导致局部地图更新不及时

---

## 9. 关于扩展至 10 m/s 的核心结论

### 9.1 可行性结论

**EGO-Planner V2 与 px4ctrl 的组合在 10 m/s 下具备可用性**，但前提是：
1. 必须在 headless 模式下运行（GUI 渲染引入的 CPU 竞争导致规划超时）
2. 必须对 clearance 和 inflation 参数进行精细调优（±0.005 m 量级的步长敏感）
3. 不能盲目提高速度增益（kv 分支已证明简单提高 kv 无效）

### 9.2 与理想状态的差距

| 期望 | 当前最优（d_rigid_clearance_balance） | 差距 |
|------|---------------------------------------|------|
| 跟踪误差 P95 < 0.5 m | 0.693 m | 还需约 28% 改善 |
| 控制滞后 P95 < 5000 ms | 7659 ms | 还需约 34% 降低 |
| 安全违规 < 100 次 | 153 次 | 还需约 35% 减少 |
| 最小间距 > 0.3 m | 0.289 m | 接近，差 3.7% |
| 重规划成功率 > 0.95 | 0.9206 | 需再提升约 3.2% |

### 9.3 建议的下一步工作

| 优先级 | 动作 | 预期收益 |
|--------|------|---------|
| P0 | 对 `d_rigid_clearance_balance` 进行 run_02 复现验证 | 确认统计稳定性 |
| P1 | 在 clearance 0.7725–0.775、inflation 0.285–0.2875 之间做 1–2 个中间点扫描 | 寻找更优的 Pareto 点 |
| P2 | 运行 Pareto/风险/敏感性分析工具 | 全局视角量化最优区域 |
| P3 | 在 10 m/s 主线稳定后，尝试 GUI 模式验证 | 确认可视化兼容性 |
| P4 | 考虑引入前馈补偿降低控制滞后 | 理论上可将滞后降至 3000 ms 内 |

---

## 10. 实验归档结构

```
benchmark_artifacts/research_profiles/
├── stage_a/
│   ├── v2_speed_1p2/run_01/    # rosbag + summary + CSV
│   ├── v2_speed_3p0/run_01/
│   └── v2_speed_5p0/run_01/
├── stage_b/
│   ├── baseline_pathological/run_{01,02}/
│   ├── perception_clearance_plus/run_01/   # ★ B阶段最优
│   └── ...（6 个子目录）
├── stage_c/
│   ├── balanced_reference/run_01/
│   ├── smooth_priority/run_01/             # ★ C阶段最优
│   └── time_priority/run_01/
├── stage_d/
│   ├── d_rigid_clearance_balance/run_01/   # ★★ D阶段最优（当前主线）
│   ├── d_rigid_clearance_shallow/run_01/    # 低滞后备选
│   └── ...（14 个子目录）
├── campaign_index.json     # 全部运行的扁平索引
└── campaign_index.csv      # 同上（CSV 格式，便于导入 Excel）
```

### 归档命令

```bash
# 刷新索引
source tools/source_phase1_env.sh
python3 tools/benchmark_research/collect_campaign_index.py

# 分析工具
python3 tools/pareto_frontier_analysis.py
python3 tools/risk_statistics_analysis.py
python3 tools/parameter_sensitivity_analysis.py
```

---

## 11. 参考文档

| 文档 | 位置 | 说明 |
|------|------|------|
| Stage D 入场检查点 | `benchmark_artifacts/research_profiles/stage_d_entry_checkpoint_20260406_CN.md` | 完整迭代过程记录 |
| Stage D Headless 简报 | `benchmark_artifacts/research_profiles/stage_d_headless_brief_20260406_CN.md` | 老师汇报简洁版 |
| Rosbag 回放文档 | `docs/swarm_benchmark_rosbag_offline_replay_and_campaign_update_CN.md` | 脚本使用详解 |
| RViz 回放配置 | `src/clean_uav_core/rviz/benchmark_rosbag_replay.rviz` | 专用回放配置 |
| 系统架构 | `docs/system_architecture_CN.md` | 整体架构说明 |
| px4ctrl 配置说明 | `docs/phase1_px4ctrl_config_explanation_CN.md` | 控制参数解读 |

---

*本文档由基准实验自动化工具链生成，数据来源：`benchmark_artifacts/research_profiles/campaign_index.json`（2026-04-06 版本）*
