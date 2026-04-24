# 10 m/s 与 1.2 m/s 基准对照详解：EGO-Planner V2 的 CPU 负担与频率观测

**版本**：2026-04-08  
**适用范围**：Phase A / Phase D 基准实验补充说明  
**实验平台**：XTDrone + PX4 SITL + Gazebo + ROS Noetic + EGO-Planner V2 + px4ctrl  
**硬件平台**：Ryzen 9 7945HX，16 物理核 / 32 逻辑线程  
**对照样本**：1.2 m/s 基线与 10 m/s 高速复跑

本文是对 [phase_a_d_benchmark_teacher_report_CN.md](phase_a_d_benchmark_teacher_report_CN.md) 的详细补充。老师版报告侧重结论，这份文档侧重方法、指标解释、对照逻辑和部署判断依据。

---

## 1. 为什么要把 CPU 频率单独拿出来看

在这次 benchmark 里，单看 CPU 占用率还不够，因为“占用高”不等于“机器真的已经跑满”，而“占用低”也不代表算力富余。对 7945HX 这类高频、多核、动态加速很明显的 CPU 来说，真正影响规划时延和调度稳定性的，不只是平均占用，还包括：

1. 当前有多少核心在持续工作。
2. 这些核心实际跑在多少 MHz。
3. Boost 是否只是在短时冲高，还是能维持在较高频率。
4. GUI、Gazebo、MAVROS、记录进程一起竞争时，系统是否还能保持足够余量。

因此，本次新增了两类频率观测：

- `system_cpu_freq_current_*`：系统当前频率的统计值。
- `system_cpu_freq_peak_core_*`：每个采样时刻里，所有核心中“当前频率最高的那个核心”的统计值。

这两组值分别回答两个问题：

- “整机现在普遍跑到什么频率了？”
- “有没有某些核心在做明显的高频冲刺？”

---

## 2. 数据来源与统计方法

本次数据来自 benchmark session 的 summary 文件，分别是：

- [1.2 m/s 复跑 summary](../benchmark_artifacts/validation/stage_a_speed_1p2_run_02/auto_20260408_155659/auto_20260408_155659_summary.json)
- [10 m/s 复跑 summary](../benchmark_artifacts/validation/drone0_sync_run_07/auto_20260408_155900/auto_20260408_155900_summary.json)

这些 summary 由 [src/clean_uav_core/scripts/benchmark_manager.py](../src/clean_uav_core/scripts/benchmark_manager.py) 生成。它会同时采集：

- planner 栈相关进程的 CPU 占用。
- 主机总 CPU 占用。
- 每个采样周期的 CPU 当前频率。
- 每个采样周期里“峰值核心”的当前频率。

这里的 CPU 占用有一个重要含义：`planner_stack_cpu_percent_mean = 272.2586%` 这类值并不是错误，而是多线程进程在多核上累加后的结果。为了更直观地理解，可以把它换算成“逻辑核心等效数”：

$$
\text{core\_equivalent} = \frac{\text{cpu\_percent}}{100}
$$

所以 `272.26%` 约等于 `2.72` 个逻辑核心持续满载。

---

## 3. 指标怎么读

### 3.1 planner 栈 CPU 占用

这个指标主要反映 EGO-Planner V2 相关进程的计算压力。它更接近“算法本体的算力需求”，但不包含 Gazebo、RViz、其他 ROS 节点的全部开销。

### 3.2 system CPU 占用

这个指标反映整个主机的总压力，能看到仿真、控制、记录、通信、调度一起叠加后的结果。它更接近“这台机器是否还能继续稳定背负全栈任务”。

### 3.3 current frequency

表示系统采样时刻下的当前频率均值、P95 和峰值。它更像“当前整体工作档位”。

### 3.4 peak-core current frequency

表示在每个采样点里，所有核心中频率最高的那个核心的当前频率统计。它更像“有没有单核冲高、短时加速或局部热点”。

### 3.5 reported max frequency

这是系统/库报告的最大频率参考值，不应单独拿来判断真实算力上限。对这台机器来说，它显示为 2500 MHz，但实测 current frequency 和 peak-core current frequency 都明显高于这个值，所以这个字段更像“名义参考”，不是实际 turbo 上限。

---

## 4. 1.2 m/s 基线：系统非常宽松，但仍能看到 boost

### 4.1 任务结果

1.2 m/s 这组实验的结论是明确成功的：

- `stop_reason = all_goals_reached`
- 3 架无人机都完成了目标。
- 这一组可以作为“低速基线”来读。

### 4.2 关键数值

| 指标 | 数值 |
|------|------|
| planner 栈 CPU 平均占用 | 85.1264% |
| planner 栈 CPU P95 | 169.1% |
| planner 栈 CPU 峰值 | 277.3% |
| planner 栈折算核心数均值 | 0.8513 core |
| system CPU 平均占用 | 15.5306% |
| system CPU P95 | 32.1% |
| system CPU 峰值 | 98.8% |
| current frequency 均值 | 2183.6571 MHz |
| current frequency P95 | 2540.6313 MHz |
| current frequency 峰值 | 3755.5985 MHz |
| peak-core current frequency 均值 | 4214.2708 MHz |
| peak-core current frequency P95 | 4764.2480 MHz |
| peak-core current frequency 峰值 | 4895.5000 MHz |
| reported max frequency | 2500.0 MHz |

### 4.3 怎么理解这组数据

1. 平均只有 `0.85 core` 左右，说明 1.2 m/s 主要只是轻负载验证，不会逼近算力极限。
2. system CPU 平均只有 `15.5%`，说明全栈层面余量很大。
3. 但频率并不是“完全平平无奇”：peak-core current frequency 仍然能到 `4.9 GHz`，说明这台 7945HX 即便在低速时也会做明显 boost。
4. 这类现象很常见，说明“低速”并不等于“低频率”，只是高频冲刺出现得更零散、更短时。

### 4.4 低速基线的工程含义

1. 1.2 m/s 时，算法本体对 CPU 的压力很低。
2. 此时如果系统还是出现 jitter、启动不同步或发布链异常，那大概率不是“算力不够”，而是“启动链路、时序、语义或参数配置”问题。
3. 换句话说，1.2 m/s 这组数据更适合拿来验证“系统是否干净”，而不是拿来压测算力。

---

## 5. 10 m/s 复跑：planner 明显吃紧，系统开始接近调度上限

### 5.1 任务结果

10 m/s 复跑最终以 `emergency_stop` 结束，而不是 `all_goals_reached`。这意味着：

- 它不是一个“完整成功的任务结果”；
- 但它仍然是一个有效的“高速算力压力样本”；
- 对 CPU 占用和频率分析来说，数据依然有价值，因为它覆盖了高速阶段的真实工作负载。

### 5.2 关键数值

| 指标 | 数值 |
|------|------|
| planner 栈 CPU 平均占用 | 272.2586% |
| planner 栈 CPU P95 | 343.5% |
| planner 栈 CPU 峰值 | 395.4% |
| planner 栈折算核心数均值 | 2.7226 core |
| system CPU 平均占用 | 24.3009% |
| system CPU P95 | 43.6% |
| system CPU 峰值 | 99.5% |
| current frequency 均值 | 2332.0284 MHz |
| current frequency P95 | 2716.1172 MHz |
| current frequency 峰值 | 3473.7437 MHz |
| peak-core current frequency 均值 | 4018.9831 MHz |
| peak-core current frequency P95 | 4302.8360 MHz |
| peak-core current frequency 峰值 | 4352.5610 MHz |
| reported max frequency | 2500.0 MHz |

### 5.3 怎么理解这组数据

1. planner 栈平均占用 `2.72 core`，已经明显高于 1.2 m/s 的 `0.85 core`，约高出 `3.2` 倍。
2. system CPU 平均占用虽然只有 `24.3%`，看起来没有“满载”，但峰值已经到 `99.5%`，说明任务执行中确实会触发整机级别的资源紧张瞬间。
3. current frequency 均值比 1.2 m/s 略高，说明高速确实推高了持续工作频率。
4. 但 peak-core current frequency 反而比 1.2 m/s 的峰值略低，这是一个很有价值的现象：
   - 1.2 m/s 更像是少数核心短时冲得很高；
   - 10 m/s 更像是多个核心持续参与，整体进入多核工作态；
   - 这说明高速下的问题不只是“单核能不能冲上去”，而是“多核持续调度是否稳”。

### 5.4 高速样本的工程含义

1. 10 m/s 时，planner 不是“只占一点点 CPU”，而是已经形成持续的多核压力。
2. 系统总 CPU 接近打满，说明瓶颈更偏向“Gazebo + ROS 调度 + 规划/控制 + 日志记录”的组合竞争，而不是纯算法复杂度本身。
3. 这也是为什么高速场景更建议 headless 运行：GUI/RViz 的附加开销会进一步挤压本来就不宽裕的系统调度窗口。

---

## 6. 1.2 m/s 与 10 m/s 的直接对照

| 指标 | 1.2 m/s | 10 m/s | 变化趋势 |
|------|---------|--------|---------|
| planner 栈 CPU 平均占用 | 85.13% | 272.26% | +220.0% 左右 |
| planner 栈折算核心数均值 | 0.85 core | 2.72 core | 约 3.2 倍 |
| planner 栈 CPU P95 | 169.1% | 343.5% | 明显抬升 |
| planner 栈 CPU 峰值 | 277.3% | 395.4% | 继续抬升 |
| system CPU 平均占用 | 15.53% | 24.30% | 上升，但没线性爆炸 |
| system CPU P95 | 32.1% | 43.6% | 上升 |
| current frequency 均值 | 2183.66 MHz | 2332.03 MHz | 约 +148 MHz |
| current frequency P95 | 2540.63 MHz | 2716.12 MHz | 上升 |
| peak-core current frequency 峰值 | 4895.50 MHz | 4352.56 MHz | 10 m/s 更偏多核持续，不是单核极限 |

### 对照结论

1. **速度上去后，planner 的 CPU 压力增加非常明显**，这才是最直接的算力变化。
2. **system CPU 也会上升，但没有 planner 那么陡**，说明 EGO-Planner 是上升最敏感的部分。
3. **频率确实会提升，但不会一直线性冲高**，更像是“短时 boost + 持续多核工作”的混合态。
4. **不能只看 reported max frequency**。它固定在 2500 MHz，不足以代表实测可观测频率，更不能代表 7945HX 的真实瞬时算力。

---

## 7. 对 Ryzen 9 7945HX 的部署判断

### 7.1 这台机器到底够不够

结论是：**够用，而且有余量，但余量不是无限的**。

原因如下：

1. 7945HX 是 16C/32T，硬件线程数足够多。
2. 10 m/s 下 planner 平均只占 `2.72 core`，并没有把 CPU 独占到不可调度。
3. 真正危险的是整机峰值接近满载时，系统调度抖动、Gazebo 仿真、ROS 通信和日志写盘会互相抢资源。

### 7.2 推荐部署方式

#### 单机单无人机

- 7945HX 完全可以胜任。
- 建议尽量 headless。
- 如果只是做规划与控制，不要把 GUI/RViz 一起压在同一台主机上。

#### 单机三无人机、10 m/s 场景

- 可以跑，但建议把可视化移到观察机。
- 主机至少保留 `3~4` 个逻辑核心的稳定余量。
- 不要让后台还跑其他大任务，例如视频推理、大规模编译、浏览器重负载等。

#### 如果还要叠加 VINS / 深度相机 / 更多日志

- 建议把感知或可视化拆到第二台机器。
- 如果必须同机运行，优先保留 headless + 降低日志频率 + 关闭无关节点。

### 7.3 最重要的经验

对这类 benchmark 来说，**部署判断不能只看 CPU 百分比，也不能只看“频率有没有冲到 4 GHz”**。更有效的判断方法是把下面三件事一起看：

1. planner 栈持续占用。
2. system CPU 峰值和 P95。
3. current frequency / peak-core current frequency 的持续表现。

如果三者同时上升，说明系统已经进入高压工作区；如果只有频率上升但占用不高，那更像短时 boost；如果占用高但频率不高，则可能是调度受限或散热/功耗策略在限制性能。

---

## 8. 结果怎么解释给老师听

如果需要用一句更稳妥的话来概括，可以这样说：

> 在 Ryzen 9 7945HX 上，EGO-Planner V2 的 1.2 m/s 基线只消耗约 0.85 个逻辑核心，系统余量非常充足；提升到 10 m/s 后，planner 栈平均上升到约 2.72 个逻辑核心，整机峰值接近满载，说明高速场景的主要压力来自全栈调度竞争而不是单纯的算法本体算力不足。CPU 频率观测显示该平台确实会在负载下明显 boost，但真正影响部署稳定性的仍然是持续占用、峰值抖动和 GUI/仿真带来的额外竞争。

这句话的好处是：

- 不夸大结论；
- 能解释为什么 10 m/s 仍然“可跑”；
- 也能解释为什么 10 m/s 不是“随便一台机器都能稳定跑”的档位。

---

## 9. 复现实验的最小命令

如果需要重新跑一遍这两组对照，流程如下：

```bash
source tools/source_phase1_env.sh

# 1.2 m/s 基线
tools/benchmark_research/run_single_profile.sh \
  --version v2 \
  --launch-file benchmark_stage_a_v2_speed_1p2.launch \
  --output-dir benchmark_artifacts/validation/stage_a_speed_1p2_run_02

# 10 m/s 复跑
tools/benchmark_research/run_single_profile.sh \
  --version v2 \
  --launch-file benchmark_stage_d_rigid_clearance_balance.launch \
  --output-dir benchmark_artifacts/validation/drone0_sync_run_07
```

如果只想看汇总结果，直接打开对应的 `*_summary.json` 即可。

---

## 10. 和现有报告的关系

- [phase_a_d_benchmark_teacher_report_CN.md](phase_a_d_benchmark_teacher_report_CN.md) 适合直接汇报。
- 本文适合做“附录式说明”或“答辩时展开解释”。
- 如果只需要一页纸结论，就看老师版报告；如果要解释为什么要看频率、为什么 10 m/s 仍然能跑、为什么 7945HX 够用但不该开 GUI，就看这份详版。
