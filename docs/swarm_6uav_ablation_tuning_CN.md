# 6机消融实验调参指南

## 目标

这份文档的目标不是继续“盲调一个更大的权重”，而是把 6 机场景里的行为拆成可验证的因果链，方便做消融实验：

1. 先确认告警链路是否真的被隔离。
2. 再确认群体避碰是否来自 clearance、weight、symmetry gain 的哪一项。
3. 最后确认“更快到达”和“更少锁死”分别由哪些参数推动。

当前仓库里与 6 机场景直接相关的参数，主要分布在以下位置：

- [src/clean_uav_core/config/swarm_config.yaml](../src/clean_uav_core/config/swarm_config.yaml)
- [src/clean_uav_core/scripts/swarm_launch_generator_yaml.py](../src/clean_uav_core/scripts/swarm_launch_generator_yaml.py)
- [src/clean_uav_core/launch/swarm_uav_runtime_instance_v2.launch](../src/clean_uav_core/launch/swarm_uav_runtime_instance_v2.launch)
- [src/clean_uav_core/launch/phase3_vins_pipeline.launch](../src/clean_uav_core/launch/phase3_vins_pipeline.launch)
- [src/traj_opt_v2/src/poly_traj_optimizer.cpp](../src/traj_opt_v2/src/poly_traj_optimizer.cpp)
- [src/clean_uav_core/config/swarm_planner_v2.yaml](../src/clean_uav_core/config/swarm_planner_v2.yaml)

## 基线先决条件

做任何消融前，先固定三个前提，否则数据很难比较：

1. 只跑同一种模式，不要把 truth odom、VINS bridge、手动接管混在同一个实验批次里。
2. 固定同一批初始位置、目标点和世界文件，避免场景本身成为变量。
3. 每组实验至少重复 3 次，记录均值、标准差和失败样本。

建议用当前已生成的 6 机顶层 launch 作为基线：

- V1: [src/clean_uav_core/launch/swarm_top_level_v1_6UAV.launch](../src/clean_uav_core/launch/swarm_top_level_v1_6UAV.launch)
- V2: [src/clean_uav_core/launch/swarm_top_level_v2_6UAV.launch](../src/clean_uav_core/launch/swarm_top_level_v2_6UAV.launch)

如果这些文件和当前 YAML 参数不同步，先重新生成一次默认输出：

```bash
python3 src/clean_uav_core/scripts/swarm_launch_generator_yaml.py --version v1
python3 src/clean_uav_core/scripts/swarm_launch_generator_yaml.py --version v2
```

默认生成模式会顺带重建 `3UAV/4UAV/5UAV/6UAV` 这些兼容变体，细节见 [docs/swarm_yaml_workflow_CN.md](./swarm_yaml_workflow_CN.md)。

## 先看什么，不先看什么

### 先看输出指标

消融实验优先看下面 5 个指标：

- 总完成时间：从全部无人机进入任务开始，到全部到达目标结束。
- 最小机间距离：全局最小值，以及是否低于安全阈值。
- 交汇停留时间：在冲突区域附近的累计时间。
- 重规划次数：每架机体的平均 replan 次数。
- 轨迹平滑性：可用 jerk 积分或速度方差近似。

### 暂时不要先看什么

以下指标可以记录，但不要作为第一判断依据：

- 单次规划器的瞬时耗时。
- 单次重规划时的局部最优 cost 值。
- 某个时刻的局部最小间距快照。

这些量很容易受噪声影响，只有和上面的主指标一起看才有意义。

## 参数分层

为了做消融，建议把所有参数分成 4 层：

### 1. 链路层

这层决定“告警是不是伪问题”。

- `enable_mavros_vision_pose`
- `enable_fallback_local_odom`
- `use_truth_odom_runtime`
- `enable_vins`

推荐做法：

- 6 机纯仿真基线先固定为 `use_truth_odom_runtime=true`。
- 如果要测 VINS，再单独开一个实验批次，不要和 truth odom 混跑。
- 如果目标是排查 `Estimator source 8`，先把 `enable_mavros_vision_pose=false` 作为对照组。

### 2. 几何安全层

这层决定“离得开不开”。

- `swarm_clearance`
- `swarm_acceptance_radius`
- `swarm_filter_far_trajectories`

推荐做法：

- 先固定 `swarm_filter_far_trajectories=false`，只用 clearance 和 weight 去看最基础的避碰效果。
- 只在基础避碰稳定后，再打开 far trajectory filter，看是否能减少远场耦合和抖动。

### 3. 反对称层

这层决定“会不会互相锁死”。

- `swarm_weight`
- `swarm_symmetry_gain`
- `weight_time`
- `weight_sqrvariance`

推荐做法：

- `swarm_weight` 负责安全边界强度。
- `swarm_symmetry_gain` 负责破局，通常只需要很小的值。
- `weight_time` 负责压缩无意义绕行。
- `weight_sqrvariance` 负责抑制过度扭摆和蛇形。

### 4. 恢复层

这层决定“失败后会不会继续拖”。

- 规划器重启逻辑
- 时间阈值 `swarm_time_warn_threshold`
- 重规划触发条件

这层通常不建议一开始就大改，先用它判断系统是不是进入了“更安全但更慢”的吸引域。

## 推荐调参顺序

消融实验不要同时动太多参数。推荐顺序如下。

### 阶段 A：先定安全边界

目标是让无人机“不会明显贴得太近”。

固定其它参数，只扫这两个：

- `swarm_clearance`
- `swarm_weight`

建议扫点：

- `swarm_clearance`: 0.45 / 0.55 / 0.65
- `swarm_weight`: 10000 / 25000 / 40000

判据：

- 如果最小距离仍然太小，说明 clearance 不够，而不是 weight 不够。
- 如果最小距离明显变大，但总完成时间也显著拉长，说明已经进入“安全但保守”的区域。

### 阶段 B：再破局

目标是让“镜像绕行”消失或明显减少。

在阶段 A 的最佳点上，只扫：

- `swarm_symmetry_gain`

建议扫点：

- 0.0
- 0.01
- 0.03
- 0.05

判据：

- 如果 `0.0` 和 `0.01` 差不多，说明对称性还没有被打破。
- 如果 `0.03` 开始明显减少交汇停留时间，同时不显著恶化最小距离，这是一个合适区间。
- 如果 `0.05` 让某些机体明显偏置过强，导致局部拥堵转移到别的路口，说明 gain 过大。

### 阶段 C：压缩无效周旋

目标是减少“慢慢绕、一直不进”的情况。

在前面稳定后，再扫：

- `weight_time`
- `weight_sqrvariance`

建议扫点：

- `weight_time`: 10 / 20 / 40
- `weight_sqrvariance`: 5000 / 10000 / 20000

判据：

- `weight_time` 升高后，总时间应当下降或至少不增长。
- `weight_sqrvariance` 升高后，轨迹应该更短更稳，但不能把机体推到直角折线或局部停顿。

### 阶段 D：再做邻居筛选

目标是减少过密冲突图带来的噪声。

这个阶段主要看：

- `swarm_filter_far_trajectories`
- `swarm_acceptance_radius`

建议从 `false` 切到 `true`，再把 `swarm_acceptance_radius` 设成一个有限值。

判据：

- 如果打开筛选后，总完成时间下降，且最小距离没有变差，说明远场耦合是噪声源。
- 如果打开筛选后反而更容易贴近，说明筛掉了本该参与协调的邻居，筛选过强。

## 推荐实验矩阵

为了避免实验规模爆炸，建议做一个 2 层矩阵，而不是全因子笛卡尔积。

### 第一层：基线对比

每组做 3 次重复：

1. Baseline A: 当前 6UAV 默认值。
2. Baseline B: 关闭 `swarm_symmetry_gain`。
3. Baseline C: 关闭 `enable_mavros_vision_pose`。

这一层的作用是拆开“告警链路”和“优化破局”的贡献。

### 第二层：核心扫参

在 Baseline A 基础上，只做 6 组：

1. `swarm_clearance=0.45`, `swarm_weight=25000`, `swarm_symmetry_gain=0.0`
2. `swarm_clearance=0.55`, `swarm_weight=25000`, `swarm_symmetry_gain=0.0`
3. `swarm_clearance=0.65`, `swarm_weight=25000`, `swarm_symmetry_gain=0.0`
4. `swarm_clearance=0.55`, `swarm_weight=25000`, `swarm_symmetry_gain=0.01`
5. `swarm_clearance=0.55`, `swarm_weight=25000`, `swarm_symmetry_gain=0.03`
6. `swarm_clearance=0.55`, `swarm_weight=25000`, `swarm_symmetry_gain=0.05`

这 6 组已经足够回答 80% 的问题：

- clearance 影响的是“能不能保持距离”。
- symmetry gain 影响的是“会不会互相僵持”。

如果你还需要继续深挖，再在第 5 组附近扫 `weight_time` 和 `weight_sqrvariance`。

## 怎么判读结果

### 情况 1：距离够了，但时间长

这说明问题主要不在避碰强度，而在“破局不足”。

处理顺序：

1. 先增大 `swarm_symmetry_gain`。
2. 再适度提高 `weight_time`。
3. 最后才考虑继续抬 `swarm_weight`。

### 情况 2：时间缩短了，但最小距离变小

这说明你把系统推得太激进了。

处理顺序：

1. 先回退 `swarm_symmetry_gain`。
2. 再提高 `swarm_clearance`。
3. 必要时轻微增加 `swarm_weight`。

### 情况 3：某一两架机体总是慢

这通常是 `drone_id` 偏置已经开始影响调度，或者该机体的几何路径本来更难走。

处理顺序：

1. 先检查是否是拓扑上更难的那条路径，不要误判为参数坏了。
2. 如果路径复杂度相近，再减小 `swarm_symmetry_gain`。
3. 必要时引入更明确的让路优先级，而不是继续加大偏置。

### 情况 4：告警还在

这说明你现在看到的不是优化问题，而是链路问题。

处理顺序：

1. 确认 `enable_mavros_vision_pose` 是否被意外打开。
2. 确认是否同时启用了 VINS bridge 和 truth odom。
3. 确认旧节点没有残留。

## 推荐的记录格式

每次实验至少记录这些字段：

- 实验名
- 运行时间
- 6 架机的初始位置和目标位置
- `swarm_clearance`
- `swarm_weight`
- `swarm_symmetry_gain`
- `weight_time`
- `weight_sqrvariance`
- `swarm_filter_far_trajectories`
- `enable_mavros_vision_pose`
- 是否出现 `Estimator source 8`
- 总完成时间
- 最小机间距离
- 每架机的重规划次数
- 交汇区停留时间

建议把这些字段按 CSV 或 YAML 统一落盘，后面画图会很省事。

## 建议的优先级结论

如果你后面只打算做少量消融，我建议按这个顺序：

1. 先固定链路，验证 `Estimator source 8` 是否真的消失。
2. 只扫 `swarm_clearance` 和 `swarm_weight`，找到安全边界。
3. 在安全边界上扫 `swarm_symmetry_gain`，观察是否减少锁死。
4. 再扫 `weight_time`，压缩无效周旋。
5. 最后才考虑 `swarm_filter_far_trajectories` 和 `swarm_acceptance_radius`。

一句话总结：

先确认“能飞且不会撞”，再确认“不会互相等”，最后才追求“更快更稳”。