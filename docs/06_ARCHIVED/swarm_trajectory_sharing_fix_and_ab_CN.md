# 多机轨迹共享修复与 A/B 测试说明

## 背景

在当前三机对飞场景中，V1 与 V2 都出现了交汇时不互避、最终碰撞的问题。经过对 `clean_uav_core` 顶层 launch、`ego_planner` / `plan_manage_v2` 接收回调、以及 V1/V2 优化器使用 `swarm_traj` 缓冲区逻辑的核对，可以确认当前问题的主因不是“广播 topic 没对齐”，也不是“广播相关包没有启动”，而是“接收端过早丢弃远处轨迹”与“V2 默认安全距离偏小”。

## 关键判断

1. 当前 `clean_uav_core` 的 V1 与 V2 顶层入口都已使用全局共享广播 topic，而不是每机各自的隔离 topic。
2. V1 与 V2 都会在规划成功后持续发布自身轨迹，优化器也都会读取 `swarm_traj` 缓冲区计算 swarm cost。
3. V1 当前会在接收回调中直接丢弃时间差超过 0.25s 的轨迹，并且会把距离自己超过 `planning_horizon * 4 / 3` 的轨迹直接标记为无效。
4. V2 当前虽然时间策略较宽，但仍会把“暂时离自己较远”的轨迹作废；同时 V2 默认 `swarm_clearance=0.15`，对三机交汇场景过小。
5. 因此更合理的修复方向不是继续改 topic，而是：
   - 让远距离队友轨迹默认先进入缓冲区，不在接收阶段过早作废。
   - 将旧行为参数化，保留 A/B 测试入口。
   - 适度提高 V2 默认 `swarm_clearance`。

## 实施方案

### 1. 接收逻辑参数化

为 V1 / V2 都引入以下参数：

- `swarm/acceptance_radius`
- `swarm/filter_far_trajectories`
- `swarm/time_warn_threshold`
- `swarm/time_reject_threshold`

新默认行为：

- `filter_far_trajectories=false`
- `acceptance_radius=-1.0`
- `time_warn_threshold=0.25`
- `time_reject_threshold=10.0`

含义：

- 默认不在接收阶段作废远处轨迹，先存入缓冲区，由后续 collision check 和优化器按真实时空位置决定是否产生惩罚。
- 若需要复现旧行为，可将 `filter_far_trajectories=true` 并设置一个正的 `acceptance_radius`。
- 时间戳超过 `warn_threshold` 时仅告警；超过 `reject_threshold` 才直接丢弃。

### 2. V2 默认 swarm clearance 上调

将 V2 默认 `swarm_clearance` 从 `0.15` 提升到 `0.35`。

理由：

- V2 优化器内部实际使用的是带放大系数的 clearance 惩罚边界。
- `0.15` 在三机正面对冲或大角度交汇场景中过小，往往等到非常接近才会产生足够惩罚。
- 先提升到 `0.35`，能明显增加提前规避的空间，同时不至于过早引入大面积无解。

### 3. 顶层 launch 暴露 A/B 开关

在 `swarm_top_level.launch` 与 `swarm_top_level_v2.launch` 中统一暴露下列参数：

- `swarm_acceptance_radius`
- `swarm_filter_far_trajectories`
- `swarm_time_warn_threshold`
- `swarm_time_reject_threshold`
- `swarm_clearance`

这样用户可以直接通过 `roslaunch` 命令做 A/B 测试，而不需要再次改源码。

## 旧参数快照

### V1 旧行为

| 项目 | 旧值 / 旧规则 |
| --- | --- |
| 远轨迹过滤 | 开启，且写死在回调里 |
| 过滤阈值 | `planning_horizon * 4 / 3`，当前约为 `7.5 * 4 / 3 = 10.0m` |
| 时间戳告警阈值 | 无单独告警逻辑 |
| 时间戳拒绝阈值 | `0.25s` |
| `swarm_clearance` | `0.5` |

### V2 旧行为

| 项目 | 旧值 / 旧规则 |
| --- | --- |
| 远轨迹过滤 | 开启，且写死在回调里 |
| 过滤阈值 | 使用重建轨迹点到本机距离，小于 `planning_horizon * 4 / 3` 才保留，当前近似为 `10.0m` |
| 时间戳告警阈值 | `0.25s` |
| 时间戳拒绝阈值 | `10.0s` |
| `swarm_clearance` | `0.15` |

## 新默认参数

### V1 新默认

| 参数 | 默认值 |
| --- | --- |
| `swarm_acceptance_radius` | `-1.0` |
| `swarm_filter_far_trajectories` | `false` |
| `swarm_time_warn_threshold` | `0.25` |
| `swarm_time_reject_threshold` | `10.0` |
| `swarm_clearance` | `0.5` |

### V2 新默认

| 参数 | 默认值 |
| --- | --- |
| `swarm_acceptance_radius` | `-1.0` |
| `swarm_filter_far_trajectories` | `false` |
| `swarm_time_warn_threshold` | `0.25` |
| `swarm_time_reject_threshold` | `10.0` |
| `swarm_clearance` | `0.35` |

## A/B 测试建议命令

### V1 新默认

```bash
roslaunch clean_uav_core swarm_top_level.launch
```

### V1 近似复现旧行为

```bash
roslaunch clean_uav_core swarm_top_level.launch \
  swarm_filter_far_trajectories:=true \
  swarm_acceptance_radius:=10.0 \
  swarm_time_reject_threshold:=0.25 \
  swarm_clearance:=0.5
```

### V2 新默认

```bash
roslaunch clean_uav_core swarm_top_level_v2.launch
```

### V2 近似复现旧行为

```bash
roslaunch clean_uav_core swarm_top_level_v2.launch \
  swarm_filter_far_trajectories:=true \
  swarm_acceptance_radius:=10.0 \
  swarm_time_warn_threshold:=0.25 \
  swarm_time_reject_threshold:=10.0 \
  swarm_clearance:=0.15
```

## 观察指标

建议在 A/B 测试时同时关注：

1. 是否能在交汇前更早触发重规划。
2. 交汇区最近距离是否明显增大。
3. 是否出现过多“因队友轨迹过多而无解”的副作用。
4. 日志中是否仍频繁出现“时间戳差过大导致丢弃”的提示。

## 备注

当前仓库中 `docs/BROADCAST_BSPLINE_AND_SWARM_CN.md` 关于“默认广播 topic 仍按命名空间隔离”的描述已经落后于当前 `clean_uav_core` 实现；现在的顶层入口实际上已经统一使用全局广播 topic。判断问题时应以当前 launch 与源码实现为准。