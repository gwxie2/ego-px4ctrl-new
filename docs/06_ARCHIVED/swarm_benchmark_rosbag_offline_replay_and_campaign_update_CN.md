# Swarm Benchmark Rosbag 离线回放与脚本更新说明

## 1. 文档目的

这份补充文档专门解释两件事：

1. 现在这套 rosbag 离线回放脚本应该怎么执行，输入应该传什么，输出会看到什么。
2. 目前的基准测试脚本链路做了哪些更新，为什么这些更新能让离线回放、可视化运行和索引汇总连起来。

这份文档是对 [docs/swarm_benchmark_visual_quick_reference_CN.md](docs/swarm_benchmark_visual_quick_reference_CN.md) 的详细补充，不替代速查页，而是给“我要实际跑一遍”的场景准备的。

## 2. 当前脚本链路

现在这条链路是：

`run_benchmark_campaign.py` -> `run_single_profile.sh` -> `test_swarm_top_level_runtime_health.sh` -> 生成 session 目录和 rosbag -> `collect_campaign_index.py` -> `replay_benchmark_rosbag.sh`

其中：

- `run_benchmark_campaign.py` 负责 campaign 级别的 case 调度、launch 生成、run_spec 写入和运行目录管理。
- `run_single_profile.sh` 负责单次 benchmark 的统一包装，并把运行参数传给 runtime health 链路。
- `collect_campaign_index.py` 负责读取 run 目录中的 summary 和 rosbag 元数据，生成 campaign 总索引。
- `replay_benchmark_rosbag.sh` 负责把已有 bag 从 run 目录或 session 目录里回放出来，并可选打开 RViz。

## 3. 基准测试脚本更新

### 3.1 `run_single_profile.sh`

现在这个脚本新增了 `--visualize`。

执行后会自动补入：

- `gui:=true`
- `use_rviz:=true`

如果你已经显式传了同名参数，脚本会保留你的显式设置，不会强行覆盖。

这意味着你可以保留原有 benchmark 组织方式，只在需要看画面时加一个 `--visualize`。

### 3.2 `run_benchmark_campaign.py`

这个脚本现在也支持 `--visualize`，并且支持额外的 `--launch-arg`。

更新后有三个关键变化：

1. `--visualize` 会统一注入 `gui:=true` 和 `use_rviz:=true`。
2. 额外的 `--launch-arg key:=value` 会被写入 `run_spec.json`，方便后续复盘。
3. 如果同一个 case 之前已经跑过，脚本会自动选择新的 `run_XX` 目录，避免旧结果覆盖或误读。

这三点很重要，因为它们把“可视化调试”、“正式跑分”和“结果留痕”统一起来了。

### 3.3 `collect_campaign_index.py`

这个脚本现在不只是汇总 summary，还会直接读取每个 `.bag` 的元数据。

它会调用：

```bash
rosbag info --yaml <bag_path>
```

然后把下面这些字段写进 campaign 索引：

- `rosbag_file_count`
- `rosbag_total_duration_sec`
- `rosbag_max_duration_sec`
- `rosbag_total_size_mb`
- `rosbag_total_messages`
- `rosbag_topics`

这带来的直接好处是：

- 你可以在不打开 bag 的情况下先看每个 run 的 bag 覆盖范围。
- 后续做 Pareto、风险统计和参数敏感度分析时，索引里已经有 bag 级别元数据。
- 你能快速判断某次 run 是不是完整闭环，而不是只看 summary JSON。

### 3.4 `replay_benchmark_rosbag.sh`

这个脚本是当前推荐的 rosbag 离线回放入口。

它支持的输入方式有三种：

- `--input <run_dir>`：传整个 run 目录，脚本会自动向下找 `.bag`。
- `--input <session_dir>`：传 session 目录，脚本直接回放该目录下的 bag。
- `--bag <bag_path>`：直接指定一个 bag 文件。

它支持的控制项有：

- `--rate <x>`：控制播放速度。
- `--paused`：暂停启动，方便先把 RViz 视图摆好。
- `--loop`：循环播放。
- `--no-rviz`：只做纯回放，不打开 RViz。
- `--rviz-config <path>`：指定 RViz 配置文件。

脚本默认会做这些事：

- 进入工作区并 source `tools/source_phase1_env.sh`。
- 必要时自动启动 `roscore`。
- 设置 `use_sim_time=true`。
- 默认打开 RViz，并使用 `clean_uav_core/rviz/swarm_rviz_v2.rviz`。

## 4. rosbag 离线回放怎么跑

### 4.1 从 run 目录回放

这是最常见的方式。

```bash
tools/benchmark_research/replay_benchmark_rosbag.sh \
  --input benchmark_artifacts/research_profiles/stage_a/v2_speed_1p2/run_01
```

脚本会自动在这个 run 目录下查找 bag 文件，然后逐个回放。

### 4.2 从 session 目录回放

如果你已经知道某个 session 目录，也可以直接传进去：

```bash
tools/benchmark_research/replay_benchmark_rosbag.sh \
  --input benchmark_artifacts/research_profiles/stage_a/v2_speed_1p2/run_01/auto_20260406_153515
```

这种方式更适合你已经确认要看哪一次 session 的原始轨迹。

### 4.3 先暂停再播放

如果你想先打开 RViz，再开始播放：

```bash
tools/benchmark_research/replay_benchmark_rosbag.sh \
  --input benchmark_artifacts/research_profiles/stage_a/v2_speed_1p2/run_01 \
  --paused
```

这适合你要做离线复核、对照轨迹或者手动调 RViz 视角的时候使用。

### 4.4 只回放，不打开 RViz

```bash
tools/benchmark_research/replay_benchmark_rosbag.sh \
  --input benchmark_artifacts/research_profiles/stage_a/v2_speed_1p2/run_01 \
  --no-rviz
```

这个模式适合你只想验证 bag 是否能正常播放，或者想把播放结果接到别的监听节点上。

### 4.5 指定播放速率

```bash
tools/benchmark_research/replay_benchmark_rosbag.sh \
  --input benchmark_artifacts/research_profiles/stage_a/v2_speed_1p2/run_01 \
  --rate 0.5
```

低速回放适合检查 planner 重规划点、碰撞逼近区间和控制滞后。

## 5. 基准测试和回放的完整推荐流程

如果你要把这套流程完整跑通，推荐顺序是：

1. 先用 `run_benchmark_campaign.py` 或 `run_single_profile.sh` 生成一个正式 run。
2. 等 session 结束后，确认 run 目录里有 `.bag` 而不是 `.bag.active`。
3. 再运行 `collect_campaign_index.py`，把 bag 元数据写进 `campaign_index.json/csv`。
4. 如果需要复核原始行为，再用 `replay_benchmark_rosbag.sh` 回放该 run 或 session。

## 6. 常见判断

### 6.1 只有 `.bag.active`

说明 rosbag 没有正常收尾，这次 session 不能算完整闭环。

### 6.2 `campaign_index` 里看不到 bag 信息

先确认 run 目录下是否真的有 `.bag` 文件，再确认 `collect_campaign_index.py` 是否在同一个仓库根目录下运行。

### 6.3 回放时 RViz 没有显示

先检查是否有图形环境，再确认 `--rviz-config` 是否指向正确文件。

### 6.4 回放播放了，但没有预期话题

先看 bag 内 `rosbag info --yaml` 的 topics 列表，再确认 RViz 订阅的 topic 名称和 namespace 是否匹配。

## 7. 这次文档补充的重点

这次补充的不是“又多了一个命令”，而是把三件事连起来了：

- 基准脚本现在能显式切换可视化和 headless。
- campaign 索引现在能读 rosbag 元数据。
- 离线回放现在能从 run/session 目录直接复核原始 bag。

这样你在汇报时就可以很清楚地说：这套研究流程不仅能跑分，还能回放、索引和复核。