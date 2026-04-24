# Swarm Benchmark 可视化与 Rosbag 速查

这份速查文档面向当前 research campaign 工作流，目标是两件事：一是在执行基准测试时直接看到 Gazebo 和 RViz；二是在已有样本落地后，快速读取和回放 rosbag。

如果你需要更完整的脚本执行说明、参数变化和离线回放流程细节，请直接看 [docs/swarm_benchmark_rosbag_offline_replay_and_campaign_update_CN.md](docs/swarm_benchmark_rosbag_offline_replay_and_campaign_update_CN.md)。

## 1. 可视化运行基准测试

先进入工作区：

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
```

直接以可视化模式运行已有样本：

```bash
python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_a \
  --case v2_speed_1p2 \
  --visualize
```

上面的 `--visualize` 会自动注入：

```text
gui:=true
use_rviz:=true
```

如果你只想开 Gazebo，不想开 RViz：

```bash
python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_a \
  --case v2_speed_1p2 \
  --launch-arg gui:=true \
  --launch-arg use_rviz:=false
```

如果你想换 RViz 配置：

```bash
python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_b \
  --case b_baseline_pathological \
  --visualize \
  --launch-arg rviz_config:=$(pwd)/src/clean_uav_core/rviz/swarm_rviz_v2.rviz
```

直接运行单个生成好的 launch 也支持可视化：

```bash
tools/benchmark_research/run_single_profile.sh \
  --version v2 \
  --launch-file benchmark_stage_b_pathological.launch \
  --output-dir benchmark_artifacts/manual_visual_runs/stage_b_pathological \
  --visualize
```

## 2. 重点命令速查

刷新 campaign 索引与分析：

```bash
python3 tools/benchmark_research/collect_campaign_index.py
python3 tools/pareto_frontier_analysis.py
python3 tools/risk_statistics_analysis.py
python3 tools/parameter_sensitivity_analysis.py
```

只生成 launch 和 run_spec，不实际执行：

```bash
python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_b \
  --case b_recovery_combo \
  --generate-only
```

运行 stage B 的首个病理基线：

```bash
python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_b \
  --case b_baseline_pathological \
  --visualize
```

运行 stage B 的组合修复 case：

```bash
python3 tools/benchmark_research/run_benchmark_campaign.py \
  --phase stage_b \
  --case b_recovery_combo \
  --visualize
```

查看已有 run 目录下的 bag：

```bash
find benchmark_artifacts/research_profiles/stage_a/v2_speed_1p2/run_01 -maxdepth 2 -name '*.bag' | sort
```

## 3. Rosbag 读取

`collect_campaign_index.py` 现在会直接调用 `rosbag info --yaml` 读取每个 run 下的 bag 元数据，并把以下字段写入：

- `rosbag_file_count`
- `rosbag_total_duration_sec`
- `rosbag_max_duration_sec`
- `rosbag_total_size_mb`
- `rosbag_total_messages`
- `rosbag_topics`

因此执行完索引刷新后，你可以直接在以下文件里看到 bag 级别信息：

- `benchmark_artifacts/research_profiles/campaign_index.json`
- `benchmark_artifacts/research_profiles/campaign_index.csv`

快速查看某个 bag 的原始元数据：

```bash
rosbag info --yaml \
  benchmark_artifacts/research_profiles/stage_a/v2_speed_1p2/run_01/auto_20260406_153515/drone_0_vel1p2_20260406_153515.bag
```

## 4. 一键回放 Rosbag + RViz

从 session 目录直接回放：

```bash
tools/benchmark_research/replay_benchmark_rosbag.sh \
  --input benchmark_artifacts/research_profiles/stage_a/v2_speed_1p2/run_01/auto_20260406_153515
```

从 run 目录直接回放，脚本会自动向下找 `.bag` 文件：

```bash
tools/benchmark_research/replay_benchmark_rosbag.sh \
  --input benchmark_artifacts/research_profiles/stage_a/v2_speed_1p2/run_01
```

以暂停态启动，先摆好 RViz 再播放：

```bash
tools/benchmark_research/replay_benchmark_rosbag.sh \
  --input benchmark_artifacts/research_profiles/stage_a/v2_speed_1p2/run_01 \
  --paused
```

不打开 RViz，只做纯回放：

```bash
tools/benchmark_research/replay_benchmark_rosbag.sh \
  --input benchmark_artifacts/research_profiles/stage_a/v2_speed_1p2/run_01 \
  --no-rviz
```

## 5. 使用提示

- 当前终端需要有图形显示环境，当前会话已经确认 `DISPLAY=:0`。
- `test_swarm_top_level_runtime_health.sh` 现在会尊重你显式传入的 `gui:=true` 和 `use_rviz:=true`，不会再被内部默认值强行改回去。
- 如果你想保留“正式 benchmark 执行”风格，但让画面更直观，优先使用 `--visualize`，必要时再补 `--launch-arg rviz_config:=...` 这种细粒度覆盖。