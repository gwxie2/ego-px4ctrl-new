# Swarm YAML 配置与生成器教程

## 1. 为什么引入 YAML

当前 swarm 启动链路已经从“把大量参数硬编码在 Python 生成器里”，改为“配置在 YAML、生成器只负责装配 launch”。

现行入口有两个：

- `docs/uav_position_goal.md`：决定有哪些 `drone_X`，以及它们的起终点定义来源。
- `src/clean_uav_core/config/swarm_config.yaml`：决定仿真世界、基础地图参数、机体参数和 mission profile。

这样做的直接收益是：

- V1 和 V2 共用同一份 mission profile 源。
- 调参数时优先改 YAML，而不是改 Python 生成器。
- 生成器可以稳定重建默认 launch 和旧的 `3UAV/4UAV/5UAV/6UAV` 兼容文件。

## 2. 当前涉及的文件

- `src/clean_uav_core/config/swarm_config.yaml`
- `src/clean_uav_core/scripts/swarm_launch_generator_yaml.py`
- `src/clean_uav_core/launch/swarm_top_level_v1.launch`
- `src/clean_uav_core/launch/swarm_top_level_v2.launch`
- `src/clean_uav_core/launch/swarm_top_level.launch`

其中：

- `swarm_top_level.launch` 目前仍是 V1 兼容入口。
- `swarm_top_level_v1.launch` / `swarm_top_level_v2.launch` 是当前默认主输出。
- `swarm_top_level_v1_3UAV.launch` 这类文件是为了兼容旧实验脚本和旧文档保留的计数后缀输出。
- `swarm_config_2ms.yaml` / `swarm_config_5ms.yaml` / `swarm_config_10ms.yaml` 是可直接切换的独立速度预设。

## 3. `swarm_config.yaml` 结构

当前 YAML 分为四层：

### 3.1 `simulation`

控制仿真层默认行为，例如：

- `world_path`
- `gui`
- `use_rviz`
- `enable_vins`
- `use_truth_odom_runtime`

`world_path` 必须是 ROS 包内相对路径或 world 文件名，不能写绝对路径。

### 3.2 `physical_uav`

控制飞控相关共享物理参数，例如：

- `mass`
- `hover_percent`
- `pid_gain.Kp`
- `pid_gain.Kv`

这些值会被新生成器映射到 `px4ctrl` 的参数覆盖项。

### 3.3 `planning_base`

控制规划和地图的共享基础参数，例如：

- `map_size_x`
- `map_size_y`
- `map_size_z`
- `resolution`
- `ground_height`
- `odom_depth_timeout`
- `grid_map_depth_filter_mindist`
- `local_update_range_x/y/z`

### 3.4 `mission_profiles`

这是 profile 系统的核心。当前默认包含：

- `original`
- `fast`
- `extreme`

每个 profile 里定义：

- `max_vel`
- `max_acc`
- `max_jerk`
- `planning_horizon`
- `replan_time`
- `emergency_time`
- `weight_time`
- `lambda_smooth`
- `grid_map_obstacles_inflation`
- `swarm_clearance`
- `swarm_collision_weight`
- `swarm_weight`
- `swarm_symmetry_gain`

## 4. 生成器怎么工作

`swarm_launch_generator_yaml.py` 会做四件事：

1. 读取 `docs/uav_position_goal.md`，确定默认编队规模。
2. 读取 `swarm_config.yaml`，选出对应的 mission profile。
3. 把共享参数映射到 V1 或 V2 各自的 launch 参数面。
4. 生成顶层 launch 文件。

默认生成模式下：

- `--version v1` 会生成 `swarm_top_level_v1.launch`
- 同时写出 V1 兼容入口 `swarm_top_level.launch`
- 还会顺带重建 `swarm_top_level_v1_3UAV.launch`、`4UAV.launch`、`5UAV.launch`、`6UAV.launch`
- `--version v2` 会生成 `swarm_top_level_v2.launch`
- 同时重建 `swarm_top_level_v2_3UAV.launch`、`4UAV.launch`、`5UAV.launch`、`6UAV.launch`

显式传入 `--output` 时：

- 只写出你指定的那个文件
- 不自动额外生成上述兼容变体

## 5. 常用命令

### 5.1 生成默认 V1 文件

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh
python3 src/clean_uav_core/scripts/swarm_launch_generator_yaml.py --version v1
```

### 5.2 生成默认 V2 文件

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh
python3 src/clean_uav_core/scripts/swarm_launch_generator_yaml.py --version v2
```

### 5.3 选择 profile

```bash
python3 src/clean_uav_core/scripts/swarm_launch_generator_yaml.py \
  --version v2 \
  --profile fast
```

### 5.4 指定自定义位置配置

```bash
python3 src/clean_uav_core/scripts/swarm_launch_generator_yaml.py \
  --version v1 \
  --config docs/uav_position_goal.md
```

### 5.5 只输出一个定制文件

```bash
python3 src/clean_uav_core/scripts/swarm_launch_generator_yaml.py \
  --version v2 \
  --profile original \
  --output src/clean_uav_core/launch/test_swarm_custom.launch
```

### 5.6 使用独立速度预设 YAML

```bash
python3 src/clean_uav_core/scripts/swarm_launch_generator_yaml.py \
  --version v1 \
  --swarm-config src/clean_uav_core/config/swarm_config_2ms.yaml

python3 src/clean_uav_core/scripts/swarm_launch_generator_yaml.py \
  --version v2 \
  --swarm-config src/clean_uav_core/config/swarm_config_5ms.yaml

python3 src/clean_uav_core/scripts/swarm_launch_generator_yaml.py \
  --version v2 \
  --swarm-config src/clean_uav_core/config/swarm_config_10ms.yaml
```

这三份文件都只保留了 `mission_profiles.original`，因此可以直接使用生成器默认的 `--profile original`，不需要额外再切 profile 名称。

## 6. V1 与 V2 的关系

YAML 是共享源，但 V1/V2 内部参数名并不完全一致。

当前生成器会做版本映射：

- V1 会重点覆盖 `planner_lambda_smooth`、`swarm_collision_weight` 等 V1 参数。
- V2 会重点覆盖 `planner_weight_time`、`swarm_weight`、`swarm_symmetry_gain` 等 V2 参数。

因此，推荐的工作方式是：

- 先在 YAML 里调共享 profile 语义。
- 再通过 `--version v1` 或 `--version v2` 生成对应 launch。
- 不要直接把 V1 的 launch 参数名当成 YAML 字段名去维护两套配置。

## 7. 和旧生成器的关系

旧文件 `src/clean_uav_core/scripts/swarm_launch_generator.py` 仍然保留在仓库里，主要用于历史实现对照和版本追踪。

当前建议使用的新入口只有：

- `src/clean_uav_core/scripts/swarm_launch_generator_yaml.py`

如果你在旧文档中看到 `swarm_launch_generator.py`，请把它理解为“历史方案名称”；实际执行时应优先换成 YAML 生成器。

## 8. 常见问题

### 8.1 `unknown profile` 报错

原因：`--profile` 指向了 YAML 中不存在的名字。

处理方式：

- 检查 `src/clean_uav_core/config/swarm_config.yaml` 的 `mission_profiles`
- 使用生成器报错信息里列出的可用 profile 名称

### 8.2 为什么默认会多生成几个 `*_6UAV.launch`

这是为了兼容旧实验脚本和旧文档中的计数后缀入口。当前默认生成模式会顺带把这些文件一起重建。

如果你不想生成兼容文件，请显式传 `--output` 到一个自定义 launch 文件。

### 8.3 为什么不要写绝对路径

因为这套流程需要能在不同工作区位置复用。当前生成器已经避免把绝对路径硬编码进 launch，尤其是 commander 的配置文件路径和 world 路径。

## 9. 推荐工作流

建议按这个顺序操作：

1. 修改 `docs/uav_position_goal.md` 或运行 `generate_swarm_config.py` 更新队形。
2. 修改 `src/clean_uav_core/config/swarm_config.yaml` 里的 profile。
3. 运行 `swarm_launch_generator_yaml.py` 生成 V1 或 V2 launch。
4. 用 `roslaunch --nodes`、`test_swarm_smoke.sh` 或 `test_swarm_v2_smoke.sh` 做快速验证。

如果需要做 6 机实验，优先先重新运行一次默认生成命令，确保 `swarm_top_level_v1_6UAV.launch` / `swarm_top_level_v2_6UAV.launch` 与当前 YAML 保持同步。