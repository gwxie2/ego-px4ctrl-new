# 覆盖记忆与虚拟目标能力两阶段实施方案

**目标**：在不修改 `ego_planner` 或 `px4ctrl` C++ 源码的前提下，把全局覆盖记忆、FoV 投影、虚拟目标检测、实时指标发布和自动报表能力，作为一条独立的 Python 增量链路接入现有基准测试体系。

**适用对象**：当前 research campaign、已有 `benchmark_manager.py` 运行链路、以及 `run_benchmark_campaign.py` / `collect_campaign_index.py` 的离线分析体系。

---

## Phase 1: 独立 Python helper 落地

### 1.1 目标

先把所有新增逻辑收敛到一个独立 Python 文件中，建议放在 [src/clean_uav_core/scripts](../src/clean_uav_core/scripts) 下，例如 `benchmark_coverage_engine.py`。

这个 helper 负责：

- 维护全局低分辨率 2D 覆盖网格。
- 根据 `/drone_%d/odom` 做 80 度 FoV 投影。
- 统计累计覆盖率、探索率、冗余因子、轨迹 jerk。
- 从参数服务器读取 `target_positions` 并做虚拟目标检测。
- 在目标首次被发现时输出 `Time_to_First_Detection`。
- 发布覆盖与目标事件的运行时消息，并在 session 结束时导出 JSON 与图表。

### 1.2 输入与输出

**输入**：

- `/drone_%d/odom`
- 现有 benchmark session 里的 run spec / summary / CSV
- 参数服务器中的 `UAV_NUM` 或 `default_drone_ids`
- `target_positions`
- `coverage_grid_resolution`
- `coverage_fov_deg`
- `target_distance_threshold`
- `coverage_publish_hz`

**输出**：

- `/benchmark/coverage_metrics`
- `TargetDetected` 事件或等价消息
- `coverage_metrics.json`
- `target_detection.json`
- `coverage_vs_time.png`
- 自动汇总的 session 报表

### 1.3 接入方式

helper 不直接替换 benchmark_manager，而是由 [src/clean_uav_core/scripts/benchmark_manager.py](../src/clean_uav_core/scripts/benchmark_manager.py) 导入并调用：

- 在 odom 回调里更新覆盖网格。
- 在 replan/event 回调里更新指标。
- 在监控定时器里刷新发布频率。
- 在 session stop 时统一落盘。

### 1.4 当前状态

- **已完成的底座**：现有 benchmark_manager 已经能订阅 odom、position_cmd、replan_info、benchmark_event、safety、attitude 和 MAVROS state，并且已经能生成 summary。
- **尚未完成的新能力**：覆盖网格、FoV 投影、虚拟目标、CoverageMetrics topic、TargetDetected topic、离线报表。

---

## Phase 2: 工程化接入 campaign 与报表链路

### 2.1 目标

把 Phase 1 的 helper 结果接入现有 research campaign 的结果索引、离线分析和文档体系，形成可复用的 benchmark artifact。

### 2.2 需要接入的位置

- [tools/benchmark_research/run_benchmark_campaign.py](../tools/benchmark_research/run_benchmark_campaign.py)
- [tools/benchmark_research/collect_campaign_index.py](../tools/benchmark_research/collect_campaign_index.py)
- [src/clean_uav_core/config/benchmark_research/campaign_matrix_v2.yaml](../src/clean_uav_core/config/benchmark_research/campaign_matrix_v2.yaml)
- [src/clean_uav_core/scripts/swarm_launch_generator_yaml.py](../src/clean_uav_core/scripts/swarm_launch_generator_yaml.py)
- [docs/swarm_research_campaign_usage_CN.md](swarm_research_campaign_usage_CN.md)
- [docs/benchmark_metrics_and_design_details_CN.md](benchmark_metrics_and_design_details_CN.md)

### 2.3 工程化内容

- 将 coverage / target / TTD / jerk 结果写入 campaign index。
- 保留对已有 run 的离线回填能力。
- 若需要更严格的 topic 语义，再补 `quadrotor_msgs` 的消息定义。
- 把 `UAV_NUM` 作为兼容别名继续保留，避免破坏现有 `drone_count` / `default_drone_ids` 链路。

### 2.4 验证顺序

1. 先让单个 run 产出 helper 的 JSON / PNG。
2. 再让 campaign index 读入新字段。
3. 最后补文档和分析脚本的图表说明。

---

## 实施边界

- 不改 `ego_planner` 或 `px4ctrl` C++ 源码。
- 不把覆盖统计散落到多个脚本里。
- 时间统计以 ROS 时间戳为准，wall time 只用于会话生命周期。
- 默认仍以 1.2 m/s 基线作为回归验证速度。
