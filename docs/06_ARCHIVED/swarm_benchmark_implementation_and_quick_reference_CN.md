# Swarm 性能基准实现、运行解读与快速命令对照手册

## 1. 文档目的

本文面向后续维护者与实验执行者，说明三件事情：

1. 当前 swarm benchmark 体系到底由哪些代码与脚本组成。
2. 一次完整 benchmark 如何执行，产物分别是什么。
3. 看到某些日志或结果时，应该如何快速判断是否正常。

本文对应当前已验证通过的入口：

- `src/clean_uav_core/launch/swarm_top_level_v1.launch`
- `src/clean_uav_core/launch/swarm_top_level_v2.launch`
- `test_swarm_top_level_smoke.sh`
- `test_swarm_top_level_runtime_health.sh`

---

## 2. 当前 benchmark 实现由哪些部分组成

### 2.1 顶层入口

- `src/clean_uav_core/launch/swarm_top_level_v1.launch`
- `src/clean_uav_core/launch/swarm_top_level_v2.launch`

作用：

- 启动 Gazebo / PX4 / MAVROS / planner / px4ctrl / benchmark manager。
- 将 benchmark 参数透传到 `benchmark_manager.py`。

### 2.2 benchmark 核心管理器

- `src/clean_uav_core/scripts/benchmark_manager.py`

作用：

- 管理 benchmark session 生命周期。
- 订阅 odom / command / replan / event / safety / MAVROS / px4ctrl debug。
- 生成主 CSV、event CSV、replan CSV、manifest、summary。
- 按无人机启动与停止 rosbag。
- 暴露 `/benchmark/start_session` 服务。

### 2.3 规划器侧埋点

- `quadrotor_msgs/PlannerReplanInfo.msg`
- `quadrotor_msgs/PlannerBenchmarkEvent.msg`
- `ego_planner` / `ego_planner_v2` 的 FSM 与 planner manager 埋点

作用：

- 输出重规划耗时、迭代次数、成功与失败原因。
- 输出状态迁移事件，用于自动起 session 与行为分析。

### 2.4 报表脚本

- `tools/plot_advanced_performance.py`

作用：

- 离线读取 session 目录。
- 生成对比图、汇总表、PDF 报告和 summary JSON。

### 2.5 测试与执行脚本

- `test_swarm_top_level_smoke.sh`
- `test_swarm_top_level_runtime_health.sh`
- `test_swarm_v1_runtime_health.sh`
- `test_swarm_v2_runtime_health.sh`
- `cleanup_swarm.sh`

作用：

- `smoke`：做静态图检查 + 运行态健康检查。
- `runtime_health`：真正启动系统并检查 artifact 是否落盘。
- `cleanup_swarm.sh`：清理残留 ROS / Gazebo / PX4 进程。

---

## 3. 一次完整 benchmark 会生成什么

每个 session 目录下，至少应看到以下文件：

- `*_manifest.json`
- `*_summary.json`
- `drone_X_*.csv`
- `drone_X_*_event.csv`
- `drone_X_*_replan.csv`
- `drone_X_*.bag`

含义如下：

### 3.1 manifest

记录 session 启动时的基本信息：

- `session_id`
- `planner_node_name`
- `drone_ids`
- `goals`
- `v_max`
- `output_dir`

### 3.2 summary

记录每架无人机的聚合结果：

- `tracking_error_p95`
- `control_lag_p95_ms`
- `planner_latency_p95_ms`
- `replan_success_rate`
- `min_safety_margin_m`
- `stop_reason`

### 3.3 主 CSV

主 CSV 是最核心的时间序列数据，包含：

- odom 位姿与速度
- `position_cmd`
- tracking error
- control lag
- actuator ratio
- safety margin
- planner latency

如果主 CSV 只有 1 行表头，没有数据行，说明 benchmark 主采样链路有问题，不能拿去做报告。

### 3.4 event CSV

记录状态机迁移和关键事件，例如：

- `WAIT_TARGET -> GEN_NEW_TRAJ`
- `REPLAN_TRAJ -> EXEC_TRAJ`
- `EMERGENCY_STOP`

### 3.5 replan CSV

记录每一次重规划的详细开销：

- `success`
- `time_total_ms`
- `replan_interval_ms`
- `iter_count`
- `failure_reason`

### 3.6 rosbag

用于后续离线复核原始 ROS 话题。

如果目录里只剩 `.bag.active`，说明 rosbag 没有正常收尾，当前会话不能算“完整闭环”。

---

## 4. 最常用的执行命令

所有 ROS / catkin 相关命令前，都先执行：

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh
```

### 4.1 清理环境

```bash
./cleanup_swarm.sh
```

用途：

- 结束残留 `roslaunch` / `roscore` / `PX4` / `Gazebo` 进程。
- 每次正式 benchmark 前建议先执行一次。

### 4.2 两步 smoke 测试

```bash
./test_swarm_top_level_smoke.sh
```

只测 V1：

```bash
./test_swarm_top_level_smoke.sh --version v1
```

只测 V2：

```bash
./test_swarm_top_level_smoke.sh --version v2
```

用途：

- 先做静态节点图检查，再做运行态健康检查。
- 适合提交前回归和快速自检。

### 4.3 跑 V1 完整 benchmark

```bash
./test_swarm_v1_runtime_health.sh \
  --output-dir benchmark_artifacts/manual_runs/v1 \
  --wait-for-session-complete
```

### 4.4 跑 V2 完整 benchmark

```bash
./test_swarm_v2_runtime_health.sh \
  --output-dir benchmark_artifacts/manual_runs/v2 \
  --wait-for-session-complete
```

### 4.5 在命令行里直接覆写参数

```bash
./test_swarm_top_level_runtime_health.sh \
  --version v2 \
  --output-dir benchmark_artifacts/manual_runs/v2_case_a \
  --wait-for-session-complete \
  -- \
  planner_max_vel:=2.0 \
  planner_max_acc:=3.2 \
  swarm_clearance:=0.45 \
  benchmark_default_vmax:=2.0
```

说明：

- `--` 后的内容会原样透传给底层 `roslaunch`。
- 这是最推荐的试验参数扫描方式。

### 4.6 生成高级离线报告

```bash
/usr/bin/python3 tools/plot_advanced_performance.py \
  benchmark_artifacts/20260403_top_level_final/v1/auto_20260403_104104 \
  benchmark_artifacts/20260403_top_level_final/v2/auto_20260403_104514 \
  --output-dir benchmark_artifacts/20260403_top_level_final/report
```

### 4.7 快速检查目录内容

```bash
find benchmark_artifacts/20260403_top_level_final/v1 -maxdepth 2 -type f | sort
```

### 4.8 快速检查主 CSV 是否真有数据

```bash
wc -l benchmark_artifacts/20260403_top_level_final/v1/*/drone_0_*.csv
```

判读：

- 主 CSV 行数远大于 1，说明采样正常。
- 如果主 CSV 只有 1 行而 event/replan 有数据，说明主 topic 订阅链出问题。

### 4.9 快速查看 summary

```bash
python3 -m json.tool \
  benchmark_artifacts/20260403_top_level_final/v2/auto_20260403_104514/auto_20260403_104514_summary.json
```

---

## 5. runtime health 的 6 个步骤怎么读

运行 `test_swarm_top_level_runtime_health.sh` 时，会看到 `[1/6]` 到 `[6/6]`。

### [1/6] Launch headless stack

说明系统正在启动顶层 launch，并开启 benchmark。

如果这里失败，优先看：

- launch 文件本身是否能 `roslaunch --nodes`
- Gazebo / PX4 残留进程是否已清理

### [2/6] Validate base telemetry and MAVROS connectivity

检查：

- `/drone_X/odom`
- `/iris_X/mavros/state connected=true`

如果失败，通常不是 benchmark 逻辑问题，而是仿真层或命名空间映射问题。

### [3/6] Validate takeoff and planner bring-up

检查：

- 飞机是否 `armed`
- planner 与 traj server 是否起来

如果 planner 没起来，优先检查 launch remap、等待起飞脚本、或 planner 节点本身日志。

### [4/6] Validate planner outputs and benchmark telemetry

检查：

- `position_cmd`
- `planning/replan_info`

如果这里失败，说明系统还没到“可分析轨迹”的阶段。

### [5/6] Validate benchmark session artifacts

检查：

- `manifest`
- `replan CSV`
- `event CSV`
- 如果传了 `--wait-for-session-complete`，还会检查 `summary` 和正式 `.bag`

### [6/6] Validate optional expectations

这是给特定场景保留的扩展检查。

---

## 6. 看到结果后应该重点看哪些字段

### 6.1 `stop_reason`

最直接的 session 结束结论。

- `goal_reached`：理想状态
- `emergency_stop`：保护性停机，说明本轮仍需关注安全边界

### 6.2 `tracking_error_p95`

反映轨迹跟踪误差的上侧分位数。

- 越小越好。
- 比平均值更适合反映“尾部风险”。

### 6.3 `control_lag_p95_ms`

反映命令到状态之间的相对滞后。

- 若长期超过 `100 ms`，高速实验风险会上升。

### 6.4 `planner_latency_p95_ms`

反映规划器在重规划时的上侧延迟。

- 同等场景下，越小越说明规划链更轻。

### 6.5 `replan_success_rate`

反映规划器在本轮任务中的成功率。

- 低于 `90%` 时，不建议继续抬速度。
- 低于 `70%` 通常已经说明场景或参数存在显著问题。

### 6.6 `min_safety_margin_m`

这是当前最重要的安全边界指标之一。

- 越低越危险。
- 与 `swarm_clearance` 结合看，不要单看绝对值。

### 6.7 `safety_violation_count`

使用时要注意：

- 它跟阈值绑定。
- 如果 V1 / V2 `swarm_clearance` 不同，则这个计数**不能直接横向比较**。

### 6.8 `benchmark_default_vmax`

它只是 benchmark 元数据，不一定等于本轮真实 `planner_max_vel`。

因此：

- 看到 bag 文件名里有 `vel10`，不要自动理解为“这就是 10m/s 实验”。
- 必须同时检查 launch 里的实际 `planner_max_vel`。

---

## 7. 常见故障与快速判断

### 7.1 `No odom for drone_X`

优先判断：

- Gazebo 是否正常启动。
- `/drone_X/odom` 是否被 `odom_pose_adapter` 正常发布。
- 是否存在上一次残留进程干扰。

### 7.2 `Benchmark manifest was not created`

优先判断：

- planner event 是否真的发到 benchmark manager 订阅的 topic。
- benchmark manager 是否已经自动起 session。

### 7.3 只有 `.bag.active` 没有 `.bag`

说明 rosbag 没有正常结束。

常见原因：

- session 没等自然完成。
- launch 被过早打断。

正确做法：

- 使用 `--wait-for-session-complete`。

### 7.4 主 CSV 只有表头

说明 benchmark manager 主采样 topic 没对上。

这类问题以前真实出现过，典型根因包括：

- 把 `odom`、`mavros/state`、`setpoint_raw/attitude` 订错到错误命名空间。
- 忘记区分 `drone_X` 上层话题与 `iris_X` MAVROS 话题。

### 7.5 一个完整 session 后又多出第二个半截 auto session

说明自动起 session 条件太宽且缺少“单次 launch 仅自动起一次”的限制。

当前代码已经修过这个问题；如果复发，优先回看 `benchmark_manager.py` 的 auto-start 逻辑。

---

## 8. 当前已验证通过的真实产物

### 8.1 V1

- 目录：`benchmark_artifacts/20260403_top_level_final/v1/auto_20260403_104104/`
- 内容：主 CSV、event CSV、replan CSV、manifest、summary、3 个正式 rosbag

### 8.2 V2

- 目录：`benchmark_artifacts/20260403_top_level_final/v2/auto_20260403_104514/`
- 内容：主 CSV、event CSV、replan CSV、manifest、summary、3 个正式 rosbag

### 8.3 报表

- PDF：`benchmark_artifacts/20260403_top_level_final/report/advanced_benchmark_report.pdf`
- 汇总 JSON：`benchmark_artifacts/20260403_top_level_final/report/advanced_benchmark_summary.json`

---

## 9. 使用建议

如果你的目标是：

- **确认系统是否还能跑**：先跑 `test_swarm_top_level_smoke.sh`
- **拿真实 benchmark 数据**：跑 `test_swarm_v1_runtime_health.sh` / `test_swarm_v2_runtime_health.sh`
- **做调参扫描**：用 `test_swarm_top_level_runtime_health.sh -- --param:=value`
- **做导师展示材料**：优先引用最终 report 目录与 summary JSON，而不是只贴终端日志

本文档可以作为后续 benchmark 实验的标准执行与排障入口。