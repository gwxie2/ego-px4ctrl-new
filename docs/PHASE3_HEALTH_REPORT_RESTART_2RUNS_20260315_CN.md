# Phase-3 重启后双实验健康度报告（2026-03-15）

## 1. 重启说明（已执行）

本轮实验前已完成三层全重启：

1. 终端 A：`phase2_px4_multi_sim.launch`
2. 终端 B：`phase3_vins_pipeline.launch odom_mode:=vins_only`
3. 终端 C：`phase3_dual_uav_vins_stack.launch odom_mode:=vins_only`

并确认关键节点在线（Gazebo / 双 MAVROS / 双 VINS / bridge / adapter / 双机 planner+px4ctrl）。

## 2. 实验定义

- 脚本：`tools/phase3_health_quant.py`
- 模式：`vins_only`
- 实验 A（Run-1）：45 秒
  - JSON：`logs/phase3_health_report_vins_only_restart_45s.json`
- 实验 B（Run-2）：60 秒
  - JSON：`logs/phase3_health_report_vins_only_restart_60s.json`

## 3. 结果汇总

### 3.1 频率健康（Run-1 / Run-2）

- `/iris_0/vins_estimator/imu_propagate`：`247.635 / 247.988 Hz`
- `/iris_1/vins_estimator/imu_propagate`：`208.858 / 207.389 Hz`
- `/iris_0/vins_estimator/odometry`：`9.531 / 9.776 Hz`
- `/iris_1/vins_estimator/odometry`：`9.532 / 9.775 Hz`
- 深度话题（双机）：约 `19.85~19.99 Hz`
- `backward_stamp_count`：全部为 `0`

### 3.2 误差健康（Run-1 / Run-2）

- `iris_0 odom_vs_truth mean`：`25833.9052 m / 51123.2183 m`
- `iris_1 odom_vs_truth mean`：`7220.4405 m / 16443.1465 m`
- `iris_0 odom_vs_pose mean`：`25844.9143 m / 51131.6597 m`
- `iris_1 odom_vs_pose mean`：`7217.6908 m / 16455.5721 m`

### 3.3 飞控状态（两次实验）

- `iris_0`：`armed=true, mode=OFFBOARD, connected=true`
- `iris_1`：`armed=true, mode=OFFBOARD, connected=true`

### 3.4 健康度分数

- Run-1（45s）：`20.0 / 100`
- Run-2（60s）：`20.0 / 100`

## 4. 结论

1. **重启后链路稳定在线**：规划与控制链可持续运行，频率链路基本稳定。
2. **VINS 质量问题仍显著**：虽然高频链路（imu_propagate）稳定，但里程计绝对误差极大，健康度分数长期维持在低位（20分）。
3. **问题已从“链路通不通”转为“估计准不准”**：这与 Phase-4 方向一致，应聚焦 VINS 数值稳定性与漂移控制。

## 5. Phase-4 直接任务建议

- 以 `vins_only` 为评估主模式，持续使用本脚本回归。
- 把“误差绝对值量级过大”作为首要阻塞项，优先排查：
  - 双机配置一致性（尤其 iris_0/iris_1）
  - 时间同步与初始化窗口
  - 外参与 frame 对齐
- 维持 `vins_with_fallback` 仅用于演示稳态，不用于健康度结论。
