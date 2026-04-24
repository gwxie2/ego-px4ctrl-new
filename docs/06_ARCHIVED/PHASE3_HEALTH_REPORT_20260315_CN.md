# PHASE-3 健康度报告（2026-03-15）

## 1. 实验配置

- 模式：`vins_only`
- 采样脚本：`tools/phase3_health_quant.py`
- 正式采样时长：60 秒
- 报告 JSON：`logs/phase3_health_report_vins_only_60s.json`
- 对照采样（预检）：`logs/phase3_health_report_vins_only_45s.json`

## 2. 本轮关键事实

1. 规划链路已可运行：在 Phase-3 栈日志中可见多次 `plan_success=1`。
2. TF 重复告警问题已解决：双机 VINS TF 已使用唯一前缀（`iris_0_body/iris_1_body`）。
3. `Depth Lost` 主触发链已显著缓解，系统不再以该错误主导崩溃。
4. 当前主问题转移为 VINS 估计质量，且两机表现明显不对称。

## 3. 量化结果摘要（60s）

### 3.1 频率与时序

- `/iris_0/vins_estimator/imu_propagate`：`204.051 Hz`
- `/iris_1/vins_estimator/imu_propagate`：`246.800 Hz`
- `/iris_0/vins_estimator/odometry`：`9.763 Hz`
- `/iris_1/vins_estimator/odometry`：`9.762 Hz`
- 深度图（双机）：约 `19.96 Hz`
- 所有监控话题 `backward_stamp_count = 0`（未见时间戳倒退）

### 3.2 里程计一致性/漂移

- `iris_0`：
  - `odom_vs_truth mean = 7.8084 m`
  - `odom_vs_pose_offset mean = 4.8216 m`
- `iris_1`：
  - `odom_vs_truth mean = 1,515,482.7674 m`
  - `odom_vs_pose_offset mean = 1,515,484.4814 m`

补充单帧核验：

- `/iris_1/odometry` 已出现百万量级坐标（例如 x≈-579123, y≈1180908, z≈-337113），
  明确不是统计脚本误差，而是估计发散。

### 3.3 健康度分数

- `health_score = 23.5 / 100`

## 4. 阶段判断

- Phase-3 **链路层面**：已打通并具备可运行性（相对 Phase-2 的核心推进已完成）。
- Phase-3 **质量层面**：未达标，主要瓶颈在 VINS（尤其 `iris_1`）的稳定性与漂移控制。

## 5. 来源归因（本轮状态）

### 来自 VINS 的信息

- `/iris_i/vins_estimator/imu_propagate`
- `/iris_i/vins_estimator/odometry`
- bridge 输出 `/iris_i/odometry`

### 不来自 VINS 的信息

- 深度图：`/iris_i/realsense/.../depth/image_raw`
- 规划地图与轨迹：EGO planner 相关 topic
- 控制执行：px4ctrl + MAVROS + PX4
- 真值对照：Gazebo model states（仅评估）

## 6. 对 Phase-4 的建议输入

1. 将 `iris_1` 发散列为 Phase-4 首要故障：优先追查 VINS 输入与配置一致性（IMU、双目、外参、时间同步）。
2. 保留双模式：
   - `vins_only` 用于真实性能评估；
   - `vins_with_fallback` 用于工程稳态演示。
3. 将本报告 JSON 作为基线，后续每次改动后复跑同脚本并做差分比较。

## 7. 附件路径

- 量化脚本：`tools/phase3_health_quant.py`
- 60s 报告：`logs/phase3_health_report_vins_only_60s.json`
- 45s 报告：`logs/phase3_health_report_vins_only_45s.json`
- 本文档：`docs/PHASE3_HEALTH_REPORT_20260315_CN.md`
