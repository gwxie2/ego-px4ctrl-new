# Phase-3：VINS 替换真值里程计（双机）上下文说明

## 1. 目标与边界

Phase-3 的目标是：

1. 将 `uav0/uav1` 的规划与控制里程计输入，从 `/uav*/truth_odom` 替换为 VINS 来源；
2. 保持 EGO 主链不变（`planning/bspline`、`position_cmd`、`broadcast_bspline`）；
3. 保持 PX4 控制链不变（`px4ctrl` 仍消费同一 `position_cmd`）；
4. 改动只在 `cleanroom_ws` 内进行。

新增入口：

- `src/clean_uav_core/launch/phase3_dual_uav_vins_stack.launch`

该入口基于 `phase2_dual_uav_stack.launch`，仅切换里程计/位姿输入，不改规划器和控制器实现。

---

## 2. 启动顺序（固定：最多 3 个 launch）

### 终端 A：仿真与 MAVROS

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase2_px4_multi_sim.launch gui:=false
```

### 终端 B：VINS 估计器 + bridge（合并）

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase3_vins_pipeline.launch
```

该 launch 会同时执行：

1. 启动 `xtdrone_run_vio.launch`（双机 VINS 估计器）；
2. 启动 `multi_vins_bridge.py`，统一发布：
	- `/iris_0/odometry`
	- `/iris_1/odometry`
	- `/iris_0/mavros/vision_pose/pose`
	- `/iris_1/mavros/vision_pose/pose`

其中 bridge 默认启用回退：

- 优先转发 `/iris_i/vins_estimator/odometry`
- 若 VINS 尚未稳定（超时），临时回退 `/iris_i/mavros/local_position/odom`

这样可避免 `No odom` 导致起飞链被拒绝，同时不改变 Phase-3 的主接口约定。

### 终端 C：Phase-3 算法与控制栈（cleanroom）

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase3_dual_uav_vins_stack.launch
```

---

## 3. 坐标系与转换要点

1. EGO 与 cleanroom 默认使用 `world` 作为规划坐标系；
2. `phase3_dual_uav_vins_stack.launch` 中 `vins_world_frame` 默认是 `world`；
3. 若 VINS 输出不是 ENU/world，需要在 VINS transfer 层先完成坐标变换，再送入 cleanroom；
4. 该阶段不在 EGO/px4ctrl 内做额外坐标系魔改，避免引入隐藏耦合。

---

## 4. 关键 remap/topic 对照

## 4.1 Phase-2 基础入口新增可切换参数

`phase2_dual_uav_stack.launch` 现已支持：

- `use_truth_odom`：是否启用 `truth_odom_adapter`
- `uav0_odom_topic` / `uav1_odom_topic`
- `uav0_pose_topic` / `uav1_pose_topic`

因此可以在不改 EGO 代码的前提下替换 odom/pose 输入。

## 4.2 Phase-3 默认配置（`phase3_dual_uav_vins_stack.launch`）

- `uav0_vins_odom_topic` 默认 `/iris_0/odometry`
- `uav1_vins_odom_topic` 默认 `/iris_1/odometry`
- `uav0_vins_pose_topic` 默认 `/iris_0/mavros/local_position/pose`
- `uav1_vins_pose_topic` 默认 `/iris_1/mavros/local_position/pose`
- `bridge_pose_from_odom` 默认 `false`（仅在你确实需要 Odometry->PoseStamped 时开启）
- `planner_fail_safe` 默认 `false`（降低仿真中深度偶发丢失触发 EMERGENCY_STOP 的概率）

当 `bridge_pose_from_odom:=true` 时，会启动两个 `odom_pose_adapter.py`：

- 把 VINS `Odometry` 转成 `PoseStamped` 给 EGO 的 `grid_map/pose`
- 不依赖外部是否单独提供 `PoseStamped`

---

## 5. VINS 初始化与健康检查

建议 Phase-3 起飞前确认：

1. odom 已稳定更新（至少 10Hz+，且位置非零）；
2. frame_id 一致（建议统一 `world`）；
3. 双机都已收到 odom（`dual_traj_start_trigger` 不再等待）；
4. MAVROS vision 输入正常（若用于 PX4 EKF 融合）；
5. 再触发起飞与轨迹执行，避免“起飞后瞬间漂移”。

建议最小检查命令：

```bash
rostopic hz /iris_0/odometry
rostopic hz /iris_1/odometry
rostopic echo -n 1 /uav0/vins_pose/header
rostopic echo -n 1 /uav1/vins_pose/header
```

---

## 6. 为什么 EGO 不受影响

因为本阶段仅替换输入数据源：

- 保持了 `ego_planner_node`、`traj_server`、`px4ctrl` 二进制和参数主链；
- 保持了多机广播总线 `planning_broadcast_topic=/swarm/broadcast_bspline`；
- 保持了输出控制话题 `position_cmd` 及 PX4 控制路径。

换言之，EGO 的“规划机制”不变，只是“里程计观测来源”从真值切换为 VINS。

---

## 7. 最新联调结论（2026-03）

### 7.1 已确认问题与修复

1. **TF_REPEATED_DATA 根因**：双机 VINS 进程同时发布同名 TF（`world->body`、`body->camera`）。
	- 修复：VINS TF frame 增加按命名空间唯一前缀（例如 `iris_0_body`、`iris_1_body`）。
2. **Depth Lost 误触发**：`grid_map` 在“越界但仍有同步输入”场景下，`last_occ_update_time_` 未刷新，触发超时急停。
	- 修复：在 `depthPoseCallback/depthOdomCallback` 收到有效同步输入时刷新 `last_occ_update_time_`。
3. **规划 in-obstacle 主因之一**：odom 与 pose 来源不一致导致坐标错配。
	- 修复：Phase-3 默认启用 `odom_pose_adapter`，统一使用 bridge 输出 odom 生成 planner pose。

### 7.2 当前状态

- 规划主链已恢复可运行（可连续 `plan_success=1`）。
- 主要剩余风险集中在 **VINS 估计质量**（频率、漂移、短时稳定性），而非规划结构本身。

---

## 8. 两种运行模式（显式区分）

### 8.1 `vins_only`（评估模式）

- 含义：仅使用 VINS 作为 odom 主源，不启用 MAVROS 回退。
- 用途：量化 VINS 真实健康度（频率、漂移、稳定性）。

```bash
roslaunch clean_uav_core phase3_vins_pipeline.launch odom_mode:=vins_only
roslaunch clean_uav_core phase3_dual_uav_vins_stack.launch odom_mode:=vins_only
```

### 8.2 `vins_with_fallback`（工程稳态模式）

- 含义：优先 VINS；VINS 超时后短时回退 `mavros/local_position/odom`。
- 用途：演示/持续运行优先，容忍估计短时抖动。

```bash
roslaunch clean_uav_core phase3_vins_pipeline.launch odom_mode:=vins_with_fallback
roslaunch clean_uav_core phase3_dual_uav_vins_stack.launch odom_mode:=vins_with_fallback
```

注意：`vins_with_fallback` 提升可运行性，但会降低“纯 VINS 评估”真实性；做指标报告时应使用 `vins_only`。

---

## 9. 与 Phase-2 / Phase-3 完成态的关系

- 相比 Phase-2（truth odom）：已完成“VINS 接管里程计入口 + 规划控制链打通”。
- 距离 Phase-3 完成态：仍需完成稳定性量化闭环（健康度脚本、漂移基线、长程压测）。
