# Phase4 动态障碍场景双机闭环执行说明

## 1. 目标

在 `vins_ego_forest_stage3.world` 动态障碍环境中，双机从以下起点起飞并完成规划闭环：

- `iris_0` 起点：`(0, 1.5, 0.1)`
- `iris_1` 起点：`(0, -1.5, 0.1)`

动态目标规则：

- `uav0`：`x=18`，`y` 在 `[2.5, 3.5]` 内周期摆动（默认 6s）
- `uav1`：`x=18`，`y` 在 `[-3.5, -2.5]` 内周期摆动（默认 6s）

主启动文件：

- `clean_uav_core/launch/phase4_dual_uav_dynamic_goal_stack.launch`

---

## 2. 构建

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
catkin_make --pkg clean_uav_core
```

---

## 3. 运行方式

### 3.1 启用 VINS（enable_vins=true）

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase4_dual_uav_dynamic_goal_stack.launch \
  start_sim:=true \
  enable_vins:=true \
  odom_mode:=vins_only \
  gui:=false
```

### 3.2 禁用 VINS（enable_vins=false，走 MAVROS local_position）

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase4_dual_uav_dynamic_goal_stack.launch \
  start_sim:=true \
  enable_vins:=false \
  gui:=false
```

> 说明：`enable_vins=false` 时，规划与控制输入自动切换为 `/iris_i/mavros/local_position/odom` + `/iris_i/mavros/local_position/pose`。

---

## 4. 常用可调参数

- `goal_period_sec`：动态目标周期（默认 6.0）
- `goal_publish_rate`：动态目标发布频率（默认 6.0 Hz）
- `goal_x`：目标 X（默认 18.0）
- `uav0_goal_y_center/uav0_goal_y_amp`：uav0 目标摆动中心与振幅
- `uav1_goal_y_center/uav1_goal_y_amp`：uav1 目标摆动中心与振幅
- `planner_max_vel/planner_max_acc/planner_max_jerk`：规划约束
- `grid_map_odom_depth_timeout/grid_map_depth_filter_mindist`：深度/里程计融合鲁棒性参数

---

## 5. 验收脚本

验收脚本：`tools/phase4_acceptance_quant.py`

### 5.1 VINS 模式验收

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
python3 tools/phase4_acceptance_quant.py \
  --duration 90 \
  --uav0_odom_topic /iris_0/odometry \
  --uav1_odom_topic /iris_1/odometry \
  --output logs/phase4_acceptance_vins.json \
  --md_output logs/phase4_acceptance_vins.md
```

### 5.2 MAVROS 模式验收

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
python3 tools/phase4_acceptance_quant.py \
  --duration 90 \
  --uav0_odom_topic /iris_0/mavros/local_position/odom \
  --uav1_odom_topic /iris_1/mavros/local_position/odom \
  --output logs/phase4_acceptance_mavros.json \
  --md_output logs/phase4_acceptance_mavros.md
```

验收核心判据：

1. 双机均达到 `x >= 18 - x_reach_tol`
2. 机间最小间距始终大于 `min_separation`
3. 无坠落代理（高度不低于 `min_altitude`）
4. 轨迹连续（无超阈值位置跳变）
5. MAVROS 状态满足 Connected + OFFBOARD + ARMED

---

## 6. 快速排障

- 若动态目标未生效：检查 `/uav0/goal`、`/uav1/goal` 是否有持续发布。
- 若规划无响应：检查 `planner_flight_type` 是否为 `1`（手动目标模式）。
- 若 OFFBOARD 未进：检查 `/iris_i/mavros/state` 与 `px4ctrl` 触发链路。
- 若 VINS 模式不稳定：先切 `enable_vins=false` 验证“非 VINS 路径”闭环，再分离定位问题。