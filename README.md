# cleanroom_ws

`cleanroom_ws` 是 XTDrone 中用于 EGO / EGO-Swarm + px4ctrl clean-room 迭代的独立 catkin 工作空间。

---

## 1. 目录说明

- `src/`：ROS package 与核心代码
- `docs/`：阶段计划、架构与启动指南
- `tools/`：环境引导脚本
- `logs/`：低频调试产物（通常不纳入版本控制）
- `build/`、`devel/`：catkin 编译产物

---

## 2. 环境准备

### 2.1 前置条件

建议环境：

- Ubuntu + ROS Noetic
- PX4 SITL + Gazebo + MAVROS 可用
- 当前仓库路径：`/home/guanwen/XTDrone/cleanroom_ws`

### 2.2 每个终端都先执行

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
```

该脚本会补齐 `ROS_PACKAGE_PATH`，确保能解析：

- `px4`
- `mavlink_sitl_gazebo`
- 本 clean-room 工作空间

### 2.3 编译

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
catkin_make
```

仅重编核心包可用：

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
catkin_make --pkg clean_uav_core
```

---

## 3. Phase 1 使用教程（单机）

### 3.1 推荐：三终端分层启动

终端 A（仿真层）：

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase1_px4_sim.launch gui:=false interactive:=false
```

终端 B（算法层）：

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase1_algo_stack.launch
```

终端 C（控制层）：

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase1_px4ctrl_stack.launch use_takeoff_trigger:=true takeoff_delay:=8.0
```

### 3.2 一键包装入口

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase1_fullstack.launch use_takeoff_trigger:=true takeoff_delay:=8.0 interactive:=false gui:=false verbose:=false
```

### 3.3 手动目标模式（替代 RViz 点击）

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase1_manual_goal_stack.launch target_x:=5.0 target_y:=0.0 target_z:=1.0 goal_delay:=2.0
```

---

## 4. Phase 2 使用教程（双机）

### 4.1 启动双机仿真（终端 A）

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase2_px4_multi_sim.launch gui:=false vehicle:=iris vehicle_num:=2
```

如需 Gazebo 可视化窗口，显式改为：

```bash
roslaunch clean_uav_core phase2_px4_multi_sim.launch gui:=true vehicle:=iris vehicle_num:=2
```

### 4.2 启动双机算法与控制（终端 B）

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase2_dual_uav_stack.launch
```

关于“先起飞再规划”的状态机时序、默认时延、以及固定目标点参数位置，请直接参考：

- [docs/phase2_runtime_log_reading_guide_CN.md](docs/phase2_runtime_log_reading_guide_CN.md)

### 4.3 验证项（最小闭环）

```bash
source tools/source_phase1_env.sh
rostopic echo -n 1 /iris_0/mavros/state
rostopic echo -n 1 /iris_1/mavros/state
rostopic echo -n 1 /uav0/mission_status
rostopic echo -n 1 /uav1/mission_status
```

期望状态：

- 两机可进入 `OFFBOARD + armed`
- 任务状态可收敛到 `phase1_mission: mission completed`

---

## 5. 常见问题与排障

1. 话题无数据：确认每个终端都已 `source tools/source_phase1_env.sh`。
2. 双机不动作：优先检查 `/iris_0/mavros/state`、`/iris_1/mavros/state`。
3. 任务状态不收敛：查看 `/tmp/phase2_dual_uav_stack.log` 中 `mission_progress_monitor` 与 `dual_traj_start_trigger` 关键字。
4. 运行残留冲突：先停止旧 launch，再重启仿真层与算法层。

---

## 6. Phase 3 固定执行方法（最多 3 个 launch）

目标：VINS 替换真值里程计，同时保持 EGO 与 px4ctrl 主链不变。

### 6.1 终端 A：双机仿真 + MAVROS

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase2_px4_multi_sim.launch gui:=true
```

### 6.2 终端 B：VINS 估计器 + bridge（合并为一个 launch）

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase3_vins_pipeline.launch
```

模式切换（建议显式传参）：

- 纯 VINS 评估模式（不回退）：

```bash
roslaunch clean_uav_core phase3_vins_pipeline.launch odom_mode:=vins_only
```

- 工程稳态模式（VINS 异常时回退到 MAVROS local odom）：

```bash
roslaunch clean_uav_core phase3_vins_pipeline.launch odom_mode:=vins_with_fallback
```

该命令会：

- 启动 `xtdrone_run_vio.launch`（双机 VINS 估计器）
- 启动 `multi_vins_bridge.py`，输出：
	- `/iris_0/odometry`
	- `/iris_1/odometry`
	- `/iris_0/mavros/vision_pose/pose`
	- `/iris_1/mavros/vision_pose/pose`

说明：Phase-3 默认使用 `/iris_i/mavros/local_position/pose` 作为 planner 的 `camera_pose` 输入；
`odom_pose_adapter.py` 仅作为可选兜底（`bridge_pose_from_odom:=true` 时启用）。
为避免仿真深度流瞬断导致 EGO 进入紧急停机，Phase-3 默认 `planner_fail_safe:=false`。

### 6.3 终端 C：Phase-3 规划与控制栈

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase3_dual_uav_vins_stack.launch
```

说明：Phase-3 默认使用 bridge 输出的 `/iris_i/odometry` 通过 `odom_pose_adapter.py` 生成 `/iris_i/bridge_pose`，
用于 planner 的 `camera_pose` 输入，避免 odom 与 pose 来自不同链路导致坐标不一致。

### 6.4 最小健康检查

```bash
source tools/source_phase1_env.sh
rostopic hz /iris_0/odometry
rostopic hz /iris_1/odometry
rostopic echo -n 1 /uav0/mission_status
rostopic echo -n 1 /uav1/mission_status
```

期望：

- odom 频率稳定（建议 >=10Hz）
- 双机可进入 `OFFBOARD + armed`
- 任务状态可收敛到 `mission completed`

---

## 7. 文档总索引（含全部其它 Markdown）

> 下列链接覆盖当前 `cleanroom_ws` 内除本 README 以外的其它 Markdown 文档。

### 7.1 阶段与架构文档

- [docs/CLEANROOM_PHASE2_MULTI_UAV_PLAN_CN.md](docs/CLEANROOM_PHASE2_MULTI_UAV_PLAN_CN.md)
- [docs/PHASE3_VINS_CONTEXT_CN.md](docs/PHASE3_VINS_CONTEXT_CN.md)
- [docs/phase2_architecture_and_principles_CN.md](docs/phase2_architecture_and_principles_CN.md)
- [docs/phase1_dependency_inventory.md](docs/phase1_dependency_inventory.md)
- [docs/phase1_runtime_chain_design.md](docs/phase1_runtime_chain_design.md)
- [docs/phase1_split_launch_guide.md](docs/phase1_split_launch_guide.md)
- [docs/phase2_runtime_log_reading_guide_CN.md](docs/phase2_runtime_log_reading_guide_CN.md)

### 7.2 clean_uav_core 文档

- [src/clean_uav_core/README.md](src/clean_uav_core/README.md)

### 7.3 ego_planner 相关导读

- [src/ego_planner/launch/multi_uav.launch_guide.md](src/ego_planner/launch/multi_uav.launch_guide.md)
- [src/ego_planner/launch/rviz.launch_guide.md](src/ego_planner/launch/rviz.launch_guide.md)
- [src/ego_planner/launch/run_in_sim.launch_guide.md](src/ego_planner/launch/run_in_sim.launch_guide.md)
- [src/ego_planner/launch/run_in_xtdrone.launch_guide.md](src/ego_planner/launch/run_in_xtdrone.launch_guide.md)
- [src/ego_planner/launch/simple_run.launch_guide.md](src/ego_planner/launch/simple_run.launch_guide.md)
- [src/ego_planner/launch/single_uav.launch_guide.md](src/ego_planner/launch/single_uav.launch_guide.md)
- [src/ego_planner/launch/swarm.launch_guide.md](src/ego_planner/launch/swarm.launch_guide.md)
- [src/ego_planner/launch/swarm_large.launch_guide.md](src/ego_planner/launch/swarm_large.launch_guide.md)

### 7.4 px4ctrl 相关导读

- [src/px4ctrl/launch/run_ctrl.launch_guide.md](src/px4ctrl/launch/run_ctrl.launch_guide.md)
- [src/px4ctrl/launch/thrust_calibrate.launch_guide.md](src/px4ctrl/launch/thrust_calibrate.launch_guide.md)
- [src/px4ctrl/src/PX4CtrlFSM.cpp_guide.md](src/px4ctrl/src/PX4CtrlFSM.cpp_guide.md)
- [src/px4ctrl/src/PX4CtrlParam.cpp_guide.md](src/px4ctrl/src/PX4CtrlParam.cpp_guide.md)
- [src/px4ctrl/src/controller.cpp_guide.md](src/px4ctrl/src/controller.cpp_guide.md)
- [src/px4ctrl/src/input.cpp_guide.md](src/px4ctrl/src/input.cpp_guide.md)
- [src/px4ctrl/src/px4ctrl_node.cpp_guide.md](src/px4ctrl/src/px4ctrl_node.cpp_guide.md)
- [src/px4ctrl/thrust_calibrate_scrips/thrust_calibrate.py_guide.md](src/px4ctrl/thrust_calibrate_scrips/thrust_calibrate.py_guide.md)

### 7.5 quadrotor_msgs 相关导读

- [src/quadrotor_msgs/src/quadrotor_msgs/msg/_AuxCommand.py_guide.md](src/quadrotor_msgs/src/quadrotor_msgs/msg/_AuxCommand.py_guide.md)
- [src/quadrotor_msgs/src/quadrotor_msgs/msg/_Corrections.py_guide.md](src/quadrotor_msgs/src/quadrotor_msgs/msg/_Corrections.py_guide.md)
- [src/quadrotor_msgs/src/quadrotor_msgs/msg/_Gains.py_guide.md](src/quadrotor_msgs/src/quadrotor_msgs/msg/_Gains.py_guide.md)
- [src/quadrotor_msgs/src/quadrotor_msgs/msg/_OutputData.py_guide.md](src/quadrotor_msgs/src/quadrotor_msgs/msg/_OutputData.py_guide.md)
- [src/quadrotor_msgs/src/quadrotor_msgs/msg/_PositionCommand.py_guide.md](src/quadrotor_msgs/src/quadrotor_msgs/msg/_PositionCommand.py_guide.md)
- [src/quadrotor_msgs/src/quadrotor_msgs/msg/_PPROutputData.py_guide.md](src/quadrotor_msgs/src/quadrotor_msgs/msg/_PPROutputData.py_guide.md)
- [src/quadrotor_msgs/src/quadrotor_msgs/msg/_SO3Command.py_guide.md](src/quadrotor_msgs/src/quadrotor_msgs/msg/_SO3Command.py_guide.md)
- [src/quadrotor_msgs/src/quadrotor_msgs/msg/_Serial.py_guide.md](src/quadrotor_msgs/src/quadrotor_msgs/msg/_Serial.py_guide.md)
- [src/quadrotor_msgs/src/quadrotor_msgs/msg/_StatusData.py_guide.md](src/quadrotor_msgs/src/quadrotor_msgs/msg/_StatusData.py_guide.md)
- [src/quadrotor_msgs/src/quadrotor_msgs/msg/_TRPYCommand.py_guide.md](src/quadrotor_msgs/src/quadrotor_msgs/msg/_TRPYCommand.py_guide.md)

### 7.6 uav_utils 相关导读

- [src/uav_utils/scripts/odom_to_euler.py_guide.md](src/uav_utils/scripts/odom_to_euler.py_guide.md)
- [src/uav_utils/scripts/send_odom.py_guide.md](src/uav_utils/scripts/send_odom.py_guide.md)
- [src/uav_utils/scripts/tf_assist.py_guide.md](src/uav_utils/scripts/tf_assist.py_guide.md)
- [src/uav_utils/scripts/topic_statistics.py_guide.md](src/uav_utils/scripts/topic_statistics.py_guide.md)

### 7.7 日志与阶段记录

- [logs/README.md](logs/README.md)
- [logs/phase1_implementation_log.md](logs/phase1_implementation_log.md)

