# Phase 1 Dependency Inventory

## Goal
Build the smallest clean-room catkin workspace that can support:

`Gazebo/PX4/MAVROS + ground-truth odom + EGO (single UAV first) + px4ctrl`

## Confirmed core packages

### Control side
- `px4ctrl`
  - depends on: `roscpp`, `rospy`, `geometry_msgs`, `sensor_msgs`, `mavros`, `quadrotor_msgs`, `uav_utils`, `Eigen3`

### EGO side
- `ego_planner` (package name in `plan_manage`)
  - depends on: `plan_env`, `path_searching`, `bspline_opt`, `traj_utils`, `quadrotor_msgs`, `cv_bridge`, `PCL`, `geometry_msgs`, `std_msgs`

### Utility/message side
- `quadrotor_msgs`
- `uav_utils`
- `cmake_utils`

## Likely minimum EGO package set for phase 1
These packages are expected to be needed in the workspace for single-UAV EGO compilation:
- `motion_planning/3d/ego_planner/plan_manage`
- `motion_planning/3d/ego_planner/plan_env`
- `motion_planning/3d/ego_planner/path_searching`
- `motion_planning/3d/ego_planner/bspline_opt`
- `motion_planning/3d/ego_planner/traj_utils`
- `motion_planning/3d/ego_planner/Utils/quadrotor_msgs`
- `motion_planning/3d/ego_planner/Utils/uav_utils`
- `motion_planning/3d/ego_planner/Utils/cmake_utils`

## Explicitly deferred in phase 1
- XTDrone `communication/*.py`
- XTDrone keyboard controllers
- VINS-Fusion
- `vins_transfer.py` / `multi_vins_transfer.py`
- swarm bridge / TCP bridge logic
- XTDrone-specific EGO remap launch as the main path

## Working decision
Prefer official EGO launch/layout as the main integration path. XTDrone EGO launch files are reference material for remap and topic naming only.

## Next checks
1. Verify `catkin_make` prerequisites exist in this environment.
2. Create workspace `src/CMakeLists.txt` catkin entry.
3. Decide whether phase 1 imports packages by symlink or by clean copy.
4. Build the first package layout inside `cleanroom_ws/src`.
