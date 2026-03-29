# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the **ego-px4ctrl** workspace - a ROS Noetic catkin workspace containing a clean-room integration of EGO-Planner (3D trajectory planning) with px4ctrl (high-level flight controller) for PX4-based multi-rotor drones. It serves as the Phase 1-4 development workspace for autonomous UAV navigation in simulation and potentially real-world deployment.

**Workspace Path**: `/home/guanwen/XTDrone/ego-px4ctrl`

## Build Commands

### Environment Setup (Required for every new terminal)

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl
source tools/source_phase1_env.sh
```

This script sets up `ROS_PACKAGE_PATH` to include:
- PX4 firmware (`~/XTDrone/PX4_Firmware`)
- Gazebo plugins (`mavlink_sitl_gazebo`)
- This workspace

### Build Commands

```bash
# Full workspace build
cd /home/guanwen/XTDrone/ego-px4ctrl
source tools/source_phase1_env.sh
catkin_make -j4

# Build specific package only
catkin_make --pkg px4ctrl
catkin_make --pkg clean_uav_core
catkin_make --pkg ego_planner
```

### Key Build Dependencies

- **roscpp/rospy**: Core ROS
- **mavros**: PX4 communication bridge
- **quadrotor_msgs**: Custom messages (use local copy, NOT apt version)
- **uav_utils**: UAV utilities
- **Eigen3**: Linear algebra
- **PX4 Firmware**: Located at `~/XTDrone/PX4_Firmware`

## High-Level Architecture

### System Layers (Bottom to Top)

```
┌─────────────────────────────────────────────────────────────┐
│  Task/Planning Layer: EGO-Planner (path search + B-spline)  │
├─────────────────────────────────────────────────────────────┤
│  Control Layer: px4ctrl (cascaded PID + thrust model)       │
├─────────────────────────────────────────────────────────────┤
│  Glue Layer: clean_uav_core (odom bridge, triggers, launch) │
├─────────────────────────────────────────────────────────────┤
│  Middleware: MAVROS (ROS ↔ MAVLink bridge)                  │
├─────────────────────────────────────────────────────────────┤
│  Physics/Simulation: Gazebo + PX4 SITL                      │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow

```
Gazebo → truth_odom_adapter.py → /truth_odom
                                      ↓
                    ┌─────────────────┴─────────────────┐
                    ↓                                   ↓
            ego_planner_node                     px4ctrl_node
                    ↓                                   ↑
            traj_server ────────────► /position_cmd ────┘
                                                  ↓
                                         /mavros/setpoint_raw/attitude
                                                  ↓
                                                MAVROS
                                                  ↓
                                                 PX4
```

### Core Components

| Component | Location | Purpose |
|-----------|----------|---------|
| **px4ctrl** | `src/px4ctrl/` | High-level flight controller with FSM and cascaded PID |
| **ego_planner** | `src/ego_planner/` | 3D trajectory planner (B-spline optimization) |
| **clean_uav_core** | `src/clean_uav_core/` | Glue package with launch files and adapter scripts |
| **quadrotor_msgs** | `src/quadrotor_msgs/` | Custom messages (use local copy only) |
| **plan_env** | `src/plan_env/` | Grid map for obstacle representation |
| **traj_utils** | `src/traj_utils/` | B-spline utilities and trajectory server |
| **bspline_opt** | `src/bspline_opt/` | B-spline trajectory optimization |
| **path_searching** | `src/path_searching/` | A* path search for initial trajectory |

## Launch Files

### Phase 1 (Single UAV)

```bash
# Full stack (simulation + planning + control)
roslaunch clean_uav_core phase1_fullstack.launch use_takeoff_trigger:=true

# Split launch (3 terminals)
roslaunch clean_uav_core phase1_px4_sim.launch gui:=false
roslaunch clean_uav_core phase1_algo_stack.launch
roslaunch clean_uav_core phase1_px4ctrl_stack.launch use_takeoff_trigger:=true
```

### Phase 2 (Dual UAV)

```bash
# Terminal 1: Simulation layer (start first)
roslaunch clean_uav_core phase2_px4_multi_sim.launch gui:=false vehicle:=iris vehicle_num:=2

# Terminal 2: Algorithm + control layer (after sim loads)
roslaunch clean_uav_core phase2_dual_uav_stack.launch
```

### Phase 3 (VINS Integration)

```bash
# Terminal 1: Simulation
roslaunch clean_uav_core phase2_px4_multi_sim.launch gui:=true

# Terminal 2: VINS pipeline
roslaunch clean_uav_core phase3_vins_pipeline.launch odom_mode:=vins_with_fallback

# Terminal 3: Planning + control
roslaunch clean_uav_core phase3_dual_uav_vins_stack.launch
```

### Phase 4 (Dynamic Goal)

```bash
# Simulation
roslaunch clean_uav_core phase4_px4_multi_sim.launch vehicle_num:=2

# Dynamic goal stack
roslaunch clean_uav_core phase4_dual_uav_dynamic_goal_stack.launch
```

## Key ROS Topics

### Subscribed Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `~odom` | `nav_msgs/Odometry` | Odometry input (remapped to `/truth_odom` or `/iris_X/odometry`) |
| `~cmd` | `quadrotor_msgs/PositionCommand` | Position command from trajectory server |
| `/mavros/state` | `mavros_msgs/State` | PX4 connection and flight mode |
| `/mavros/imu/data` | `sensor_msgs/Imu` | IMU sensor data |
| `~takeoff_land` | `quadrotor_msgs/TakeoffLand` | Takeoff/landing trigger (no-RC mode) |
| `/traj_start_trigger` | `std_msgs/Bool` | Trajectory planning start trigger |

### Published Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | Attitude and thrust command to PX4 |
| `/traj_start_trigger` | `std_msgs/Bool` | Trajectory start signal |
| `/px4ctrl/debug` | `quadrotor_msgs/Px4ctrlDebug` | Debug information |

## px4ctrl FSM States

```
MANUAL_CTRL  →  AUTO_TAKEOFF  →  AUTO_HOVER  →  CMD_CTRL  →  AUTO_LAND
(RC only)       (Auto takeoff)   (Hover)        (Tracking)   (Auto land)
```

- **MANUAL_CTRL**: FCU controlled by RC only, px4ctrl inactive
- **AUTO_TAKEOFF**: Automatic takeoff sequence with motor idling (2.8s)
- **AUTO_HOVER**: Maintains hover while waiting for commands
- **CMD_CTRL**: Executing position commands (main operational state)
- **AUTO_LAND**: Automatic landing with touchdown detection

## Configuration Files

### px4ctrl Configuration

**Primary**: `src/px4ctrl/config/ctrl_param_fpv.yaml`

**Override**: `src/clean_uav_core/config/phase1_px4ctrl_no_rc.yaml`

**Key Parameters**:
- `mass`: Vehicle mass (default 1.2 kg)
- `hover_percentage`: Hover throttle percentage (~0.58 in simulation)
- `auto_takeoff_land.no_RC`: Enable no-RC mode
- `auto_takeoff_land.takeoff_height`: Takeoff target height
- `gain.Kp0/1/2`: Position PID gains (x/y/z)
- `gain.Kv0/1/2`: Velocity PID gains (x/y/z)
- `msg_timeout.*`: Sensor timeout thresholds

### EGO Planner Parameters

Embedded in `src/clean_uav_core/launch/phase1_minimal_demo.launch`:
- `fsm/realworld_experiment`: Enable real-world mode (waits for trigger)
- `optimization/max_vel`: Maximum velocity constraint
- `optimization/max_acc`: Maximum acceleration constraint
- `grid_map/resolution`: Grid map resolution (default 0.1m)

## Important Constraints

1. **quadrotor_msgs MUST be local copy**: The local version at `src/quadrotor_msgs/` includes custom fields (`TakeoffLand`, `Px4ctrlDebug`, `jerk`) not present in upstream EGO-Planner. Never replace with apt package.

2. **Environment sourcing is mandatory**: Always `source tools/source_phase1_env.sh` before any catkin operations or roslaunch. This sets up PX4 package paths.

3. **Multi-UAV startup order**: For Phase 2+, always start simulation layer first, wait for Gazebo to fully load (10-15s), then start algorithm/control layer.

4. **Topic namespace**: Multi-UAV scenarios use `/uavN/` namespaces for algorithm topics and `/irisN/mavros/` for MAVROS topics.

## Python Scripts

Located in `src/clean_uav_core/scripts/`:

- `truth_odom_adapter.py`: Bridges Gazebo model states to ROS odometry
- `takeoff_land_trigger.py`: Triggers automatic takeoff/landing
- `traj_start_trigger.py`: Triggers trajectory planning
- `px4_param_bootstrap.py`: Sets PX4 parameters (COM_RCL_EXCEPT=4 for no-RC)
- `multi_traj_start_trigger.py`: Synchronized multi-UAV trajectory triggering
- `mission_progress_monitor.py`: Monitors mission progress

All Python scripts use `#!/usr/bin/env python3` and log with `[clean_uav_core]` prefix.

## Testing Commands

### Check MAVROS Connection

```bash
rostopic echo /mavros/state -n 1
# Should show: connected: true, armed: true/false, mode: "OFFBOARD" or similar
```

### Check Odometry

```bash
rostopic hz /truth_odom  # Should be ~50-100 Hz
rostopic echo /truth_odom/pose/pose/position -n 1
```

### Check Position Commands

```bash
rostopic echo /position_cmd -n 1
```

### View Node Graph

```bash
rosrun rqt_graph rqt_graph
```

## Common Issues

1. **catkin_make fails**: Ensure `source tools/source_phase1_env.sh` is run first
2. **TakeoffLand message not found**: Verify local `quadrotor_msgs` is being used (not apt version)
3. **UAV doesn't take off**: Check `/mavros/state`, `/truth_odom`, and `/px4ctrl/takeoff_land` topics
4. **UAV takes off but doesn't move to target**: Check `/traj_start_trigger` and `/position_cmd` topics
5. **Multi-UAV not starting**: Ensure simulation layer is fully loaded before starting algorithm layer

## Reference Documentation

- `docs/beginner_guide_CN.md`: Comprehensive beginner guide (Chinese)
- `docs/system_architecture_CN.md`: Detailed system architecture
- `docs/phase1_px4ctrl_config_explanation_CN.md`: Parameter configuration guide
- `docs/phase2_architecture_and_principles_CN.md`: Phase 2 multi-UAV architecture
