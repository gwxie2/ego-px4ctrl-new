# Phase 1 Runtime Chain Design

## Target chain

`Gazebo/PX4/MAVROS + truth odom adapter + EGO(single UAV) + px4ctrl`

## Phase-1 runtime objective
Obtain the first runnable single-UAV clean-room demo without:
- XTDrone communication scripts
- XTDrone keyboard control scripts
- VINS
- swarm bridge logic

## Planned runtime components

### 1. Backend simulation side
- PX4 SITL
- Gazebo world (indoor first)
- MAVROS bridge

### 2. Planning side
- `ego_planner_node`
- `traj_server`
- a simple goal source (fixed goal or RViz goal)

### 3. Control side
- `px4ctrl_node`
- no-RC mode for phase-1 automated path

### 4. Glue side (`clean_uav_core`)
- truth odom adapter
- later: commander / mode API
- later: launch glue and remap unification

## First implementation sub-steps
1. Identify the most convenient truth odom source in Gazebo/PX4 stack.
2. Define one clean odom topic for both EGO and px4ctrl consumption.
3. Decide whether phase-1 goal enters via fixed waypoint or `/move_base_simple/goal`.
4. Build a single launch entry in `clean_uav_core/launch`.
5. Verify topic graph before any flight attempt.

## Non-goals in this sub-step
- VINS replacement
- swarm startup
- elegant fail-safe APIs
- full commander node implementation

## Success condition
A single launch sequence starts the minimal chain and reaches the point where the only remaining work is runtime adapter logic, not workspace/package/build repair.

## Confirmed design decisions

### Why not use official `run_in_sim.launch` directly
- It also launches EGO's internal simulator/sensing stack.
- That path depends on packages such as `so3_quadrotor_simulator`, `so3_control`, `odom_visualization`, and `local_sensing_node`, which are outside the current phase-1 minimal imported workspace.
- Therefore phase 1 should reuse the official node contracts, but provide a clean-room launch entry assembled in `clean_uav_core`.

### Confirmed EGO startup gating
- `ego_planner_node` waits for odom.
- In preset-target mode it may also wait for `/traj_start_trigger` when `fsm/realworld_experiment=true`.
- Missing cloud/depth does not hard-stop node startup; the grid map can remain effectively empty.

### Phase-1 unified topic contract
- Truth odom topic: `/iris_0/truth_odom`
- Truth pose topic: `/iris_0/truth_pose`
- Planner output topic: `/position_cmd`
- Takeoff trigger topic: `/px4ctrl/takeoff_land`
- PX4 attitude output remains MAVROS native:
	- `/mavros/setpoint_raw/attitude`

### Immediate runtime implication
The next runtime risk is no longer message/build compatibility.
The next runtime risk is backend launch compatibility and topic availability from `PX4 + Gazebo + MAVROS`.
