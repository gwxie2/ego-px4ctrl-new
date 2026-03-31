#!/bin/bash

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$ROOT_DIR"

source tools/source_phase1_env.sh

echo "=========================================="
echo "Swarm V2 Smoke Test"
echo "=========================================="

echo "[1/5] Generate V2 top-level launch"
/usr/bin/python3 src/clean_uav_core/scripts/swarm_launch_generator.py --version v2 >/dev/null
test -f src/clean_uav_core/launch/swarm_top_level_v2.launch

echo "[2/5] Validate base V2 node graph"
BASE_NODES=$(roslaunch --nodes clean_uav_core swarm_top_level_v2.launch)
echo "$BASE_NODES" | grep -q "/drone_0/ego_planner_v2"
echo "$BASE_NODES" | grep -q "/drone_0/traj_server_v2"
echo "$BASE_NODES" | grep -q "/swarm_dynamic_commander_v2"

echo "[3/5] Validate optional GoalSet tooling"
TOOL_NODES=$(roslaunch --nodes clean_uav_core swarm_top_level_v2.launch enable_goal_tooling_v2:=true)
echo "$TOOL_NODES" | grep -q "/assign_goals_v2"

echo "[4/5] Validate optional moving obstacles"
OBS_NODES=$(roslaunch --nodes clean_uav_core swarm_top_level_v2.launch enable_moving_obstacles_v2:=true)
echo "$OBS_NODES" | grep -q "/moving_obstacles_v2"

echo "[5/5] Validate optional manual takeover and visualization"
AUX_NODES=$(roslaunch --nodes clean_uav_core swarm_top_level_v2.launch enable_manual_take_over_v2:=true enable_manual_take_over_station_v2:=true enable_odom_visualization_v2:=true)
echo "$AUX_NODES" | grep -q "/drone_0/manual_take_over_v2"
echo "$AUX_NODES" | grep -q "/drone_0/odom_visualization_v2"
echo "$AUX_NODES" | grep -q "/manual_take_over_station"

echo ""
echo "=========================================="
echo "V2 smoke test passed"
echo "=========================================="