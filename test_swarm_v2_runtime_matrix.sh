#!/bin/bash

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$ROOT_DIR"

run_case() {
  local title="$1"
  shift

  echo ""
  echo "=========================================="
  echo "$title"
  echo "=========================================="
  ./test_swarm_v2_runtime_health.sh "$@"
}

echo "=========================================="
echo "Swarm V2 Runtime Matrix"
echo "=========================================="

run_case "[1/5] Default V2 runtime" \

run_case "[2/5] Goal tooling runtime" \
  --expect-node /assign_goals_v2 \
  -- enable_goal_tooling_v2:=true

run_case "[3/5] Moving obstacles runtime" \
  --expect-node /moving_obstacles_v2 \
  -- enable_moving_obstacles_v2:=true

run_case "[4/5] Auxiliary support runtime" \
  --expect-node /manual_take_over_station \
  --expect-node /drone_0/manual_take_over_v2 \
  --expect-node /drone_0/odom_visualization_v2 \
  --expect-topic /drone_0/odom_visualization_v2/robot \
  -- enable_manual_take_over_v2:=true enable_manual_take_over_station_v2:=true enable_manual_take_over_joy_node_v2:=false enable_odom_visualization_v2:=true

run_case "[5/5] Full software-only optional runtime" \
  --expect-node /assign_goals_v2 \
  --expect-node /moving_obstacles_v2 \
  --expect-node /manual_take_over_station \
  --expect-node /drone_0/manual_take_over_v2 \
  --expect-node /drone_0/odom_visualization_v2 \
  --expect-topic /drone_0/odom_visualization_v2/robot \
  -- enable_goal_tooling_v2:=true enable_moving_obstacles_v2:=true enable_manual_take_over_v2:=true enable_manual_take_over_station_v2:=true enable_manual_take_over_joy_node_v2:=false enable_odom_visualization_v2:=true

echo ""
echo "=========================================="
echo "V2 runtime matrix passed"
echo "=========================================="