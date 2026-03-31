#!/bin/bash

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$ROOT_DIR"

source tools/source_phase1_env.sh

declare -a EXPECTED_NODES=()
declare -a EXPECTED_TOPICS=()
declare -a LAUNCH_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --expect-node)
      EXPECTED_NODES+=("$2")
      shift 2
      ;;
    --expect-topic)
      EXPECTED_TOPICS+=("$2")
      shift 2
      ;;
    --)
      shift
      while [[ $# -gt 0 ]]; do
        LAUNCH_ARGS+=("$1")
        shift
      done
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

LAUNCH_LOG=$(mktemp /tmp/swarm_v2_runtime_health.XXXXXX.log)
LAUNCH_PID=""

cleanup() {
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
    sleep 5
    kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}

print_failure_context() {
  echo ""
  echo "[runtime-health] launch log tail: $LAUNCH_LOG"
  tail -n 80 "$LAUNCH_LOG" || true
}

fail() {
  echo "[runtime-health] ERROR: $1" >&2
  print_failure_context
  exit 1
}

wait_for_master() {
  local timeout_s="$1"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if rosparam list >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_node() {
  local node_name="$1"
  local timeout_s="$2"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if rosnode list 2>/dev/null | grep -Fxq "$node_name"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_topic_message() {
  local topic_name="$1"
  local timeout_s="$2"
  timeout "${timeout_s}s" rostopic echo -n1 "$topic_name" >/dev/null 2>&1
}

wait_for_topic_field() {
  local topic_name="$1"
  local field_regex="$2"
  local timeout_s="$3"
  local deadline=$((SECONDS + timeout_s))
  local output

  while (( SECONDS < deadline )); do
    if output=$(timeout 3s rostopic echo -n1 "$topic_name" 2>/dev/null); then
      if printf '%s\n' "$output" | grep -Eiq "$field_regex"; then
        return 0
      fi
    fi
    sleep 1
  done
  return 1
}

echo "=========================================="
echo "Swarm V2 Runtime Health Check"
echo "=========================================="

echo "[1/6] Generate V2 top-level launch"
/usr/bin/python3 src/clean_uav_core/scripts/swarm_launch_generator.py --version v2 >/dev/null
test -f src/clean_uav_core/launch/swarm_top_level_v2.launch

echo "[2/6] Launch headless V2 stack"
setsid roslaunch clean_uav_core swarm_top_level_v2.launch \
  gui:=false \
  use_rviz:=false \
  enable_oscillation:=false \
  takeoff_delay:=5.0 \
  goal_start_delay:=10.0 \
  "${LAUNCH_ARGS[@]}" \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
trap cleanup EXIT

wait_for_master 30 || fail "ROS master did not become available"
wait_for_node "/gazebo" 40 || fail "Gazebo node did not start"
wait_for_node "/swarm_dynamic_commander_v2" 20 || fail "V2 goal commander did not start"

echo "[3/6] Validate base telemetry and MAVROS connectivity"
wait_for_topic_message "/goal_with_id" 20 || fail "No GoalSet messages on /goal_with_id"

for drone_id in 0 1 2; do
  wait_for_node "/drone_${drone_id}/px4ctrl" 40 || fail "px4ctrl missing for drone_${drone_id}"
  wait_for_topic_message "/drone_${drone_id}/odom" 25 || fail "No odom for drone_${drone_id}"
  wait_for_topic_field "/iris_${drone_id}/mavros/state" "connected:[[:space:]]*(true|True)" 35 || \
    fail "MAVROS never connected for iris_${drone_id}"
done

echo "[4/6] Validate takeoff and planner bring-up"
for drone_id in 0 1 2; do
  wait_for_topic_field "/iris_${drone_id}/mavros/state" "armed:[[:space:]]*(true|True)" 45 || \
    fail "Vehicle iris_${drone_id} never armed"
  wait_for_node "/drone_${drone_id}/ego_planner_v2" 80 || fail "ego_planner_v2 missing for drone_${drone_id}"
done

echo "[5/6] Validate planner heartbeat and control commands"
for drone_id in 0 1 2; do
  wait_for_topic_message "/drone_${drone_id}/traj_server_v2/heartbeat" 20 || \
    fail "No planner heartbeat for drone_${drone_id}"
  wait_for_topic_message "/drone_${drone_id}/position_cmd" 25 || \
    fail "No position_cmd for drone_${drone_id}"
done

echo "[6/6] Validate process graph completeness"
PLANNER_COUNT=$(rosnode list 2>/dev/null | grep -Ec '^/drone_[0-2]/ego_planner_v2$' || true)
TRAJ_SERVER_COUNT=$(rosnode list 2>/dev/null | grep -Ec '^/drone_[0-2]/traj_server_v2$' || true)
PX4CTRL_COUNT=$(rosnode list 2>/dev/null | grep -Ec '^/drone_[0-2]/px4ctrl$' || true)

[[ "$PLANNER_COUNT" -eq 3 ]] || fail "Expected 3 planner nodes, got $PLANNER_COUNT"
[[ "$TRAJ_SERVER_COUNT" -eq 3 ]] || fail "Expected 3 traj_server_v2 nodes, got $TRAJ_SERVER_COUNT"
[[ "$PX4CTRL_COUNT" -eq 3 ]] || fail "Expected 3 px4ctrl nodes, got $PX4CTRL_COUNT"

for expected_node in "${EXPECTED_NODES[@]}"; do
  wait_for_node "$expected_node" 30 || fail "Expected optional node not found: $expected_node"
done

for expected_topic in "${EXPECTED_TOPICS[@]}"; do
  wait_for_topic_message "$expected_topic" 20 || fail "Expected optional topic inactive: $expected_topic"
done

echo ""
echo "=========================================="
echo "V2 runtime health check passed"
echo "Launch log: $LAUNCH_LOG"
echo "=========================================="