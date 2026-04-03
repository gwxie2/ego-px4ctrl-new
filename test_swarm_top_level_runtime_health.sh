#!/bin/bash

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$ROOT_DIR"

source tools/source_phase1_env.sh

VERSION=""
OUTPUT_DIR=""
WAIT_FOR_SESSION_COMPLETE="false"
declare -a EXPECTED_NODES=()
declare -a EXPECTED_TOPICS=()
declare -a LAUNCH_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --version)
      VERSION="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --wait-for-session-complete)
      WAIT_FOR_SESSION_COMPLETE="true"
      shift
      ;;
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

if [[ -z "$VERSION" ]]; then
  echo "[runtime-health] ERROR: missing --version {v1|v2}" >&2
  exit 1
fi

LAUNCH_FILE=""
COMMANDER_NODE=""
PLANNER_NODE_NAME=""
TRAJ_SERVER_NODE_NAME=""
REPLAN_TOPIC_SUFFIX=""
declare -a DEFAULT_ARGS=()

case "$VERSION" in
  v1)
    LAUNCH_FILE="swarm_top_level_v1.launch"
    COMMANDER_NODE="/swarm_dynamic_commander"
    PLANNER_NODE_NAME="ego_planner"
    TRAJ_SERVER_NODE_NAME="traj_server"
    REPLAN_TOPIC_SUFFIX="/ego_planner/planning/replan_info"
    DEFAULT_ARGS=("traj_trigger_delay:=16.0")
    ;;
  v2)
    LAUNCH_FILE="swarm_top_level_v2.launch"
    COMMANDER_NODE="/swarm_dynamic_commander_v2"
    PLANNER_NODE_NAME="ego_planner_v2"
    TRAJ_SERVER_NODE_NAME="traj_server_v2"
    REPLAN_TOPIC_SUFFIX="/ego_planner_v2/planning/replan_info"
    DEFAULT_ARGS=()
    ;;
  *)
    echo "[runtime-health] ERROR: unsupported version '$VERSION'" >&2
    exit 1
    ;;
esac

LAUNCH_LOG=$(mktemp "/tmp/swarm_${VERSION}_runtime_health.XXXXXX.log")
if [[ -n "$OUTPUT_DIR" ]]; then
  BENCHMARK_OUTPUT_DIR="$OUTPUT_DIR"
  mkdir -p "$BENCHMARK_OUTPUT_DIR"
else
  BENCHMARK_OUTPUT_DIR=$(mktemp -d "/tmp/swarm_${VERSION}_benchmark.XXXXXX")
fi
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
  echo "[runtime-health] benchmark output dir: $BENCHMARK_OUTPUT_DIR"
  find "$BENCHMARK_OUTPUT_DIR" -maxdepth 2 -type f | sort || true
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

wait_for_service() {
  local service_name="$1"
  local timeout_s="$2"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if rosservice list 2>/dev/null | grep -Fxq "$service_name"; then
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

wait_for_path_with_min_lines() {
  local find_name="$1"
  local min_lines="$2"
  local timeout_s="$3"
  local deadline=$((SECONDS + timeout_s))
  local file_path=""
  while (( SECONDS < deadline )); do
    file_path=$(find "$BENCHMARK_OUTPUT_DIR" -type f -name "$find_name" -print -quit 2>/dev/null || true)
    if [[ -n "$file_path" ]]; then
      if [[ $(wc -l < "$file_path") -ge "$min_lines" ]]; then
        return 0
      fi
    fi
    sleep 1
  done
  return 1
}

wait_for_file_count() {
  local find_name="$1"
  local min_count="$2"
  local timeout_s="$3"
  local deadline=$((SECONDS + timeout_s))
  local count=0
  while (( SECONDS < deadline )); do
    count=$(find "$BENCHMARK_OUTPUT_DIR" -type f -name "$find_name" | wc -l)
    count=${count// /}
    if [[ "$count" -ge "$min_count" ]]; then
      return 0
    fi
    sleep 1
  done
  return 1
}

echo "=========================================="
echo "Swarm ${VERSION^^} Top-level Runtime Health"
echo "=========================================="

echo "[1/6] Launch headless ${VERSION} stack with benchmark enabled"
setsid roslaunch clean_uav_core "$LAUNCH_FILE" \
  gui:=false \
  use_rviz:=false \
  enable_oscillation:=false \
  benchmark_enable:=true \
  benchmark_output_dir:="$BENCHMARK_OUTPUT_DIR" \
  takeoff_delay:=5.0 \
  goal_start_delay:=10.0 \
  planner_post_takeoff_delay:=0.5 \
  "${DEFAULT_ARGS[@]}" \
  "${LAUNCH_ARGS[@]}" \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
trap cleanup EXIT

wait_for_master 30 || fail "ROS master did not become available"
wait_for_node "/gazebo" 40 || fail "Gazebo node did not start"
wait_for_node "$COMMANDER_NODE" 30 || fail "Commander node missing: $COMMANDER_NODE"
wait_for_node "/benchmark/benchmark_manager" 20 || fail "Benchmark manager node did not start"
wait_for_service "/benchmark/start_session" 20 || fail "Benchmark service /benchmark/start_session not available"

echo "[2/6] Validate base telemetry and MAVROS connectivity"
for drone_id in 0 1 2; do
  wait_for_node "/drone_${drone_id}/px4ctrl" 45 || fail "px4ctrl missing for drone_${drone_id}"
  wait_for_topic_message "/drone_${drone_id}/odom" 25 || fail "No odom for drone_${drone_id}"
  wait_for_topic_field "/iris_${drone_id}/mavros/state" "connected:[[:space:]]*(true|True)" 35 || \
    fail "MAVROS never connected for iris_${drone_id}"
done

echo "[3/6] Validate takeoff and planner bring-up"
for drone_id in 0 1 2; do
  wait_for_topic_field "/iris_${drone_id}/mavros/state" "armed:[[:space:]]*(true|True)" 50 || \
    fail "Vehicle iris_${drone_id} never armed"
  wait_for_node "/drone_${drone_id}/${PLANNER_NODE_NAME}" 80 || fail "Planner missing for drone_${drone_id}"
  wait_for_node "/drone_${drone_id}/${TRAJ_SERVER_NODE_NAME}" 80 || fail "Traj server missing for drone_${drone_id}"
done

echo "[4/6] Validate planner outputs and benchmark telemetry"
for drone_id in 0 1 2; do
  wait_for_topic_message "/drone_${drone_id}/position_cmd" 40 || fail "No position_cmd for drone_${drone_id}"
  wait_for_topic_message "/drone_${drone_id}${REPLAN_TOPIC_SUFFIX}" 50 || fail "No replan_info for drone_${drone_id}"
done

echo "[5/6] Validate benchmark session artifacts"
wait_for_path_with_min_lines "*_manifest.json" 1 40 || fail "Benchmark manifest was not created"
wait_for_path_with_min_lines "drone_0_*_replan.csv" 2 40 || fail "Benchmark replan CSV for drone_0 has no data rows"
wait_for_path_with_min_lines "drone_0_*_event.csv" 2 40 || fail "Benchmark event CSV for drone_0 has no data rows"

if [[ "$WAIT_FOR_SESSION_COMPLETE" == "true" ]]; then
  wait_for_path_with_min_lines "*_summary.json" 1 160 || fail "Benchmark summary was not created"
  wait_for_file_count "*.bag" 3 60 || fail "Finalized rosbag files were not created"
fi

echo "[6/6] Validate optional expectations"
for expected_node in "${EXPECTED_NODES[@]}"; do
  wait_for_node "$expected_node" 30 || fail "Expected optional node not found: $expected_node"
done

for expected_topic in "${EXPECTED_TOPICS[@]}"; do
  wait_for_topic_message "$expected_topic" 20 || fail "Expected optional topic inactive: $expected_topic"
done

echo ""
echo "=========================================="
echo "${VERSION^^} top-level runtime health passed"
echo "Launch log: $LAUNCH_LOG"
echo "Benchmark output: $BENCHMARK_OUTPUT_DIR"
echo "=========================================="