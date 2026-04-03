#!/bin/bash

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$ROOT_DIR"

source tools/source_phase1_env.sh

VERSIONS=(v1 v2)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --version)
      case "$2" in
        all)
          VERSIONS=(v1 v2)
          ;;
        v1|v2)
          VERSIONS=("$2")
          ;;
        *)
          echo "[smoke] ERROR: unsupported --version value '$2'" >&2
          exit 1
          ;;
      esac
      shift 2
      ;;
    *)
      echo "[smoke] ERROR: unknown argument '$1'" >&2
      exit 1
      ;;
  esac
done

check_static_graph() {
  local version="$1"
  local launch_file=""
  local nodes=""

  case "$version" in
    v1)
      launch_file="swarm_top_level_v1.launch"
      ;;
    v2)
      launch_file="swarm_top_level_v2.launch"
      ;;
  esac

  test -f "src/clean_uav_core/launch/${launch_file}"
  nodes=$(roslaunch --nodes clean_uav_core "$launch_file" gui:=false use_rviz:=false benchmark_enable:=true)

  echo "$nodes" | grep -Fxq "/benchmark/benchmark_manager"
  echo "$nodes" | grep -Fxq "/drone_0/px4ctrl"

  if [[ "$version" == "v1" ]]; then
    echo "$nodes" | grep -Fxq "/drone_0/ego_planner"
    echo "$nodes" | grep -Fxq "/drone_0/traj_server"
    echo "$nodes" | grep -Fxq "/swarm_traj_trigger"
    echo "$nodes" | grep -Fxq "/swarm_dynamic_commander"
  else
    echo "$nodes" | grep -Fxq "/drone_0/ego_planner_v2"
    echo "$nodes" | grep -Fxq "/drone_0/traj_server_v2"
    echo "$nodes" | grep -Fxq "/swarm_dynamic_commander_v2"
  fi
}

echo "=========================================="
echo "Swarm Top-level Two-step Smoke Test"
echo "=========================================="

echo "[1/2] Static node-graph checks"
for version in "${VERSIONS[@]}"; do
  echo "  - checking ${version} launch graph"
  check_static_graph "$version"
done

echo "[2/2] Runtime health checks"
for version in "${VERSIONS[@]}"; do
  echo "  - running ${version} runtime health"
  "${ROOT_DIR}/test_swarm_top_level_runtime_health.sh" --version "$version"
done

echo ""
echo "=========================================="
echo "Two-step smoke test passed"
echo "=========================================="