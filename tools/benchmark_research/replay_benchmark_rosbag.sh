#!/bin/bash

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "$0")/../.." && pwd)
cd "$ROOT_DIR"

source tools/source_phase1_env.sh

INPUT_PATH=""
RATE="1.0"
PAUSED="false"
LOOP_PLAYBACK="false"
OPEN_RVIZ="true"
RVIZ_CONFIG="$(rospack find clean_uav_core)/rviz/benchmark_rosbag_replay.rviz"
declare -a EXPLICIT_BAGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --input)
      INPUT_PATH="$2"
      shift 2
      ;;
    --bag)
      EXPLICIT_BAGS+=("$2")
      shift 2
      ;;
    --rate)
      RATE="$2"
      shift 2
      ;;
    --paused)
      PAUSED="true"
      shift
      ;;
    --loop)
      LOOP_PLAYBACK="true"
      shift
      ;;
    --no-rviz)
      OPEN_RVIZ="false"
      shift
      ;;
    --rviz-config)
      RVIZ_CONFIG="$2"
      shift 2
      ;;
    *)
      echo "[bag-replay] ERROR: unknown argument '$1'" >&2
      exit 1
      ;;
  esac
done

declare -a BAG_FILES=()
if [[ ${#EXPLICIT_BAGS[@]} -gt 0 ]]; then
  BAG_FILES=("${EXPLICIT_BAGS[@]}")
elif [[ -n "$INPUT_PATH" ]]; then
  if [[ -f "$INPUT_PATH" ]]; then
    BAG_FILES=("$INPUT_PATH")
  elif [[ -d "$INPUT_PATH" ]]; then
    while IFS= read -r bag_file; do
      BAG_FILES+=("$bag_file")
    done < <(find "$INPUT_PATH" -maxdepth 2 -type f -name '*.bag' | sort)
  fi
fi

if [[ ${#BAG_FILES[@]} -eq 0 ]]; then
  echo "[bag-replay] ERROR: no rosbag files found. Use --input <run_dir|session_dir|bag> or --bag <path>." >&2
  exit 1
fi

ROSCORE_PID=""
RVIZ_PID=""
REPLAY_HELPER_PID=""

cleanup() {
  if [[ -n "$REPLAY_HELPER_PID" ]] && kill -0 "$REPLAY_HELPER_PID" 2>/dev/null; then
    kill "$REPLAY_HELPER_PID" 2>/dev/null || true
    wait "$REPLAY_HELPER_PID" 2>/dev/null || true
  fi
  if [[ -n "$RVIZ_PID" ]] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill "$RVIZ_PID" 2>/dev/null || true
    wait "$RVIZ_PID" 2>/dev/null || true
  fi
  if [[ -n "$ROSCORE_PID" ]] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
    kill "$ROSCORE_PID" 2>/dev/null || true
    wait "$ROSCORE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if ! rosparam list >/dev/null 2>&1; then
  roscore >/tmp/benchmark_rosbag_replay_roscore.log 2>&1 &
  ROSCORE_PID=$!
  for _ in $(seq 1 15); do
    if rosparam list >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done
fi

rosparam set use_sim_time true

rosrun clean_uav_core replay_path_publisher.py >/tmp/benchmark_rosbag_replay_helper.log 2>&1 &
REPLAY_HELPER_PID=$!

if [[ "$OPEN_RVIZ" == "true" ]]; then
  rviz -d "$RVIZ_CONFIG" >/tmp/benchmark_rosbag_replay_rviz.log 2>&1 &
  RVIZ_PID=$!
fi

PLAY_COMMAND=(rosbag play --clock --rate "$RATE")
if [[ "$PAUSED" == "true" ]]; then
  PLAY_COMMAND+=(--pause)
fi
if [[ "$LOOP_PLAYBACK" == "true" ]]; then
  PLAY_COMMAND+=(--loop)
fi
PLAY_COMMAND+=("${BAG_FILES[@]}")

echo "[bag-replay] replaying ${#BAG_FILES[@]} bag file(s)"
printf '[bag-replay] command:'
printf ' %q' "${PLAY_COMMAND[@]}"
printf '\n'

"${PLAY_COMMAND[@]}"