#!/bin/bash

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "$0")/../.." && pwd)
cd "$ROOT_DIR"

VERSION=""
LAUNCH_FILE=""
OUTPUT_DIR=""
WAIT_FOR_SESSION_COMPLETE="true"
CLEANUP_BEFORE_RUN="true"
DRY_RUN="false"
VISUALIZE="false"
declare -a LAUNCH_ARGS=()

has_launch_arg_key() {
  local key="$1"
  local arg_text
  for arg_text in "${LAUNCH_ARGS[@]}"; do
    if [[ "$arg_text" == "$key:="* ]]; then
      return 0
    fi
  done
  return 1
}

append_launch_arg_if_missing() {
  local key="$1"
  local value="$2"
  if ! has_launch_arg_key "$key"; then
    LAUNCH_ARGS+=("$key:=$value")
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --version)
      VERSION="$2"
      shift 2
      ;;
    --launch-file)
      LAUNCH_FILE="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --launch-arg)
      LAUNCH_ARGS+=("$2")
      shift 2
      ;;
    --no-wait)
      WAIT_FOR_SESSION_COMPLETE="false"
      shift
      ;;
    --no-cleanup)
      CLEANUP_BEFORE_RUN="false"
      shift
      ;;
    --dry-run)
      DRY_RUN="true"
      shift
      ;;
    --visualize)
      VISUALIZE="true"
      shift
      ;;
    *)
      echo "[benchmark-runner] ERROR: unknown argument '$1'" >&2
      exit 1
      ;;
  esac
done

if [[ -z "$VERSION" ]]; then
  echo "[benchmark-runner] ERROR: missing --version" >&2
  exit 1
fi

if [[ -z "$OUTPUT_DIR" ]]; then
  echo "[benchmark-runner] ERROR: missing --output-dir" >&2
  exit 1
fi

if [[ "$VISUALIZE" == "true" ]]; then
  append_launch_arg_if_missing "gui" "true"
  append_launch_arg_if_missing "use_rviz" "true"
fi

append_launch_arg_if_missing "planner_log_to_file" "true"
append_launch_arg_if_missing "planner_log_session" "$(basename "$OUTPUT_DIR")"

COMMAND=("$ROOT_DIR/test_swarm_top_level_runtime_health.sh" "--version" "$VERSION" "--output-dir" "$OUTPUT_DIR")

if [[ -n "$LAUNCH_FILE" ]]; then
  COMMAND+=("--launch-file" "$LAUNCH_FILE")
fi

if [[ "$WAIT_FOR_SESSION_COMPLETE" == "true" ]]; then
  COMMAND+=("--wait-for-session-complete")
fi

if [[ ${#LAUNCH_ARGS[@]} -gt 0 ]]; then
  COMMAND+=("--" "${LAUNCH_ARGS[@]}")
fi

echo "[benchmark-runner] version=$VERSION"
echo "[benchmark-runner] launch_file=${LAUNCH_FILE:-<default>}"
echo "[benchmark-runner] output_dir=$OUTPUT_DIR"
echo "[benchmark-runner] wait_for_session_complete=$WAIT_FOR_SESSION_COMPLETE"
echo "[benchmark-runner] visualize=$VISUALIZE"
if [[ ${#LAUNCH_ARGS[@]} -gt 0 ]]; then
  printf '[benchmark-runner] launch_args=%s\n' "${LAUNCH_ARGS[*]}"
fi

if [[ "$DRY_RUN" == "true" ]]; then
  printf '[benchmark-runner] dry-run command:'
  printf ' %q' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

mkdir -p "$OUTPUT_DIR"

if [[ "$CLEANUP_BEFORE_RUN" == "true" ]]; then
  "$ROOT_DIR/cleanup_swarm.sh"
fi

"${COMMAND[@]}"