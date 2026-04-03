#!/bin/bash

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "$0")" && pwd)
exec "${ROOT_DIR}/test_swarm_top_level_runtime_health.sh" --version v2 "$@"