#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$ROOT/env.sh"
export LIVOX_MID360_ROOT="$ROOT"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
exec ros2 launch slam3d_mid360_dashboard foxglove.launch.py
