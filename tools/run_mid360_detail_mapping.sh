#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$ROOT/env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
if systemctl --user is-active --quiet mid360-dashboard.service; then
  echo "Standard MID360 dashboard is already running with FAST-LIO/P4." >&2
  echo "First stop it explicitly: systemctl --user stop mid360-dashboard.service" >&2
  echo "This script refuses duplicate LIO/pose-graph publishers." >&2
  exit 2
fi
if ! timeout 5s ros2 topic echo /livox/lidar --once --field header >/dev/null 2>&1; then
  echo "No live LiDAR PointCloud2 messages. Power/connect and verify driver first." >&2
  exit 3
fi
if ! timeout 5s ros2 topic echo /livox/imu --once --field header >/dev/null 2>&1; then
  echo "No live IMU messages. Do not start mapping." >&2
  exit 4
fi
TAG="$(date +%Y%m%d_%H%M%S)"
MAPS="${SLAM3D_DATA:-$HOME/.local/share/slam3d-mid360s}/results/detail_mapping_$TAG/maps"
mkdir -p "$MAPS"
echo "Starting HIGH-DETAIL RESEARCH profile, no motor control."
echo "P4 map path: $MAPS"
echo "Extrinsics must still be calibrated for physical accuracy."
exec ros2 launch slam3d_mid360_dashboard mapping.launch.py \
  start_sensor:=false start_pipeline:=true web_port:=5001 \
  lio_config:="$ROOT/config/profiles/fast_lio_indoor_detail.yaml" \
  p4_config:="$ROOT/config/profiles/p4_indoor_detail.yaml" \
  map_output_dir:="$MAPS"
