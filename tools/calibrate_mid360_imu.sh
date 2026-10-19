#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ "${1:-}" != "--confirm-stationary" ]]; then
  echo "Use: $0 --confirm-stationary" >&2
  echo "Keep Livox MID-360S physically still for >=15 seconds; no motor commands." >&2
  exit 2
fi
source "$ROOT/env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
mkdir -p "${SLAM3D_DATA:-$HOME/.local/share/slam3d-mid360s}/results/calibration"
FILE="${SLAM3D_DATA:-$HOME/.local/share/slam3d-mid360s}/results/calibration/imu_stationary_live_$(date +%Y%m%d_%H%M%S).yaml"
echo "Collecting stationary Livox IMU on /livox/imu -> $FILE"
timeout -s INT -k 3 48s ros2 run livox_mid360_manager imu_stationary_stats --ros-args \
  -p duration_seconds:=15.0 \
  -p min_samples:=2400 \
  -p wait_timeout_seconds:=25.0 \
  -p output_path:="$FILE"
if [[ ! -s "$FILE" ]]; then
  echo "No baseline written; check power/network /livox/imu topic." >&2
  exit 3
fi
if ! grep -Eq '^[[:space:]]*stationary_ok:[[:space:]]*true' "$FILE"; then
  echo "IMU did not pass stationary thresholds. Do not use this as calibration." >&2
  cat "$FILE"
  exit 4
fi
echo "Stationary IMU baseline PASSED: $FILE"
echo "Sensor-to-base_link and LiDAR-to-IMU extrinsics remain UNCALIBRATED."
echo "Never set extrinsics.status=verified without mounting measurements."
