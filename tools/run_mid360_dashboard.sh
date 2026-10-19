#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source /opt/ros/humble/setup.bash
source "$ROOT/env.sh"
export LIVOX_MID360_ROOT="$ROOT"
export LIVOX_MID360_CONFIG="$ROOT/config/livox_runtime.yaml"
export LIVOX_MID360_EXTRINSICS="$ROOT/config/extrinsics.yaml"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# Never launch two Livox SDK2 drivers on the same UDP ports.
# If external Livox driver exists, attach to it. After reboot, start our own driver.
if pgrep -f '[/]livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node' >/dev/null; then
  START_SENSOR=false
  echo "[mid360-dashboard] Existing Livox driver found: reusing ROS topics."
else
  START_SENSOR=true
  echo "[mid360-dashboard] No existing driver: starting Livox with mapping pipeline."
fi
exec /opt/ros/humble/bin/ros2 launch slam3d_mid360_dashboard mapping.launch.py \
  start_pipeline:=true start_sensor:="$START_SENSOR" \
  enable_loop_closure:=true web_port:=5001
