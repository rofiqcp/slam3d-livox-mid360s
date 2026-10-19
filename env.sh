#!/usr/bin/env bash
# Portable LiDAR workspace environment. Does not overwrite a caller's ROOT/CACHE.
SLAM3D_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
[[ -f "$ROS_SETUP" ]] || { echo "Missing ROS setup: $ROS_SETUP" >&2; return 2; }
_source_ros_safe() {
  local had_nounset=0 rc
  [[ $- == *u* ]] && had_nounset=1
  set +u
  source "$1"
  rc=$?
  if (( had_nounset )); then set -u; fi
  return "$rc"
}
_source_ros_safe "$ROS_SETUP" || return
SLAM3D_KEY="$(printf '%s' "$SLAM3D_ROOT" | cksum | cut -d' ' -f1)"
SLAM3D_COLCON_CACHE="${ROS_COLCON_CACHE_BASE:-${XDG_CACHE_HOME:-$HOME/.cache}/otomasi-colcon}/$(basename "$SLAM3D_ROOT")-$SLAM3D_KEY"
[[ -f "$SLAM3D_COLCON_CACHE/install/setup.bash" ]] || { echo "Build required: $SLAM3D_ROOT/build.sh" >&2; return 1; }
export LIVOX_MID360_ROOT="$SLAM3D_ROOT"
export SLAM3D_CACHE="$SLAM3D_COLCON_CACHE"
export SLAM3D_DATA="${SLAM3D_DATA:-${XDG_DATA_HOME:-$HOME/.local/share}/slam3d-mid360s}"
export LIVOX_MID360_CONFIG="$SLAM3D_ROOT/config/livox_runtime.yaml"
export LIVOX_MID360_EXTRINSICS="$SLAM3D_ROOT/config/extrinsics.yaml"
export ROS_LOG_DIR="$SLAM3D_COLCON_CACHE/ros-log"
export LD_LIBRARY_PATH="$HOME/.local/opt/gtsam/opt/ros/humble/lib/x86_64-linux-gnu:$HOME/.local/opt/livox-sdk2/lib:${LD_LIBRARY_PATH:-}"
_source_ros_safe "$SLAM3D_COLCON_CACHE/install/setup.bash"
unset -f _source_ros_safe
