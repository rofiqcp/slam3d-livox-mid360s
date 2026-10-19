#!/usr/bin/env bash
# Relocatable, memory-conservative ROS 2 build. Artifacts are separate from sources.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
[[ -f "$ROS_SETUP" ]] || { echo "Missing ROS setup: $ROS_SETUP" >&2; exit 2; }
_source_ros_safe() {
  local had_nounset=0 rc
  [[ $- == *u* ]] && had_nounset=1
  set +u
  source "$1"
  rc=$?
  if (( had_nounset )); then set -u; fi
  return "$rc"
}
_source_ros_safe "$ROS_SETUP"
AVAILABLE_KB="$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)"
MIN_AVAILABLE_KB="${MIN_AVAILABLE_KB:-1800000}"
if (( AVAILABLE_KB < MIN_AVAILABLE_KB )); then
  echo "Not starting build: available RAM ${AVAILABLE_KB} kB < ${MIN_AVAILABLE_KB} kB" >&2
  exit 75
fi
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
export PYTHONDONTWRITEBYTECODE=1
KEY="$(printf '%s' "$ROOT" | cksum | cut -d' ' -f1)"
CACHE="${ROS_COLCON_CACHE_BASE:-${XDG_CACHE_HOME:-$HOME/.cache}/otomasi-colcon}/$(basename "$ROOT")-$KEY"
mkdir -p "$CACHE"
exec 9>"${ROS_COLCON_CACHE_BASE:-${XDG_CACHE_HOME:-$HOME/.cache}/otomasi-colcon}/.global-colcon.lock"
flock -n 9 || { echo "Another protected colcon build is already running" >&2; exit 75; }
ulimit -v "${COLCON_MAX_VMEM_KB:-4194304}"
export CMAKE_PREFIX_PATH="$HOME/.local/opt/gtsam/opt/ros/humble:$HOME/.local/opt/sophus-1.24.6:$HOME/.local/opt/livox-sdk2:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$HOME/.local/opt/gtsam/opt/ros/humble/lib/x86_64-linux-gnu:$HOME/.local/opt/livox-sdk2/lib:${LD_LIBRARY_PATH:-}"
cd "$ROOT"
unset -f _source_ros_safe
exec nice -n 10 ionice -c2 -n7 colcon --log-base "$CACHE/log" build \
  --base-paths "$ROOT/src" --build-base "$CACHE/build" --install-base "$CACHE/install" \
  --executor sequential --parallel-workers 1 --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DRAPIDJSON_INCLUDE_DIR="$HOME/.local/include" "$@"
