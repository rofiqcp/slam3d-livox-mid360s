#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /media/sirobo/HDD/slam3d-livox-mid360s/install/setup.bash
export LIVOX_MID360_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export ROS_DOMAIN_ID=0
exec ros2 launch slam3d_mid360_dashboard foxglove.launch.py
