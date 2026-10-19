#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
JOBS="${JOBS:-8}"

echo "[1/3] ROS2 workspace"
cd "$ROOT"
./build.sh

echo "[2/3] MAD-ICP C++ backend"
cmake -S "$ROOT/external/mad_icp"       -B "$ROOT/external/mad_icp/build"       -DCMAKE_BUILD_TYPE=Release
cmake --build "$ROOT/external/mad_icp/build" -j"$JOBS"

echo "[3/3] CT-ICP C++ backend"
cmake -S "$ROOT/external/ct_icp"       -B "$ROOT/external/ct_icp/build"       -DCMAKE_BUILD_TYPE=Release       -DSUPERBUILD_CONFIG="$ROOT/external/ct_icp/superbuild_import.cmake"       -DSUPERBUILD_INSTALL_DIR="$ROOT/external/ct_icp/install"       -DWITH_GTSAM=OFF       -DWITH_VIZ3D=OFF       -DWITH_PYTHON_BINDING=OFF
cmake --build "$ROOT/external/ct_icp/build" -j"$JOBS"

echo "Stage2 backends ready."
