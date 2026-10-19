#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/third_party/mad_icp"
BLD="$ROOT/build_external/mad_icp"

mkdir -p "$BLD"
cmake -S "$SRC" -B "$BLD"   -DCMAKE_BUILD_TYPE=Release   -DCOMPILE_CPP_APPS=ON
cmake --build "$BLD" -j"${MAD_BUILD_JOBS:-2}"

BIN="$BLD/apps/cpp_runners/bin_runner"
test -x "$BIN"
echo "MAD-ICP ready: $BIN"
