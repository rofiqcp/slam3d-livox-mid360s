#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DATA="${1:-$ROOT/datasets/stage2/synthetic_kitti/scans}"
OUT="${2:-${SLAM3D_DATA:-$HOME/.local/share/slam3d-mid360s}/results/stage2/mad_icp_run}"
MAD="$ROOT/external/mad_icp"

if [ ! -d "$DATA" ]; then
  echo "MAD input scan directory not found: $DATA" >&2
  exit 2
fi

rm -rf "$OUT"
mkdir -p "$OUT"
/usr/bin/time -v "$MAD/build/bin_runner"   -data_path "$DATA"   -estimate_path "$OUT"   -dataset_config "$MAD/configurations/datasets/mid360s_stage2.cfg"   -mad_icp_config "$MAD/configurations/default.cfg"   -num_cores "${MAD_CORES:-8}"   -num_keyframes "${MAD_KEYFRAMES:-8}"   > "$OUT/run.log" 2> "$OUT/time.log"

echo "MAD-ICP result: $OUT/estimate.txt"
tail -20 "$OUT/run.log"
