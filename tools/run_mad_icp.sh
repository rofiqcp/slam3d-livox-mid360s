#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATASET="${1:-$ROOT/datasets/stage2/mid360s}"
NAME="${2:-mad_icp}"
SCANS="$DATASET/scans"
BIN="$ROOT/build_external/mad_icp/apps/cpp_runners/bin_runner"
DATA_CFG="$ROOT/third_party/mad_icp/configurations/datasets/mid360s.cfg"
PARAM_CFG="$ROOT/third_party/mad_icp/configurations/default.cfg"

if [ ! -d "$SCANS" ]; then
  echo "Missing scan directory: $SCANS" >&2
  echo "Export a bag first with stage2_export.launch.py." >&2
  exit 2
fi

COUNT=$(find "$SCANS" -maxdepth 1 -type f -name '*.bin' | wc -l)
if [ "$COUNT" -lt 2 ]; then
  echo "Need at least 2 .bin scans; found $COUNT" >&2
  exit 3
fi

if [ ! -x "$BIN" ]; then
  "$ROOT/tools/build_mad_icp.sh"
fi

STAMP=$(date +%Y%m%d_%H%M%S)
OUT="${SLAM3D_DATA:-$HOME/.local/share/slam3d-mid360s}/results/stage2/${NAME}_$STAMP"
mkdir -p "$OUT"

echo "MAD-ICP dataset: $DATASET"
echo "Scans: $COUNT"
echo "Output: $OUT"

/usr/bin/time -v "$BIN"   -data_path "$SCANS"   -estimate_path "$OUT"   -dataset_config "$DATA_CFG"   -mad_icp_config "$PARAM_CFG"   -num_cores "${MAD_CORES:-4}"   -num_keyframes "${MAD_KEYFRAMES:-4}"   > "$OUT/run.log" 2> "$OUT/resources.txt"

python3 "$ROOT/tools/summarize_matrix_trajectory.py"   "$OUT/estimate.txt"   "$OUT/summary.yaml"

echo "Done: $OUT"
cat "$OUT/summary.yaml"
