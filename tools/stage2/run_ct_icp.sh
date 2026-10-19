#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DATA="${1:-$ROOT/datasets/stage2/ct_ply}"
OUT="${2:-${SLAM3D_DATA:-$HOME/.local/share/slam3d-mid360s}/results/stage2/ct_icp_run}"
CT="$ROOT/external/ct_icp"
TEMPLATE="$CT/config/odometry/mid360s_stage2.yaml"
CFG="$OUT/ct_runtime.yaml"

if [ ! -d "$DATA/00/frames" ]; then
  echo "CT input expected at: $DATA/00/frames" >&2
  echo "Export it with stage2_export_ct.launch.py first." >&2
  exit 2
fi

rm -rf "$OUT"
mkdir -p "$OUT"
ln -sfn 00 "$DATA/Town01"

python3 - "$TEMPLATE" "$CFG" "$DATA" "$OUT" <<'PY'
import sys, yaml, os
from pathlib import Path
src,dst,data,out=map(Path,sys.argv[1:])
cfg=yaml.safe_load(src.read_text())
cfg['output_dir']=str(out)
cfg['generate_directory_prefix']=False
cfg['with_viz3d']=False
cfg['dataset_options'][0]['root_path']=str(data)
cfg['dataset_options'][0]['dataset']='KITTI_CARLA'
cfg['dataset_options'][0]['use_all_datasets']=False
cfg['dataset_options'][0]['sequence_options']=[{
    'sequence_name':'Town01',
    'start_frame_id':0,
    'max_num_frames':int(os.environ.get('CT_MAX_FRAMES','100'))
}]
dst.write_text(yaml.safe_dump(cfg,sort_keys=False))
PY

/usr/bin/time -v "$CT/build/command/run_odometry"   -c "$CFG"   > "$OUT/run.log" 2> "$OUT/time.log"

echo "CT-ICP result: $OUT/KITTI_CARLA_Town01.PLY"
tail -20 "$OUT/run.log"
