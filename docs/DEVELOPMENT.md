# Development, reproducibility, and data hygiene

## Build and inspect

```bash
cd /path/to/slam3d-livox-mid360s
./build.sh
source ./env.sh
python3 tools/check_repo_hygiene.py
```

The host must supply ROS 2 Humble, Livox SDK and all required SLAM libraries. A clean build and a live hardware check are separate validations. In particular, GTSAM/CT-ICP/MAD-ICP optional backends can need host-specific dependency installation.

## Source versus artifacts

Check in maintained code, CMake, ROS packages, launch files, reference YAML, upstream sources needed to build and essential docs. Keep `datasets/`, `results/`, `build/`, `install/`, `log/`, rosbags, point clouds, trajectories, calibration captures, screenshots and temporary diagnostic reports locally. Repositories are not backups of raw LiDAR data.

Do not put additional full copies of Git-tracked packages in `src/`. Never remove vendored copyright and license metadata. If a vendored dependency must be upgraded, record upstream source and version in the change.

## Validation checklist

```bash
python3 tools/check_repo_hygiene.py
bash -n build.sh tools/*.sh tools/stage2/*.sh
git diff --check
git status --short
```

Run `colcon build`/targeted `colcon test` on a prepared ROS system for code changes, and report packages tested separately from sensor-connected experiments. Protect low-RAM MiniPC1 from parallel high-memory builds. A valid script does not prove the LiDAR is reachable or SLAM accuracy is sufficient.

See [../SYNC_GUIDE.md](../SYNC_GUIDE.md) for Sirobo/MiniPC1 synchronization.
