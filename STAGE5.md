# Stage 5 — Saved 3D Map Loading, Relocalization and Tracking Recovery

ROS2 Humble workspace: /media/sirobo/HDD/slam3d-livox-mid360s

## Architecture and guarantees

Stage5 adds the independent C++ ROS2 package `src/slam3d_relocalizer`. It consumes:

- P4 saved global map PCD, e.g. `results/stage4/maps/stage4_map_*.pcd`.
- Matching P4 keyframe trajectory `stage4_poses_*.csv` (automatically inferred when names match; also accepts explicit `poses_csv`).
- Live P3 `/lio/odometry` with header `lio_odom`, child `livox_frame`.
- P3 `/lio/cloud_registered` in `lio_odom`.

The map must be explicitly selected by path. The normal live launch never silently picks the saved synthetic fixture as a production map. The separate `stage5_fixture.launch.py` intentionally selects the newest saved P4 fixture map for test purposes only.

Stage5 **does not start P4 pose graph** while localizing. This avoids two publishers for `map -> lio_odom`. P3 remains responsible for local drift-smooth odometry `lio_odom -> livox_frame`; P5 adds the global correction `map -> lio_odom`. No ROS1 stack or Docker/WSL is needed.

## Algorithms

1. Validate/load PCD and paired CSV, reject corrupt or too small maps, discard nonfinite points, voxel-downsample.
2. Synchronize registered world cloud and P3 odometry by timestamp. Validate frame IDs and monotonic sensor stamps.
3. Convert registered cloud from `lio_odom` to sensor-local using inverse of P3 `lio_odom -> livox_frame` pose.
4. On global relocalization, sample up to `max_candidates` saved keyframe poses and perform hierarchical coarse + fine PCL ICP. A manually supplied `/initialpose` in the `map` frame takes priority over global candidates.
5. Require successful ICP convergence, bounded fitness and minimum inlier overlap. Reject unknown or ambiguous/poor matches instead of publishing an unsupported pose.
6. Once accepted, compute and publish corrected pose and `map -> lio_odom`. Continue output from the last accepted correction at scan frequency, while running fast ICP refinements periodically.
7. Monitor stale scans, registration failures and frame errors. On loss, stop publishing localized pose and TF; attempt global recovery after a cooldown if configured. Do not pretend interrupted localization is continuous.
8. Expose status and diagnostics and services for relocalization, map reload and snapshotting the current validated position.

Global candidate search is a **keyframe-seeded local-regression search**, not an unlimited arbitrary 6-DoF feature-based global place recognizer. Saved map and trajectory quality are essential. A very sparse map may safely remain LOST; use a denser P4 survey and avoid weakening quality gates without independent ground truth.

## ROS2 topics

- Inputs: `/lio/odometry`, `/lio/cloud_registered`, `/initialpose`.
- `/p5/reference_map`: latched global PCD PointCloud2, frame `map`.
- `/p5/localized_odometry`: corrected nav_msgs/Odometry with frame `map`, child `livox_frame`.
- `/p5/localized_path`: corrected path in frame `map`.
- `/p5/aligned_scan`: localized scan for inspection in RViz.
- `/p5/status`: `UNINITIALIZED`, `SEARCHING`, `TRACKING`, or `LOST` with counters.
- `/diagnostics`: status, map points, keyframe candidates, ICP fitness/overlap, failures, etc.
- TF: `map -> lio_odom` (P5) and `lio_odom -> livox_frame` (P3).

Services:
- `/p5/relocalize`: request global search on the next synchronized scan.
- `/p5/reload_map`: reload map path and CSV from current ROS node parameters, reset localization; refuse corrupt map.
- `/p5/save_pose`: save current validated sensor position to timestamped YAML. Returns success=false when LOST or uninitialized.

The `/initialpose` hint is the **Livox sensor pose**, not the robot base_link pose, since the physical sensor-to-base extrinsic has not yet been calibrated. Parent frame must be `map`.

## Build

```bash
cd /media/sirobo/HDD/slam3d-livox-mid360s
./build.sh
source env.sh
```

## Fully offline P5 fixture from saved P4 map

First save a P4 fixture map if none exists:

```bash
ros2 launch slam3d_pose_graph loop_fixture.launch.py
# In another terminal:
ros2 service call /p4/save_map std_srvs/srv/Trigger "{}"
```

Then test automatic relocalization against the most recent matched P4 fixture PCD/CSV:

```bash
ros2 launch livox_mid360_manager stage5_fixture.launch.py rviz:=true
```

To test loss and recovery caused by a deliberate 5-second point-cloud blackout:

```bash
ros2 launch livox_mid360_manager stage5_fixture.launch.py \
  rviz:=false frames:=110 fixture_rate:=5.0 \
  blackout_start:=25 blackout_frames:=25
```

Synthetic fixture results: `results/stage5/fixture_localized.csv`, `results/stage5/fixture_localized.summary.yaml`. The fixture uses the saved P4 PCD directly and a known sensor trajectory from its CSV, so this tests real ROS2 P5 geometry/ICP/TF, but not sensor physics.

## Live MID-360S (after powering the sensor)

Select a saved **real** Stage4 PCD and trajectory from the **same snapshot**:

```bash
source /media/sirobo/HDD/slam3d-livox-mid360s/env.sh
ros2 run livox_mid360_manager livox_discover --interface auto --timeout 5

ros2 launch livox_mid360_manager stage5.launch.py \
  map_pcd:=/path/to/stage4_map_<stamp>.pcd \
  poses_csv:=/path/to/stage4_poses_<stamp>.csv \
  rviz:=true
```

Do **not** start `stage4.launch.py` simultaneously with P5: both would publish `map -> lio_odom`. Stop P4 mapping before switching to P5 saved-map localization.

View status and force a new search:

```bash
ros2 topic echo /p5/status
ros2 service call /p5/relocalize std_srvs/srv/Trigger "{}"
ros2 service call /p5/save_pose std_srvs/srv/Trigger "{}"
```

For a manual hint, use RViz **2D Pose Estimate** only if it is converted correctly into the Livox sensor pose and the correct map frame. For true 3D initialization, publish `geometry_msgs/PoseWithCovarianceStamped` to `/initialpose` with `header.frame_id: map`, the calibrated sensor position and quaternion.

For replay of raw cloud + IMU bags:

```bash
ros2 launch livox_mid360_manager stage5_bag.launch.py \
  bag:=/path/to/rosbag \
  map_pcd:=/path/to/stage4_map_<stamp>.pcd \
  poses_csv:=/path/to/stage4_poses_<stamp>.csv \
  rviz:=false
```

The P3 rosbag launcher filters playback to raw sensor topics and regenerates FAST-LIO odometry; it never replays old recorded `/lio/odometry` or old `map` TF.

## Acceptance findings — October 8, 2026

- Full ROS2 workspace compiled: **6/6 packages**.
- P4 saved fixture map loaded: ~4,884 voxel points, 60 keyframe pose candidates.
- First search accepted pose with ICP fitness ~0.033 and 100% measured inlier overlap.
- Continuous tracking fixture: **106 localized poses**, approximately 5 Hz, 0 jumps, 0 non-monotonic stamps, translational RMSE **0.01638 m** versus synthetic GT.
- `/p5/save_pose` succeeded and created a timestamped YAML file.
- Blackout test: a cloud pause of 25 frames at 5 Hz triggered `LOST`, and P5 successfully accepted a **second relocalization** after cloud return (fitness ~0.0316, measured overlap 100%). The evaluator recorded one pose jump due to *real motion during the unobserved blackout*, not false timestamp ordering. Navigation must stop motion while LOST and handle recovery safely.
- A sparse 4-keyframe P4 map produced from a short synthetic open route **failed** global relocalization when replaying the same raw sensor bag. P5 correctly stayed LOST and refused `/p5/save_pose`. This is **not a passed localization case**. Validate map overlap / rerun mapping with denser keyframes and do not relax production rejection thresholds blindly.

The physical MID-360S remained powered off. Real-world localization precision, convergence and long-term tracking are **not validated**.

## Limitations and next integration

- A PCD map plus CSV keyframe pose list is required for automatic global search.
- Global matching time and CPU consumption scale with the number of candidate poses and ICP thresholds; benchmark a dense real map and realistic CPU load.
- The saved-map pipeline currently does not load full keyframe descriptors or do learned global place recognition; long-term global relocalization should eventually incorporate a dedicated descriptor database with stronger geometric verification.
- During LOST, do not send map-frame navigation goals based on stale TF. The consumer should monitor `/p5/status` and suspend autonomous motion until tracking revalidates.
- Current sensor extrinsic remains uncalibrated; a real `base_link` transform and Nav2 integration require a subsequent controlled stage.
