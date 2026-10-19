# STAGE4 Audit — 2026-10-08

Workspace: `/media/sirobo/HDD/slam3d-livox-mid360s`
Device: `sirobo-rdc1` / Ubuntu 22.04 / ROS2 Humble

## Scope and implementation

P4 adds `src/slam3d_pose_graph` as an independent ROS2 C++ backend, with GTSAM incremental iSAM2 optimization, a ring/sector Scan Context-style place descriptor, geometric ICP verification, Huber-robust loop factors, and voxelized corrected 3D cloud building.

The P3 FAST-LIO module and `livox_ros_driver2` are not replaced. LIO remains the smooth local pose estimate, and P4 provides an optional global map-frame correction.

Inputs are `/lio/odometry` and `/lio/cloud_registered`, both in `lio_odom`. P4 publishes `/p4/optimized_odometry`, `/p4/optimized_path`, `/p4/optimized_map`, diagnostics, and optional `map -> lio_odom` TF. The P3 `lio_odom -> livox_frame` TF remains intact.

### Protection / bounds

- Pose/cloud timestamp synchronization tolerance with stale-cloud dropping.
- Input frame validation to reject mismatched local/world coordinate systems.
- Keyframe distance/rotation thresholds and a cap on keyframe count.
- Candidate search restricted by minimum elapsed time, minimum keyframe separation, rough pose distance and descriptor score.
- ICP geometric verification with fitness, correction translation/rotation limits.
- Minimum keyframe gap after a verified loop to avoid repeated near-duplicate constraints.
- GTSAM first-pose prior, sequential odometry factors, and Huber-robust loop factors.
- Map voxel filter, maximum output point budget, and global map only rebuilt when dirty.
- Service save refuses empty maps, writes binary PCD + pose CSV, and uses unique wall-clock snapshot filenames to avoid overwriting previous saved snapshots.
- P4 output folders and evaluator arguments separated from P3 to prevent shared LaunchConfiguration collisions.

## Build

Full `./build.sh` with colcon scoped to `src/`: **5/5 ROS2 packages passed** during integration:
- `livox_ros_driver2`
- `livox_mid360_manager`
- `kiss_icp`
- `fast_lio`
- `slam3d_pose_graph`

P4 uses GTSAM available in ROS2 Humble and pins PCL to the system 1.12 CMake directory rather than mixing with locally installed PCL 1.15.

## Closed-loop fixture acceptance

`ros2 launch slam3d_pose_graph loop_fixture.launch.py` simulates a 180-frame circular rover route with actual closure and imposed drifting raw odometry.

Initial exploratory test (two accepted loop edges):
- Keyframes: 60; loops accepted: 2
- ICP fitness: ~0.0128 and ~0.0088
- PCD export: binary point cloud 11,113 points
- Final raw position error: ~0.69642 m
- Final optimized position error: ~0.04035 m (94.2% reduction)
- Trigger /p4/save_map succeeded, PCD+CSV saved, all processes exit 0

Conservative acceptance with loop-cooldown and tighter descriptor threshold:
- Keyframes: 60; verified loops: 1
- Candidate: keyframe 0 -> 59
- Descriptor distance: 0.167; ICP fitness: 0.0107
- Rejected loops: 0, dropped cloud count: 0
- Final optimized position error: ~0.17667 m vs raw ~0.69642 m (74.6% reduction)
- P4 evaluator: 179 streaming poses, 0 jumps, 0 nonmonotonic timestamps after duplicate-stamp fix
- Note: evaluator streaming RMSE (~0.396 m) includes poses before loop optimization and should NOT be interpreted as the final optimized pose-graph RMSE. The final saved keyframe CSV provides the optimized trajectory.

Results from P4 fixture:
`results/stage4/fixture_maps/stage4_map_*.pcd`
`results/stage4/fixture_maps/stage4_poses_*.csv`
`results/stage4/fixture_optimized.summary.yaml`

## P3 raw rosbag -> P4 acceptance

Replayed `/tmp/p3_bags/synthetic_lio_20261008_080652` using `stage4_bag.launch.py`.
Only raw /livox/lidar, /livox/imu and optional GT are replayed from MCAP; previously recorded LIO odometry/TF are not replayed.
P3 FAST-LIO recomputed output, P4 built the map from P3 registered point clouds.

- P4 keyframes: 4; loops: 0; rejected loops: 0; dropped cloud count: 0.
- Correctly did not detect a loop on the 8-second open synthetic route.
- Corrected odometry: 77 poses, 0 trajectory jumps, 0 nonmonotonic stamps.
- GT translation RMSE: ~0.02693 m on the open-route synthetic replay.
- P4 saved map and pose CSV successfully through /p4/save_map.
- All ROS2 processes exit cleanly.
- Separate evaluator folders:
  - `results/stage3/runs/bag_20261008_084836/`
  - `results/stage4/runs/20261008_084836/`

## Synthetic full-stack launch

`stage4_synthetic.launch.py rviz:=false` started synthetic Livox-like PointCloud2, 200 Hz synthetic IMU, FAST-LIO, P3/P4 evaluators, diagnostics, and P4 pose graph.
FAST-LIO initialized IMU, P4 created keyframes, no false loop; all processes exited normally with launch code 0.

## RViz

`stage4.launch.py start_sensor:=false start_lio:=false rviz:=true`
ran under an isolated virtual display:
- OpenGL 4.5 / GLSL 4.5 initialized
- `map` fixed frame; corrected path and corrected map topics configured.
- RViz, P4 node and evaluator all exited cleanly with launch exit code 0.
- Real desktop monitor with live Livox remains to be evaluated.

## Remaining hardware / product limitations

Physical Livox MID-360S was OFF during these tests.

No real field accuracy claim is made. LiDAR/IMU extrinsic (current identity placeholder), IMU stationary drift, outdoor accuracy, loop false-positive rate on repetitive structures, CPU/memory for long routes, IMU clock synchronization, and true map consistency remain to be proven on actual sensor data.

P4 supports **mapping and exporting the corrected map**, not loading a map and relocalizing robot pose from an existing map. Persistent map loading, localization/recovery and integration with Nav2 belong in the next stage.

On real data: validate P3 calibration first, record a route with at least one clear revisit, compare raw LIO path vs corrected graph trajectory and map, verify PCD against independent ground truth, and inspect all rejected and accepted loop closures before autonomous operation.
