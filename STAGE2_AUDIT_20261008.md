# Stage 2 Audit — 2026-10-08

Workspace:
- /media/sirobo/HDD/slam3d-livox-mid360s

Status:
- implementation: COMPLETE
- synthetic acceptance: COMPLETE
- live MID-360S acceptance: PENDING (sensor powered off)

## ROS2 packages

Full colcon workspace:
- livox_ros_driver2
- livox_mid360_manager
- kiss_icp

KISS is vendored offline from reference commit:
- 1ffa7d7512f10bfc8b1185095011fa31184019e3
- v1.3.0-1-g1ffa7d7

No network FetchContent is required for KISS build.

## KISS integration

Input:
- /livox/lidar
- sensor_msgs/PointCloud2
- xfer_format=0
- timestamp FLOAT64 used for deskew

Outputs:
- /kiss/odometry
- /kiss/frame
- /kiss/keypoints
- /kiss/local_map
- /kiss/processing_time_ms

Synthetic 100-pose accuracy window:
- translation RMSE: 0.03121 m
- Z span: 0.00184 m
- max pose step: 0.08438 m
- max angular step: 0.00464 rad
- jumps: 0
- non-monotonic stamps: 0

Timing after warm-up:
- samples: 105
- mean: 6.53 ms
- median: 6.46 ms
- p95: 7.16 ms
- max: 7.65 ms

## MAD-ICP

Local:
- external/mad_icp

C++ build:
- Eigen
- yaml-cpp
- OpenMP
- no Python binding required for benchmark

OpenMP is linked to the odometry target.

100 synthetic scans:
- translation RMSE: 0.02376 m
- yaw RMSE: 0.0711 deg
- max Z absolute: 0.00105 m
- mean algorithm time: 21.25 ms
- max algorithm time: 24.91 ms
- wall time: 2.16 s
- CPU utilization: ~711 percent

## CT-ICP

Local:
- external/ct_icp

Build result:
- SlamCore: PASS
- CT_ICP: PASS
- run_odometry: PASS

Ubuntu 22 compatibility/local changes:
- uses system Eigen/Ceres/glog/yaml-cpp/OpenMP/nanoflann
- installed libcereal-dev, libtclap-dev, robin-map-dev
- tinyply sourced from local Gaussian-LIC reference
- visualization disabled
- Python binding disabled
- tests disabled
- modern CMake imported-target LOCATION compatibility patched
- install/export disabled for benchmark build
- SIGSTKSZ static-array issue replaced with 64 KiB alternate stack
- sigaltstack size uses sizeof(alternate_stack)

CT data path:
- PointCloud2 -> PLY x/y/z/timestamp
- timestamp seconds normalized to first exported point
- 8,907 unique per-point timestamps per synthetic scan
- ~0.100 s timestamp span per scan
- KITTI_CARLA loader used only as a PLY container to avoid the KITTI Velodyne 0.205 degree correction

100 synthetic scans:
- translation RMSE: 0.17590 m
- yaw RMSE: 0.1570 deg
- max relative Z: 0.30116 m
- wall time: 14.29 s
- mean wall budget: 142.9 ms/frame
- CPU utilization: ~585 percent
- max RSS: 61,820 KiB

Conclusion:
- CT builds/runs successfully, but current tuning is not selected for 10 Hz live operation.

## Dataset exporters

MAD:
- cloud_to_kitti
- float32 XYZI
- validated with 100 scans x 8,907 points

CT:
- cloud_to_ct_ply
- PLY x/y/z/timestamp
- validated with 100 scans x 8,907 points
- per-point timestamps preserved

## Synthetic result files

- results/stage2/kiss_icp_odometry.csv
- results/stage2/kiss_icp_odometry.summary.yaml
- results/stage2/mad_icp_synthetic/estimate.txt
- results/stage2/mad_icp_synthetic/mad_icp_synthetic.summary.yaml
- results/stage2/ct_icp_synthetic/KITTI_CARLA_Town01.PLY
- results/stage2/ct_icp_synthetic/ct_icp_synthetic.summary.yaml
- results/stage2/BACKEND_COMPARISON.yaml
- results/stage2/BACKEND_COMPARISON.md

## Decision

Live ROS2 default:
- KISS-ICP

Offline accuracy cross-check:
- MAD-ICP

Continuous-time research comparator:
- CT-ICP

## Remaining hardware acceptance

Pending only because MID-360S is powered off:
- live point cloud rate
- live KISS odometry rate
- live processing latency
- real rover stationary drift
- real route jump/dropout behavior
- real MCAP recording
- MAD/CT comparison using the exact same real route


## Latest P2 verification

- Root colcon list sees precisely 3 ROS2 packages after adding COLCON_IGNORE under external and third_party.
- build.sh now scopes colcon to src; full 3/3 package build verified.
- Replay of 35 scan synthetic rosbag yielded 35 KISS poses, no jumps, no non-monotonic timestamps, 0.033408754 m translation RMSE.
- MCAP + Zstd recorded 252 messages and passed ros2 bag info and graceful shutdown.
- Per-run result folders prevent CSV and summary from overwriting earlier benchmarks.
- Deleted duplicate uncompiled odom_metrics.cpp; stage2_odom_evaluator remains the authoritative evaluator.
- All live MID-360S hardware validation is still pending because the device is powered off.
