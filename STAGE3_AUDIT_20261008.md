# P3 / Stage 3 Audit — 2026-10-08

Workspace: /media/sirobo/HDD/slam3d-livox-mid360s

## Status

- Software implementation: COMPLETE for primary FAST-LIO baseline
- Full ROS2 workspace build: PASS — 4/4 packages
- Synthetic LiDAR + IMU integration: PASS
- Real-sensor bag recording with synthetic inputs: PASS
- Raw sensor rosbag replay + re-estimation: PASS
- RViz startup and clean shutdown: PASS
- Physical MID-360S acceptance: PENDING (sensor power off)
- Physical LiDAR-to-IMU extrinsic/time calibration: PENDING
- Point-LIO / GLIM / LIO-SAM comparison: NOT YET INTEGRATED

## Four ROS2 packages

- livox_ros_driver2
- livox_mid360_manager
- kiss_icp
- fast_lio

Use `./build.sh`. It scans `src/` only and sets Release mode consistently. Offline MAD-ICP and CT-ICP remain outside the ROS2 colcon build.

## Provenance

FAST-LIO ROS2 derived from:
`Referensi-slam3d/02_LIDAR_INERTIAL_LIO/FAST_LIO_SLAM_Livox` HEAD d139eed.

IKD-Tree (missing submodule in chosen upstream fork) derived from:
`Referensi-slam3d/02_LIDAR_INERTIAL_LIO/FAST_LIO_SAM_ROS2/include/ikd-Tree` HEAD c418013.

Files copied into `src/fast_lio/` and modified for the MID-360S ROS2 PointCloud2 interface.

## Necessary source fixes

- preprocess.lidar_type changed to 4, matching enum MID360 in C++ (reference config used 5).
- MID360 PointCloud2 parser uses x/y/z/intensity FLOAT32 and timestamp FLOAT64, validates buffer bounds and finite values.
- Per-point nanosecond timestamps converted to relative milliseconds for FAST-LIO deskew/curvature; sorted by time.
- Removed inaccurate azimuth-derived point-time estimation for non-repetitive MID-360S scanning.
- LiDAR and IMU subscriptions use SensorDataQoS (Best Effort compatible).
- FAST-LIO log and PCD output paths are generated via CMake and placed in build/fast_lio/Log and build/fast_lio/PCD.
- PointCloud2 xfer_format=0 retained, preserving P2 KISS-ICP compatibility.
- P3 ROS outputs remapped to /lio/odometry, /lio/path, /lio/cloud_registered, /lio/cloud_body, /lio/map.
- TF frame names standardized as lio_odom and livox_frame.
- ROS2 FAST-LIO IMU normalization accounts for raw Livox acceleration in g; gyro rad/s.

## PCL ABI fix

Originally FAST-LIO binary linked both PCL 1.12 and PCL 1.15.
CMake PCL_DIR is now pinned to /usr/lib/x86_64-linux-gnu/cmake/pcl for both fast_lio and livox_ros_driver2.
Clean rebuild passed for both packages.
Final ldd on FAST-LIO: 8 PCL 1.12 libraries, zero PCL 1.15 libraries.

## Synthetic live integration

With stage3_synthetic.launch.py:
- Livox-style PointCloud2: 10 Hz
- Synthetic IMU: 200 Hz
- FAST-LIO initialized the IMU and IKD-Tree.
- Stable /lio/odometry: about 10 Hz.
- poses = 239
- path_length_m = 12.0063442
- z_span_m = 0.001438268524
- jump_count = 0
- nonmonotonic_stamp_count = 0
- translation_rmse_vs_synthetic_gt_m = 0.0227285548

Output: results/stage3/runs/synthetic_qos_fixed_20261008/

## MCAP/Zstd recording

Recorded from stage3_synthetic and record_stage3:
- File: /tmp/p3_bags/synthetic_lio_20261008_080652
- Bag duration: 8.091823645 seconds
- 7.5 MiB compressed bag
- total messages: 2049
- /livox/lidar 81
- /livox/imu 1620
- /lio/odometry 81
- /lio/cloud_registered 81
- /lio/path 8
- /stage3/synthetic/ground_truth 81
- /diagnostics 16
- /tf 81

Verified via ros2 bag info. Recorder and synthetic processes returned exit status 0.

## Bag replay without ground-truth leakage

stage3_bag.launch.py rejects bags missing non-empty /livox/lidar or /livox/imu.
Playback explicitly filters topics to raw LiDAR and IMU, with optional ground truth; previously recorded /lio/odometry and /tf are NOT replayed, so odometry is recalculated.

Original replay:
- poses = 77
- jump_count = 0
- nonmonotonic_stamp_count = 0
- translation_rmse_vs_synthetic_gt_m = 0.02635234794

Final regression after system PCL 1.12 rebuild:
- poses = 77
- path_length_m = 3.837339484
- z_span_m = 0.0008908728454
- jump_count = 0
- nonmonotonic_stamp_count = 0
- translation_rmse_vs_synthetic_gt_m = 0.026882103
- all ROS2 processes exited cleanly

Output: results/stage3/runs/bag_pcl112_regression_20261008/

## RViz

Fixed Frame: lio_odom
Topics:
- /lio/cloud_registered
- /lio/map
- /lio/path

Virtual X display test:
- OpenGL 4.5 / GLSL 4.5
- RViz process started successfully
- Correct shutdown order (ROS SIGINT before Xvfb termination)
- ROS launch exit code 0
- RViz, FAST-LIO, diagnostics and evaluator all exited cleanly.

The physical Ubuntu desktop visualization and real sensor cloud are not hardware-validated.

## Critical limitations / hardware acceptance

Power was off throughout P3 software tests.
Current mapping.extrinsic_T = [0,0,0] and R = identity are uncalibrated and only suitable as synthetic/software baselines.
Do NOT claim centimeter-level real rover positioning based on the synthetic RMSE.

When powered, required:
1. rediscover real Livox IP and confirm UDP data packets;
2. verify raw cloud rate, IMU rate and timestamp monotonicity;
3. capture stationary IMU baseline for at least 10 seconds;
4. measure/validate sensor extrinsic transform and time offset;
5. run FAST-LIO and inspect gravity alignment and stationary Z stability;
6. drive a controlled route, record MCAP with raw cloud+IMU and LIO outputs;
7. replay the same raw data to verify repeatability;
8. compare KISS-ICP vs FAST-LIO and optionally Point-LIO on the same route;
9. only after those checks proceed to loop closure and persistent 3D mapping.
