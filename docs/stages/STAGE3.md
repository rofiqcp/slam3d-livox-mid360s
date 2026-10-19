# P3 — ROS 2 FAST-LIO for Livox MID-360S

Scope: LiDAR–inertial odometry (LIO) baseline; no loop closure or global map persistence is claimed.

## Packages and provenance

- Driver: src/livox_ros_driver2
- Live network/discovery/diagnostics/evaluation/recording: src/livox_mid360_manager
- LiDAR-only reference: src/kiss_icp
- LIO: src/fast_lio, adapted from local Referensi-slam3d/02_LIDAR_INERTIAL_LIO/FAST_LIO_SLAM_Livox
- IKD-Tree sourced from local FAST_LIO_SAM_ROS2 reference because its submodule was absent in the selected ROS2 fork.

The FAST-LIO source is ROS2 and C++. No ROS1 nodes are required in the main colcon workspace.

## Critical MID-360S integration repairs

1. preprocess.lidar_type = 4 to match the actual source enum MID360.
2. PointCloud2 processing reads x/y/z/intensity FLOAT32 and timestamp FLOAT64 directly.
3. Livox point timestamps in nanoseconds are converted to relative milliseconds for FAST-LIO curvature/undistortion. No azimuth-derived timestamp substitution.
4. LiDAR and IMU subscribers use SensorDataQoS to accept sensor publishers offering best effort or reliable.
5. FAST-LIO gravity initialization uses measured accelerometer mean; raw Livox accelerometer g units must not be multiplied twice by 9.81.
6. Logs and PCD files are under the build/fast_lio directory, not source and not root system directories.
7. Output frame names: lio_odom and livox_frame.

## Input / output

- /livox/lidar: sensor_msgs/PointCloud2, xfer_format=0, 10 Hz expected
- /livox/imu: sensor_msgs/Imu, rate to be established from live hardware
- /lio/odometry: nav_msgs/Odometry
- /lio/path: nav_msgs/Path
- /lio/cloud_registered: sensor_msgs/PointCloud2
- /lio/cloud_body: sensor_msgs/PointCloud2
- /lio/map: sensor_msgs/PointCloud2
- /lio/evaluation_path: nav_msgs/Path from evaluator
- /diagnostics: stream/watchdog and LIO evaluator

## Build

    cd /media/sirobo/HDD/slam3d-livox-mid360s
    ./build.sh
    source env.sh

The ROS2 build only scans src/. MAD-ICP and CT-ICP remain separate offline backends under external/.

## Live sensor

With MID-360S powered and connected:

    source /media/sirobo/HDD/slam3d-livox-mid360s/env.sh
    ros2 run livox_mid360_manager livox_discover --interface auto --timeout 5
    ros2 launch livox_mid360_manager stage3.launch.py rviz:=true

The host/lidar IP is read from config/livox_runtime.yaml, updated by discovery.
P3 LiDAR data retains PointCloud2 so P2 does not need to be reconfigured.

## Simulation

    ros2 launch livox_mid360_manager stage3_synthetic.launch.py rviz:=false

Synthetic LiDAR: 10 Hz MID-360S-like x/y/z/intensity/tag/line/timestamp.
Synthetic IMU: 200 Hz, gyro rad/s, accelerometer raw g.
Outputs use a unique results/stage3/runs/ directory.

Synthetic benchmark is not evidence of calibrated real-world performance.

## Replay

Replay requires a bag with BOTH /livox/lidar and /livox/imu:

    ros2 launch livox_mid360_manager stage3_bag.launch.py bag:=/path/to/rosbag rviz:=false

The bag launcher waits for subscriptions before playback.

## Record

In a separate terminal while running live LIO:

    ros2 launch livox_mid360_manager record_stage3.launch.py name:=fast_lio_route

Recording includes raw cloud, raw IMU, /lio/odometry, /lio/path, /lio/cloud_registered, diagnostics, TF and optional ground truth. Uses MCAP and Zstd with 4096 MiB rotation.

## Extrinsic and time calibration

The project currently contains an identity/zero *uncalibrated* LiDAR–IMU extrinsic in config/stage3_fast_lio.yaml. It is a controlled software/synthetic baseline ONLY.
For real use, replace mapping.extrinsic_T/R with the validated LiDAR-to-IMU transform; confirm direction, axes and metric units. Consider trusted online extrinsic estimation only after sufficient excitation and independent validation.
Do not claim precise stationary drift, gravity-aligned Z, or reliable robot pose until the powered sensor has been tested and the extrinsic verified.

Minimum live acceptance criteria:

- /livox/lidar and /livox/imu fresh with healthy /diagnostics
- stable FAST-LIO startup after at least five seconds stationary
- /lio/odometry actively published and frame IDs valid
- no jump, nonmonotonic stamps or runaway Z at rest
- repeatable rosbag replay showing same estimator behavior
- evaluation on a controlled rover route against reference data
- compare LIO to KISS-ICP on exactly the same route

## Future comparator stages

Point-LIO, GLIM, LIO-SAM and Voxel-SLAM remain optional comparisons after the FAST-LIO MID-360S baseline is stable on actual hardware. Loop closure and saved-map localization belong to the following stages.


## Validation results — October 8, 2026

**Compiled:** FAST-LIO ROS2 package and Stage3 synthetic IMU executable. MID-360S point timestamps are used for curvature/undistortion; all raw sensor subscribers use SensorDataQoS.

Synthetic live test (~25 s):
- 10 Hz PointCloud2, 200 Hz IMU, steady LIO ~10 Hz
- 239 LIO odometry poses, 12.0063 m path length
- 0.001438 m Z-span
- 0 jumps and 0 non-monotonic odometry stamps
- 0.02273 m translation RMSE against synthetic ground truth

Recorded MCAP/Zstd (~8.09 s, 7.5 MiB):
- 2049 messages
- 81 raw clouds, 1620 raw IMU measurements
- 81 LIO odometry, 81 registered clouds, 81 ground-truth poses
- 81 TF messages, 8 paths and 16 diagnostics
- both recorder and publisher exited with status 0

Raw-sensor-only replay from that bag:
- 77 recomputed LIO poses, 3.8367 m path
- 0.000881 m Z-span; 0 jumps; 0 non-monotonic stamps
- 0.02635 m translation RMSE vs synthetic ground truth
- recorded /lio/odometry and /tf were intentionally excluded from playback

RViz configuration:
- fixed frame lio_odom
- /lio/cloud_registered, /lio/map, /lio/path
- virtual-display smoke test started OpenGL 4.5; GUI behavior with actual physical monitor still untested.

No physical sensor, precision, stationary drift, timing synchronization, IMU calibration or calibrated LiDAR–IMU extrinsic has been verified.
