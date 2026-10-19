# FAST-LIO ROS2 — Livox MID-360S

Workspace package: `src/fast_lio`

Source adapted from local `Referensi-slam3d/02_LIDAR_INERTIAL_LIO/FAST_LIO_SLAM_Livox` (Git HEAD `d139eed`). The source fork was missing its IKD-Tree submodule; the local matching ROS2 IKD-Tree was taken from `FAST_LIO_SAM_ROS2/include/ikd-Tree` (reference HEAD `c418013`).

## Integration contract

The primary live topics are `/livox/lidar` (PointCloud2) and `/livox/imu` (Imu). MID-360S driver stays in `xfer_format: 0` so the same raw data is compatible with KISS-ICP P2. The PointCloud2 processor explicitly reads x/y/z, intensity, and the Livox per-point timestamp (FLOAT64 nanoseconds) and converts the time to milliseconds relative to the first point.

ROS2 SensorDataQoS is used for both raw subscriptions to tolerate sensor publishers offering best-effort or reliable QoS.

This fork uses enum `MID360 = 4`; the configuration uses `preprocess.lidar_type: 4`, not 5. The LiDAR is non-repetitive; point timing is not inferred from azimuth.

LiDAR IMU reports gyro in rad/s and raw acceleration in g. The FAST-LIO IMU initialization internally normalizes the mean acceleration to gravity, so do not apply an additional blind 9.81 multiplier. Real IMU stationary validation is still required.

## Outputs

`/lio/odometry`, `/lio/path`, `/lio/cloud_registered`, `/lio/cloud_body`, `/lio/map`.

TF: `lio_odom -> livox_frame`.

## Config

`config/mid360s.yaml` is installed with this package. The project-level persistent override is `/media/sirobo/HDD/slam3d-livox-mid360s/config/stage3_fast_lio.yaml`. The top-level `stage3.launch.py` selects that override automatically.

The current identity extrinsic is an **uncalibrated baseline**. Physical LiDAR-to-IMU translation/rotation and time offset must be validated before claiming real positioning accuracy.

## Build

From workspace root:

    ./build.sh
    source env.sh
    ros2 launch livox_mid360_manager stage3_synthetic.launch.py rviz:=false

For live MID-360S, power the sensor and run discovery first:

    ros2 run livox_mid360_manager livox_discover --interface auto --timeout 5
    ros2 launch livox_mid360_manager stage3.launch.py rviz:=true

PCL dependency is explicitly pinned to Ubuntu/ROS2 Humble system PCL 1.12 to avoid mixing the system and locally installed PCL 1.15 ABI.

Log and optional PCD paths are under `build/fast_lio/`, never inside source. Map-file saving is disabled by default; persistent map export/localization is a later stage.
