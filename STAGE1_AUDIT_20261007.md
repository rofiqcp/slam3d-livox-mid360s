# Stage 1 Audit — 2026-10-07

Workspace:
- /media/sirobo/HDD/slam3d-livox-mid360s

Status:
- Stage 1 implementation: COMPLETE
- Offline validation: COMPLETE
- Live hardware acceptance: PENDING (MID-360S currently powered off)

## Build

Final incremental build:
- livox_mid360_manager: PASS
- livox_ros_driver2: PASS
- total: 2/2 packages

Manager code is compiled with:
- -Wall
- -Wextra
- -Wpedantic

## Installed C++ executables

- livox_discover
- livox_network_setup
- livox_monitor
- livox_diagnostics
- imu_stationary_stats
- calibration_prepare

## Launch files

- sensor.launch.py
- stage1.launch.py
- static_tf.launch.py
- record.launch.py
- play.launch.py

## Persistent configs

- config/livox_runtime.yaml
- config/extrinsics.yaml
- config/stage1.yaml
- config/li_init_mid360_template.yaml

Real hardware files intentionally NOT generated while sensor is powered off:
- config/imu_stationary.yaml
- config/li_init_mid360_generated.yaml

## Diagnostics validation

With sensor powered off:
- /diagnostics published successfully
- Livox MID360S/Overall = ERROR
- PointCloud = no data
- IMU = no data
- cloud measured rate = 0 Hz
- IMU measured rate = 0 Hz

Diagnostics include:
- rate
- arrival jitter
- sensor timestamp jitter
- non-monotonic stamp count
- large gap count
- stale timeout
- message count
- point count
- frame IDs
- Livox raw units

## IMU stationary statistics validation

Synthetic test:
- acceleration = [0, 0, 1] g
- gyro = [0.001, 0.002, 0.001] rad/s
- measured accel norm = 1.0 g
- generated conversion factor = 9.80665 m/s^2 per raw g-unit

Synthetic sample count was deliberately below its test threshold, so stationary_ok=false was expected.

Production setting:
- duration = 10 s
- min samples = 1000
- output = config/imu_stationary.yaml

## Livox units verified from driver source

- gyroscope: rad/s
- accelerometer: g

The official ROS2 driver copies these values directly into sensor_msgs/Imu.

## Extrinsic policy

- initial base_link -> livox_frame = zero transform
- status = uncalibrated
- static TF publishing is OFF by default
- publish_tf:=true must only be used after mounting measurement/calibration is trusted

## rosbag2 validation

Record preset:
- storage = MCAP
- compression = Zstd
- split size default = 4096 MiB
- topics = /livox/lidar, /livox/imu, /tf, /tf_static, /diagnostics

End-to-end synthetic recording test:
- valid .mcap.zstd file created
- metadata.yaml created
- ros2 bag info succeeded
- 52 sensor_msgs/Imu messages captured

Playback:
- compressed MCAP decompressed automatically
- rate:=2.0 tested
- clock:=true tested
- playback exited cleanly

## Calibration preparation

calibration_prepare:
- reads measured accel_norm_mean_g from config/imu_stationary.yaml
- writes config/li_init_mid360_generated.yaml
- does not use a guessed mean_acc_norm

Compatibility finding:
- upstream LiDAR_IMU_Init is ROS1
- for lidar_type=1 it subscribes to livox_ros_driver/CustomMsg
- this ROS2 workspace currently publishes sensor_msgs/PointCloud2 and sensor_msgs/Imu
- therefore a verified ROS2 port or timing-preserving adapter is required before using upstream LI-Init directly

## Process hygiene

Smoke-test child processes left by external timeout wrappers were found during audit and terminated.
Final process check:
- no livox_ros_driver2_node from tests
- no livox_mid360_diagnostics from tests
- no imu_stationary_stats from tests
- no RViz test process
- no rosbag recorder test process

## Hardware acceptance still required

After powering MID-360S:
1. run live discovery and save actual IP
2. confirm /livox/lidar rate
3. confirm /livox/imu rate
4. verify sensor timestamp monotonicity and jitter
5. verify RViz live point cloud
6. collect 10 s real stationary IMU baseline
7. require stationary_ok=true
8. record a real calibration MCAP dataset
9. generate calibration parameters from the real baseline
10. only then proceed toward LiDAR-IMU extrinsic/time calibration and Stage 2
