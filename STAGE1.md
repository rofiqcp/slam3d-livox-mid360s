# Stage 1 — Recording, Diagnostics and Calibration Baseline

Status: **implemented and offline-validated**.

Hardware-dependent acceptance remains pending because the Livox MID-360S is currently powered off.

## Components

### 1. Stream diagnostics

Executable:

```bash
ros2 run livox_mid360_manager livox_diagnostics
```

Normally use it through:

```bash
ros2 launch livox_mid360_manager stage1.launch.py
```

Publishes:

```text
/diagnostics
```

Checks both `/livox/lidar` and `/livox/imu` for:

- measured rate;
- expected rate;
- arrival jitter;
- sensor timestamp jitter;
- non-monotonic timestamps;
- large inter-message gaps;
- stale streams;
- total message count;
- latest cloud point count;
- frame ID.

Default targets:

- PointCloud: 10 Hz
- IMU: 200 Hz

The rates are parameters in `config/stage1.yaml` and should be confirmed from live hardware.

When the LiDAR is powered off, the verified behavior is:

- Overall diagnostic level = ERROR;
- PointCloud = no data;
- IMU = no data.

### 2. Stationary IMU baseline

Keep the complete LiDAR/IMU assembly motionless, then run:

```bash
source /media/sirobo/HDD/slam3d-livox-mid360s/env.sh
ros2 run livox_mid360_manager imu_stationary_stats
```

Default acquisition:

- 10 seconds;
- minimum 1000 IMU samples;
- output:
  `/media/sirobo/HDD/slam3d-livox-mid360s/config/imu_stationary.yaml`

The output contains:

- accelerometer mean/std per axis;
- gyroscope mean/std per axis;
- acceleration norm mean/std;
- gyro norm mean/std;
- sample count;
- sensor/wall duration;
- `stationary_ok`;
- `accel_multiplier_to_mps2`;
- measured `li_init_mean_acc_norm`.

Important Livox driver units verified from source:

- angular velocity = rad/s;
- linear acceleration = g.

Therefore a stationary Livox IMU near 1 g needs an approximately 9.80665 multiplier before an estimator that expects m/s².

### 3. Persistent extrinsics

File:

```text
config/extrinsics.yaml
```

It defines:

```text
base_link -> livox_frame
translation xyz [m]
rotation RPY [rad]
time offset [s]
calibration status/source
```

The initial values are intentionally zero and marked `uncalibrated`.

For safety, Stage 1 does **not** publish this zero transform by default.

Only after a verified mounting measurement/calibration:

```bash
ros2 launch livox_mid360_manager stage1.launch.py publish_tf:=true
```

### 4. rosbag2 recording

Use a second terminal while Stage 1 is running:

```bash
source /media/sirobo/HDD/slam3d-livox-mid360s/env.sh
ros2 launch livox_mid360_manager record.launch.py
```

Default:

- storage: MCAP;
- compression: Zstd;
- split size: 4096 MiB;
- output root: `bags/`;
- timestamped recording directory.

Recorded topics:

- `/livox/lidar`
- `/livox/imu`
- `/tf`
- `/tf_static`
- `/diagnostics`

Change split size if needed:

```bash
ros2 launch livox_mid360_manager record.launch.py max_bag_size_mb:=2048
```

Offline test result:

- MCAP + Zstd created successfully;
- metadata valid;
- synthetic test captured 52 IMU messages;
- `ros2 bag info` parsed it correctly.

### 5. Playback

```bash
ros2 launch livox_mid360_manager play.launch.py \
  bag:=/path/to/livox_stage1_YYYYMMDD_HHMMSS
```

Optional:

```bash
ros2 launch livox_mid360_manager play.launch.py \
  bag:=/path/to/bag rate:=0.5 clock:=true
```

Playback was tested against the compressed MCAP test bag and exited cleanly.

### 6. Calibration preparation

Reference repositories:

- `Referensi-slam3d/10_CALIBRATION_INITIALIZATION/LiDAR_IMU_Init`
- `Referensi-slam3d/10_CALIBRATION_INITIALIZATION/lidar_imu_calib`
- `Referensi-slam3d/10_CALIBRATION_INITIALIZATION/OA-LICalib`

Generate calibration parameters from the real stationary baseline:

```bash
ros2 run livox_mid360_manager calibration_prepare
```

Default input:

```text
config/imu_stationary.yaml
```

Default output:

```text
config/li_init_mid360_generated.yaml
```

The generated `mean_acc_norm` comes from the measured Livox IMU, not from a fixed guess.

### ROS1 compatibility warning

The upstream `LiDAR_IMU_Init` is ROS1. For `lidar_type: 1` it subscribes to:

```text
livox_ros_driver/CustomMsg
```

This ROS2 workspace currently uses:

```text
sensor_msgs/PointCloud2
sensor_msgs/Imu
```

Therefore the upstream LI-Init must **not** be run directly against this workspace as if message types were identical. A verified ROS2 port or timing-preserving message adapter is required before using it for final extrinsic/time calibration.

The Stage 1 rosbag and stationary outputs are intended to be the reproducible input dataset for that work.

## Recommended live acceptance procedure

When the MID-360S is powered:

### Terminal 1 — discover

```bash
source /media/sirobo/HDD/slam3d-livox-mid360s/env.sh
ros2 run livox_mid360_manager livox_discover --interface auto --timeout 5
```

### Terminal 2 — Stage 1

Keep TF publishing disabled until calibration:

```bash
source /media/sirobo/HDD/slam3d-livox-mid360s/env.sh
ros2 launch livox_mid360_manager stage1.launch.py \
  rviz:=true publish_tf:=false stationary_stats:=false
```

Check:

```bash
ros2 topic echo --once /diagnostics
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

### Terminal 3 — stationary baseline

Keep the robot completely still:

```bash
ros2 run livox_mid360_manager imu_stationary_stats
```

Require:

- `stationary_ok: true`;
- enough samples;
- acceleration norm stable;
- gyro norm low;
- timestamps monotonic.

### Terminal 4 — record calibration dataset

```bash
ros2 launch livox_mid360_manager record.launch.py name:=calibration_baseline
```

Record:
1. at least 5–10 seconds fully stationary;
2. controlled rotational excitation around multiple axes;
3. translation/rotation with enough 3D structure;
4. another stationary segment before stopping.

This produces the baseline needed for the calibration-port step and later LIO validation.
