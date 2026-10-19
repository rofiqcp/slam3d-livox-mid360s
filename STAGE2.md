# Stage 2 — LiDAR-only Odometry Baseline

Status: **implemented and synthetic-validated**.

Live MID-360S acceptance remains pending because the sensor is currently powered off.

## Goal

Stage 2 isolates LiDAR geometry and scan registration from IMU, ESKF and LIO effects.

Three reference backends are implemented:

1. **KISS-ICP** — native ROS 2 live baseline and the default backend.
2. **MAD-ICP** — C++ offline accuracy/performance cross-check.
3. **CT-ICP** — C++ continuous-time research comparator.

## Input contract

The live baseline uses:

- topic: `/livox/lidar`
- type: `sensor_msgs/PointCloud2`
- Livox driver: `xfer_format=0`
- frame: `livox_frame`

Important Livox PointCloud2 fields preserved by this workspace:

- `x`, `y`, `z`: FLOAT32
- `intensity`: FLOAT32
- `tag`: UINT8
- `line`: UINT8
- `timestamp`: FLOAT64

KISS-ICP recognizes the `timestamp` field directly, so scan deskew stays enabled.

## KISS-ICP — live ROS 2 default

Local package:

```text
src/kiss_icp
```

Reference source:

```text
Referensi-slam3d/01_LIDAR_ODOMETRY_SLAM/kiss-icp
commit 1ffa7d7512f10bfc8b1185095011fa31184019e3
v1.3.0-1-g1ffa7d7
```

The package is vendored locally and does not use FetchContent during build.

Main outputs:

- `/kiss/odometry`
- `/kiss/frame`
- `/kiss/keypoints`
- `/kiss/local_map`
- `/kiss/processing_time_ms`

TF convention:

```text
odom_lidar -> livox_frame
```

The Stage 2 baseline intentionally does not require `base_link` or IMU/extrinsic calibration.

### Live command

After the sensor is powered and discovery is valid:

```bash
cd /media/sirobo/HDD/slam3d-livox-mid360s
source env.sh

ros2 launch livox_mid360_manager stage2.launch.py \
  start_sensor:=true \
  rviz:=true \
  deskew:=true
```

### Bag replay

Run KISS against a recorded bag without starting the physical sensor:

```bash
ros2 launch livox_mid360_manager stage2_bag.launch.py \
  bag:=/path/to/bag \
  rviz:=true
```

## Stage 2 evaluator

Executable:

```text
stage2_odom_evaluator
```

It monitors:

- odometry frequency;
- stale odometry;
- non-monotonic timestamps;
- per-frame translation and rotation jumps;
- path length;
- displacement;
- Z span;
- optional translation error against ground truth;
- CSV trajectory output;
- diagnostic status.

Synthetic outputs:

```text
results/stage2/kiss_icp_odometry.csv
results/stage2/kiss_icp_odometry.summary.yaml
```

## Recording a benchmark route

Use a second terminal while Stage 2 is running:

```bash
ros2 launch livox_mid360_manager record_stage2.launch.py \
  name:=mid360_stage2_route
```

Recorded topics include:

- `/livox/lidar`
- `/livox/imu`
- `/kiss/odometry`
- `/kiss/processing_time_ms`
- `/tf`
- `/tf_static`
- `/diagnostics`

Optional KISS debug clouds can also be recorded.

## MAD-ICP offline backend

Local backend:

```text
external/mad_icp
```

It is built as a lean C++ backend with Eigen, OpenMP and yaml-cpp.

The ROS cloud is exported to KITTI XYZI:

```bash
ros2 launch livox_mid360_manager stage2_export.launch.py \
  topic:=/livox/lidar \
  output_dir:=/media/sirobo/HDD/slam3d-livox-mid360s/datasets/stage2/kitti_scans
```

Then run:

```bash
tools/stage2/run_mad_icp.sh \
  datasets/stage2/kitti_scans/scans \
  results/stage2/mad_icp_run
```

MAD-ICP is not presented as a live ROS2 node in this workspace. It is an offline independent cross-check.

## CT-ICP continuous-time backend

Local backend:

```text
external/ct_icp
```

CT-ICP now builds successfully on Ubuntu 22.04 after local compatibility work.

The CT exporter preserves per-point timestamp:

```bash
ros2 launch livox_mid360_manager stage2_export_ct.launch.py \
  topic:=/livox/lidar \
  output_root:=/media/sirobo/HDD/slam3d-livox-mid360s/datasets/stage2/ct_ply
```

The output layout is:

```text
ct_ply/
  00/
    frames/
      frame_0000.ply
      frame_0001.ply
      ...
```

Each PLY vertex contains:

```text
x float
y float
z float
timestamp double
```

Timestamp is converted to seconds and normalized relative to the first exported point.

Run:

```bash
tools/stage2/run_ct_icp.sh \
  datasets/stage2/ct_ply \
  results/stage2/ct_icp_run
```

The wrapper internally maps sequence `00` to the CT KITTI_CARLA container. This is deliberate: the CT KITTI/KITTI_raw loader applies a Velodyne-specific 0.205 degree correction that must not be applied to MID-360S.

## Build all Stage 2 backends

```bash
tools/stage2/build_backends.sh
```

This builds:

- ROS 2 workspace;
- KISS-ICP ROS2;
- MAD-ICP C++ backend;
- CT-ICP C++ backend.

## Synthetic acceptance benchmark

The synthetic publisher uses the same MID-360S PointCloud2 field contract and publishes known ground truth.

KISS was tested live through ROS2 with deskew enabled.

MAD used 100 exported KITTI XYZI scans.

CT used 100 PLY scans with 8,907 unique per-point timestamps per scan and a 0.1 s timestamp span.

Final comparison:

| Backend | Integration | Translation RMSE | Z span/max | Mean runtime | 10 Hz real-time | Live ROS2 |
|---|---|---:|---:|---:|---|---|
| KISS-ICP | native ROS2 + timestamp | 3.12 cm | 0.18 cm | 6.53 ms | YES | YES |
| MAD-ICP | C++ offline XYZI | 2.38 cm | 0.10 cm | 21.25 ms | YES | NO |
| CT-ICP | C++ continuous-time PLY | 17.59 cm | 30.12 cm | 142.90 ms | NO | NO |

KISS p95 callback time:

```text
7.16 ms
```

KISS maximum measured callback:

```text
7.65 ms
```

MAD 100-scan wall time after OpenMP was enabled:

```text
2.16 s
```

CT 100-scan wall time:

```text
14.29 s
```

Comparison files:

```text
results/stage2/BACKEND_COMPARISON.yaml
results/stage2/BACKEND_COMPARISON.md
```

Regenerate:

```bash
python3 tools/stage2/compare_results.py
```

## Current backend decision

### Default live backend: KISS-ICP

Reasons:

- native ROS2;
- directly consumes Livox PointCloud2;
- directly recognizes per-point Livox timestamp;
- low latency;
- zero synthetic trajectory jumps;
- zero non-monotonic odometry timestamps;
- low Z span.

### Offline cross-check: MAD-ICP

MAD produced the lowest translation RMSE in this synthetic benchmark and is fast enough for 10 Hz processing.

### CT-ICP role

CT-ICP is now a working research backend, not an unbuilt legacy reference. However, its current reference tuning was slower than the 10 Hz budget and showed more Z drift on this synthetic scene. It is therefore not promoted to live default yet.

## Live hardware acceptance criteria

When MID-360S is powered:

1. discovery saves the real LiDAR and host interface;
2. Stage 1 diagnostics reports healthy cloud stream;
3. `/livox/lidar` remains near configured 10 Hz;
4. `/kiss/odometry` remains near cloud rate;
5. KISS p95 processing time remains comfortably below 100 ms;
6. no non-monotonic odometry timestamps;
7. no large pose jumps during normal rover motion;
8. stationary robot does not accumulate significant Z/path drift;
9. a repeatable route is recorded to MCAP/Zstd;
10. the same route is exported to MAD and CT for offline comparison.

Only after live Stage 2 passes should Stage 3 LIO be promoted as the next baseline.


## Additional P2 acceptance — October 8, 2026

- ROS2 build only scans src; all 3 packages pass (KISS-ICP, manager, Livox ROS2 driver).
- Synthetic live baseline: 154 poses, approximately 10 Hz, zero jumps, translational RMSE 0.03112 m.
- Synthetic rosbag replay: 35/35 poses, RMSE 0.03341 m, zero jumps and no non-monotonic timestamps.
- Bag playback starts after a 4-second subscription startup delay.
- Result files are isolated per run under results/stage2/runs; override by passing result_dir:=PATH.
- Stage2 recording supports input_topic:= and ground_truth_topic:=.
- End-to-end MCAP/Zstd test succeeded: 252 messages in 6.1 s, including 62 synthetic point clouds, 61 odometry outputs, 61 KISS processing time samples, 62 ground truth poses, and 6 diagnostics. Both processes exited 0.
- Livox is still powered off. Real point-cloud quality, robot drift and outdoor performance remain unverified.

Replay:
  source /media/sirobo/HDD/slam3d-livox-mid360s/env.sh
  ros2 launch livox_mid360_manager stage2_bag.launch.py bag:=/path/to/bag startup_delay:=4.0 rviz:=true

Synthetic acceptance:
  ros2 launch livox_mid360_manager stage2_synthetic.launch.py rviz:=false result_dir:=/path/to/results
