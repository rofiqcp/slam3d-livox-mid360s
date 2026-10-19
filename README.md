# slam3d-livox-mid360s

Clean ROS 2 Humble C++ workspace for Livox MID-360 / MID-360S bring-up.

## What is included

- `livox_mid360_manager`: new C++ package for discovery, network setup and runtime monitoring.
- `livox_ros_driver2`: clean ROS 2 copy of the official Livox driver source.
- Persistent runtime YAML: `config/livox_runtime.yaml`.
- One launch path for driver + monitor + optional RViz.
- No old build/install/log trees were copied from the research library.

The initial fallback reproduces the previously working setup:

- interface: `enp4s0`
- host: `192.168.1.10/24`
- MID-360S: `192.168.1.197`
- points: `/livox/lidar`
- IMU: `/livox/imu`
- frame: `livox_frame`

The LiDAR IP is not hard-coded in C++. It lives in YAML and is replaced when live discovery succeeds.

## Build

```bash
cd /media/sirobo/HDD/slam3d-livox-mid360s
./build.sh
source env.sh
```

## Network check/setup

Check whether the host IP from YAML is already present:

```bash
ros2 run livox_mid360_manager livox_network_setup
```

Current validated setup should report:

```text
OK: enp4s0 already owns 192.168.1.10/24
```

If the sensor is connected through another Ethernet interface, a Linux bridge, or Wi-Fi, the same executable can target that interface:

```bash
sudo -E /opt/ros/humble/bin/ros2 run livox_mid360_manager livox_network_setup \
  --interface br0 --host-ip 192.168.1.10 --prefix 24 --apply
```

Replace `br0` with the actual interface. Generic bridges and Wi-Fi interfaces are supported; Docker/Tailscale/veth interfaces are ignored by automatic discovery.

## Discover Livox and save the real IP

With the LiDAR powered:

```bash
source /media/sirobo/HDD/slam3d-livox-mid360s/env.sh
ros2 run livox_mid360_manager livox_discover --interface auto --timeout 5
```

The C++ node:

1. enumerates active IPv4 Ethernet/bridge/Wi-Fi interfaces;
2. prioritizes addresses on `192.168.1.0/24`;
3. uses Livox SDK2 broadcast discovery on each candidate host IP;
4. identifies the Livox model, serial number and real LiDAR IP;
5. writes the verified interface, host IP and LiDAR IP to `$LIVOX_MID360_CONFIG`;
6. leaves the existing YAML untouched if no sensor is found.

To inspect every visible Livox instead of stopping at the first one:

```bash
ros2 run livox_mid360_manager livox_discover --interface auto --timeout 5 --all
```

## Run sensor + monitor + RViz

```bash
source /media/sirobo/HDD/slam3d-livox-mid360s/env.sh
ros2 launch livox_mid360_manager sensor.launch.py
```

The launch reads the YAML every start and generates a temporary Livox JSON configuration. No source file edit is required after discovery.

Driver only, no RViz:

```bash
ros2 launch livox_mid360_manager sensor.launch.py rviz:=false monitor:=false
```

## Runtime monitor

When `xfer_format: 0`, the C++ monitor reports:

- PointCloud2 rate;
- age of the latest cloud;
- points in the latest cloud;
- frame ID;
- IMU rate;
- age of the latest IMU.

Expected topics:

```bash
ros2 topic list | grep livox
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

## Runtime YAML

Persistent configuration:

```text
/media/sirobo/HDD/slam3d-livox-mid360s/config/livox_runtime.yaml
```

It contains network, device, driver and port settings. Discovery only replaces it after a Livox device is actually found.

## Current offline validation

Validated on Ubuntu 22.04 / ROS 2 Humble:

- clean build succeeds;
- official driver builds;
- C++ discovery executable runs;
- C++ network checker validates `enp4s0/192.168.1.10`;
- with LiDAR powered off, discovery returns no-device without modifying YAML;
- launch generates valid MID-360S driver JSON;
- driver reports `Init lds lidar success!`;
- monitor remains alive and correctly reports 0 Hz while the sensor is powered off.

Live packet/topic/RViz point-cloud validation must be repeated after the LiDAR is powered.


## Stage 1 workflow

Stage 1 recording, diagnostics, TF baseline, stationary IMU statistics and calibration preparation are documented in STAGE1.md.

Main command:

    source /media/sirobo/HDD/slam3d-livox-mid360s/env.sh
    ros2 launch livox_mid360_manager stage1.launch.py

Record a reproducible MCAP/Zstd dataset from a second terminal:

    ros2 launch livox_mid360_manager record.launch.py

Run the 10-second stationary IMU baseline while the assembly is completely still:

    ros2 run livox_mid360_manager imu_stationary_stats

Then generate measured calibration parameters:

    ros2 run livox_mid360_manager calibration_prepare

Static base_link -> livox_frame publishing is disabled by default until extrinsics are calibrated.


## Stage 2 workflow

LiDAR-only odometry baseline is documented in STAGE2.md.

Primary live ROS2 baseline:

    source /media/sirobo/HDD/slam3d-livox-mid360s/env.sh
    ros2 launch livox_mid360_manager stage2.launch.py

Replay a recorded bag through KISS-ICP:

    ros2 launch livox_mid360_manager stage2_bag.launch.py bag:=/path/to/bag

Export identical PointCloud2 scans for offline algorithms:

    ros2 launch livox_mid360_manager stage2_export.launch.py output_dir:=/path/to/dataset

Run MAD-ICP on the exported dataset:

    tools/run_mad_icp.sh /path/to/dataset mad_icp_mid360s

KISS-ICP is the live ROS2 baseline. MAD-ICP is the validated offline comparison. CT-ICP remains isolated because the available reference is ROS1/catkin with a legacy remote superbuild.


## Stage 2 LiDAR-only odometry

Stage 2 is implemented and synthetic-validated. Full instructions and measured results are in STAGE2.md.

Live default:

    source /media/sirobo/HDD/slam3d-livox-mid360s/env.sh
    ros2 launch livox_mid360_manager stage2.launch.py

Record a benchmark route:

    ros2 launch livox_mid360_manager record_stage2.launch.py name:=mid360_stage2_route

Offline comparison backends:

    tools/stage2/run_mad_icp.sh
    tools/stage2/run_ct_icp.sh
    python3 tools/stage2/compare_results.py

Current decision:
- KISS-ICP: live ROS2 default
- MAD-ICP: offline accuracy cross-check
- CT-ICP: continuous-time research comparator

See:
- STAGE2.md
- STAGE2_AUDIT_20261008.md
- results/stage2/BACKEND_COMPARISON.md


## Stage 3 — FAST-LIO

FAST-LIO ROS2 for MID-360S uses timestamp-aware PointCloud2 and built-in IMU. See STAGE3.md and STAGE3_AUDIT_20261008.md for complete details.

    cd /media/sirobo/HDD/slam3d-livox-mid360s
    ./build.sh
    source env.sh
    ros2 launch livox_mid360_manager stage3_synthetic.launch.py rviz:=false
    ros2 launch livox_mid360_manager stage3.launch.py rviz:=true

Do not treat zero/identity LiDAR–IMU extrinsics as calibrated; real hardware acceptance is required.


## Stage 4 — Loop Closure and Pose Graph

P4 uses a separate ROS2 C++ \`slam3d_pose_graph\` package that consumes FAST-LIO odometry and registered clouds, verifies candidate loop closures by ICP, optimizes a GTSAM pose graph, publishes \`/p4/optimized_map\` and \`/p4/optimized_path\`, and exports a binary PCD plus CSV on \`/p4/save_map\`.

See STAGE4.md and STAGE4_AUDIT_20261008.md for full commands, tests, thresholds and hardware limitations.

    cd /media/sirobo/HDD/slam3d-livox-mid360s
    ./build.sh
    source env.sh
    ros2 launch slam3d_pose_graph loop_fixture.launch.py
    ros2 launch livox_mid360_manager stage4_synthetic.launch.py rviz:=true
    ros2 launch livox_mid360_manager stage4.launch.py rviz:=true

The real MID-360S must be powered, discovered and calibrated before claiming field-level map accuracy.


## Stage 5 — Saved Map Relocalization

Added ROS2 \`slam3d_relocalizer\`, loading explicit saved P4 PCD + matched trajectory CSV. P5 uses timestamp-synchronized P3 odometry and registered LiDAR clouds, does quality-gated ICP global relocalization/tracking, publishes \`/p5/localized_odometry\`, \`/p5/reference_map\`, \`/p5/status\` and \`map -> lio_odom\`, and offers reload/relocalize/save-pose services.

    cd /media/sirobo/HDD/slam3d-livox-mid360s
    ./build.sh
    source env.sh
    ros2 launch livox_mid360_manager stage5_fixture.launch.py rviz:=true
    ros2 launch livox_mid360_manager stage5.launch.py map_pcd:=/path/to/stage4_map.pcd poses_csv:=/path/to/stage4_poses.csv rviz:=true

P4 global TF publisher must be stopped when P5 is localizing against its saved map. The test fixture is deliberately synthetic; live sensor/extrinsic acceptance is still pending. See STAGE5.md and STAGE5_AUDIT_20261008.md.


## Stage 6 — Nav2 Bridge and Fail-Closed Motion Guard

P6 creates a separate ROS2 C++ package `slam3d_nav2_bridge` for the corrected TF tree `map -> lio_odom -> base_link -> livox_frame`, P3 `/odom` at base_link, P4 saved PCD projection to Nav2 `/map` OccupancyGrid, and an explicit software motion gate `/cmd_vel -> /cmd_vel_safe` with P5 status/TF/command watchdogs and software E-STOP.

    cd /media/sirobo/HDD/slam3d-livox-mid360s
    ./build.sh
    source env.sh
    ros2 launch slam3d_nav2_bridge stage6_fixture.launch.py
    ros2 launch slam3d_nav2_bridge stage6_nav2_fixture.launch.py

The software gate starts disarmed, sends zero Twist on LOST, E-STOP and timeouts, and requires operator re-ARM after interruption. Live Nav2 launch is refused while `config/extrinsics.yaml` is uncalibrated. No physical actuator connection, hardwired E-STOP or motor watchdog was tested. See STAGE6.md and STAGE6_AUDIT_20261008.md for commands, acceptance details and hardware blockers.


## Stage 7 — MID-360S Mapping Dashboard (C++)

An isolated ROS2 C++ package `slam3d_mid360_dashboard` adds modular `cloud_filter`, `map_preview` and `web_dashboard` nodes. It uses the existing P3 FAST-LIO / P4 GTSAM mapping backends, publishes `/mid360/map_preview` from corrected `/p4/optimized_map`, and serves the self-contained 3D point-cloud dashboard on [http://localhost:5001/](http://localhost:5001/). The automatic user service `mid360-dashboard.service` starts the P3/P4 pipeline, reuses a running Livox driver, or starts one if none exists after reboot. See [STAGE7.md](STAGE7.md) for architecture, setup, API, tests and hardware prerequisites.

```bash
source env.sh
ros2 pkg executables slam3d_mid360_dashboard
./tools/check_mid360_dashboard.sh
```

**Hardware note:** MID-360S was disconnected/unresponsive at Stage 7 acceptance (0 Hz LiDAR/IMU); saved maps tested in the P4 synthetic fixture only. LiDAR/base extrinsics remain uncalibrated; do not treat synthetic output as a field map.

## Foxglove resmi dan mapping MID-360S

Foxglove Bridge read-only pada ws://localhost:8765 menggunakan layanan mid360-foxglove.service. Layout JSON untuk 3D P4 map, IMU plots, diagnostik dan trajectory tersedia di foxglove/layouts/MID360S_mapping_calibration.json.

Dashboard internal ROS2/C++ http://localhost:5001/ berbeda dari Foxglove resmi; kini mendukung drag pan, Shift/klik-kanan drag orbit, zoom, tiga sumber PointCloud2 dan monitoring keyframe/loop/kalibrasi.

Kalibrasi IMU saat diam, hanya setelah sensor aktif: tools/calibrate_mid360_imu.sh --confirm-stationary. Tidak mengkalibrasi transform sensor terhadap base_link.

Profil high-detail riset: config/profiles/fast_lio_indoor_detail.yaml, config/profiles/p4_indoor_detail.yaml dan tools/run_mid360_detail_mapping.sh. Hentikan mid360-dashboard.service lebih dulu agar tidak ada duplicate FAST-LIO/P4 node; pulihkan service setelah pengujian.

Dokumentasi dan batas akurasi: FOXGLOVE_MAPPING_CALIBRATION_20261008.md
