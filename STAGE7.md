# Stage 7 — MID-360S C++ Modular 3D Mapping Dashboard (2026-10-08)

## Intended role and design

ROS 2 Humble project: `/media/sirobo/HDD/slam3d-livox-mid360s`.

The new isolated, source-built C++ package `slam3d_mid360_dashboard` **does not reimplement FAST-LIO or loop closure**. It consumes the existing stack:

```text
MID-360S -> livox_ros_driver2 (the sole active driver)
    |-- /livox/lidar (PointCloud2 with Livox per-point timing) --+
    |-- /livox/imu ---------------------------------------------+--> FAST-LIO (P3)
    |-- /livox/lidar -> cloud_filter -> /mid360/points_filtered
                                                              P3 -> /lio/odometry, /lio/cloud_registered
                                                                   |
                                                              P4 pose graph/GTSAM, ICP loop verification
                                                                   |
                                                  /p4/optimized_map [frame=map, corrected 3D]
                                                                   |
                                                map_preview -> /mid360/map_preview (bounded, latched)
                                                                   |
                         web_dashboard C++ ROS node, HTTP only 127.0.0.1:5001
                                                   /api/status /api/points /api/save
```

**IMPORTANT:** `/mid360/points_filtered` is a lightweight visualization/diagnostic output. It is **not** wired into FAST-LIO, because voxel filtering discards Livox per-point timestamp fields used for deskewing. FAST-LIO subscribes directly to the original `/livox/lidar` and `/livox/imu`.

## Modular ROS2 C++ nodes

1. `cloud_filter` — configurable validity/range filtering and voxel grid, input `/livox/lidar`, output `/mid360/points_filtered`.
2. `map_preview` — subscribes **corrected** P4 `/p4/optimized_map` in `map` frame, bounded voxel preview `/mid360/map_preview` (transient-local QoS), independently saves PCD using `/mid360/save_preview`. Rejects non-map-frame input; never merges old point clouds using uncorrected poses.
3. `web_dashboard` — ROS2-native C++ + Boost.Beast loopback HTTP server and live canvas 3D viewer. Subscribes preview, sensor cloud, IMU and optimized odometry; it calls existing `/p4/save_map` asynchronously to save full PCD + poses CSV.

Launchers:

- `mapping.launch.py` — use existing sensor and launch FAST-LIO/P4 + three modular nodes.
- `dashboard.launch.py` — viewer/filter only; attach to a separately running mapping stack.
- `fixture.launch.py` — synthetic P4 fixture and web dashboard, *no sensor or motors*.

## Quick start / service

The user-level service was installed as:
`~/.config/systemd/user/mid360-dashboard.service`

The service uses `tools/run_mid360_dashboard.sh`, `ROS_DOMAIN_ID=0`, and `mapping.launch.py start_pipeline:=true start_sensor:=false web_port:=5001`.

The service startup script checks whether the Livox ROS2 driver is already running. When present it reuses its `/livox/lidar` and `/livox/imu` topics rather than starting a duplicate. If none is running after a reboot, the launch starts its own driver using the saved Livox network configuration.

```bash
export XDG_RUNTIME_DIR=/run/user/1000
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
systemctl --user status mid360-dashboard.service
systemctl --user restart mid360-dashboard.service
journalctl --user -u mid360-dashboard.service -f
```

Open from the **same host**: http://localhost:5001/

This server explicitly listens on 127.0.0.1, not 0.0.0.0. To access remotely, forward the port through SSH, e.g. `ssh -L 5001:localhost:5001 sirobo`.

To run a detached mapping stack manually, stop the service first to avoid duplicate P3 and P4 node instances:

```bash
systemctl --user stop mid360-dashboard.service
cd /media/sirobo/HDD/slam3d-livox-mid360s
source env.sh
ros2 launch slam3d_mid360_dashboard mapping.launch.py start_sensor:=false web_port:=5001
# Later restore automatic startup:
systemctl --user start mid360-dashboard.service
```

For a software-only fixture, **stop the service first** (port 5001 conflict), and run:

```bash
systemctl --user stop mid360-dashboard.service
source /media/sirobo/HDD/slam3d-livox-mid360s/env.sh
ROS_DOMAIN_ID=131 ros2 launch slam3d_mid360_dashboard fixture.launch.py web_port:=5001
```

After testing stop fixture using Ctrl+C, then `systemctl --user start mid360-dashboard.service`.

## HTTP routes

- `GET /` — self-contained canvas 3D point-cloud dashboard (no CDN).
- `GET /api/health` — HTTP status of dashboard process (not sensor-health proof).
- `GET /api/status` — LiDAR/IMU age and online condition, odometry age, map age, point counts, latest save result.
- `GET /api/points` — bounded sample of corrected global map points, max 3500 returned by default.
- `POST /api/save` — asynchronously invokes P4 service `/p4/save_map`, producing corrected map PCD and optimized trajectory CSV in `results/stage4/maps`.
- `POST /api/save_preview` — independently saves voxel-reduced corrected preview PCD in `results/mid360_dashboard/maps`.

No motor, actuator, or Nav2 movement commands are available through this HTTP service.

## Build commands

```bash
source /opt/ros/humble/setup.bash
cd /media/sirobo/HDD/slam3d-livox-mid360s
colcon build --packages-select slam3d_mid360_dashboard --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 pkg executables slam3d_mid360_dashboard
```

Configuration: `src/slam3d_mid360_dashboard/config/dashboard.yaml`.

## Verification completed 2026-10-08

- Built `slam3d_mid360_dashboard` with colcon release: **PASS**, with all three C++ executables.
- ROS2 P4 synthetic fixture: `/p4/optimized_map` to C++ `map_preview`: **PASS**.
- `GET /api/status` showed map points and P4 odometry; `GET /api/points` returned finite XYZ/intensity arrays: **PASS**.
- Both `POST /api/save` and `POST /api/save_preview` created PCD files in their expected output trees: **PASS**.
- Headless Chrome opened dashboard, initialized the 3D canvas, and displayed P4 synthetic points: **PASS**.
- Final browser test on service startup: HTTP 200, health 200, favicon 204, JavaScript page errors 0: **PASS**.
- `systemctl --user` service enabled and started in auto-restart mode: **PASS**.

## Current physical limitation / next acceptance

The MID-360S had previously been discovered at 192.168.1.197 via enp4s0 (192.168.1.10), and its existing driver process remained active. However, **at the time of the final tests it produced 0 Hz LiDAR and IMU; 192.168.1.197 did not respond to ping**. Hence the production dashboard correctly shows LiDAR **OFFLINE** and there is **no new real-world corrected 3D map**. Prior synthetic P4 fixture maps are not to be confused with hardware maps.

`config/extrinsics.yaml` still declares `status: uncalibrated`. Never assert physical mapping precision, reliable loop closure, or navigation readiness without measuring LiDAR/base extrinsics and accepting sensor time synchronization.

To complete field acceptance once LiDAR PSU/network are available:

1. Verify network `ros2 run livox_mid360_manager livox_discover --interface auto --timeout 5`.
2. Check `/livox/lidar` and `/livox/imu` data freshness/rate in a non-replayed ROS domain; if needed, restart the *existing* sensor driver, do not run a duplicate.
3. Measure LiDAR-to-base and LiDAR-to-IMU alignment and time synchronization and update extrinsics only after actual measurements.
4. Move sensor around a closed, overlapping path and observe corrected `/p4/optimized_map` at http://localhost:5001.
5. Save P4 PCD+CSV via the dashboard or `ros2 service call /p4/save_map std_srvs/srv/Trigger '{}'`.
6. Inspect point geometry, loop false closures, trajectory drift and map reliability before Nav2 motion; preserve software/hardware E-stop and motion guard design from P6.

Foxglove native WebSocket bridge is a distinct, optional service, normally `ws://localhost:8765` and is **not a substitute** for HTTP port 5001. If installed, bridge the ROS2 topics `/p4/optimized_map` and `/mid360/map_preview` for Foxglove desktop inspection.
