# P6 — ROS2 Nav2 bridge, TF base_link, and fail-closed motion gate

Workspace: `/media/sirobo/HDD/slam3d-livox-mid360s` (Ubuntu 22.04, ROS2 Humble)

## Architecture

P6 adds `src/slam3d_nav2_bridge`. It preserves the P3 FAST-LIO estimator and P5 map-based relocalizer.

TF tree in P6 mode:

```
map                  (P5, only when localized)
  └─ lio_odom        (P5 correction)
       └─ base_link  (P6 odom_base_bridge)
            └─ livox_frame (P6 static, measured extrinsic REQUIRED)
```

P3 normally publishes `lio_odom -> livox_frame`. P6 sets `publish_odom_tf:=false` on P3 to avoid two parents for Livox; the original default remains true in standalone P3/P4/P5.

P6 `odom_base_bridge`:
- subscribes `/lio/odometry`, with child `livox_frame`;
- computes `lio_odom -> base_link` using `base_link -> livox_frame` extrinsics in `config/extrinsics.yaml`;
- publishes `/odom` with the base_link child frame;
- publishes static `base_link -> livox_frame` and dynamic `lio_odom -> base_link` TF;
- does not modify P3's state estimate.

The current default extrinsic file has `status: uncalibrated`, a zero translation and identity rotation, **not a measured mount**. Production motion remains inhibited until physically calibrated.

## Software guard

`localization_stop_gate` is separate from Nav2. Nav2 Humble's velocity smoother publishes `/cmd_vel`, which the guard receives; guarded output is `/cmd_vel_safe`.

The guard publishes **zero Twist at 20 Hz** and blocks positive motion under any of:
- initialization/disarmed/E-STOP;
- invalid or unverified calibration (except isolated software fixture);
- P5 not TRACKING; lack of consecutive TRACKING reports;
- stale P5 status, missing/stale `/odom`, unavailable/stale map->base_link TF;
- no fresh velocity command, invalid lateral or nonfinite Twist, or limit violation (valid x/yaw components are clamped);
- active E-STOP, with deliberate re-ARM required.

Services:

```bash
ros2 service call /p6/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call /p6/arm std_srvs/srv/SetBool '{data: false}'
ros2 service call /p6/estop std_srvs/srv/SetBool '{data: true}'
ros2 service call /p6/estop std_srvs/srv/SetBool '{data: false}'
```

After loss of localization or a command timeout, the guard disarms and **does not resume motion automatically** after P5 recovers. Software E-STOP is not hardware safety-rated.

**Critical physical interface:** The motor gateway/ESC must subscribe **only** to `/cmd_vel_safe`, never to `/cmd_vel` or `/cmd_vel_nav`; enforce an independent motor-controller heartbeat/timeout and hardwired E-STOP. P6 does **not** connect to an ESC or physically stop the robot on its own. A software gate alone cannot protect against a computer freeze or disconnection.

## Occupancy map and Nav2

`pcd_occupancy_map` converts an explicitly selected P4 binary PCD into `/map` (`nav_msgs/OccupancyGrid`) with:
- floor and obstacle height filters;
- configurable resolution, padding and per-cell count threshold;
- unknown cells remain unknown (not automatically cleared/free);
- conservative map dimension limit;
- transient-local QoS for Nav2 static map layer.

P6 Nav2 config `config/nav2_mid360s.yaml` uses:
- `map` global frame and `lio_odom` local frame;
- `base_link` robot frame;
- MID-360S PointCloud2 obstacle input `/livox/lidar`;
- a P4 PCD-derived occupancy `/map` layer;
- existing Nav2 Humble controller/planner/BT/velocity-smoother lifecycle nodes.

`config/nav2_mid360s_fixture.yaml` uses synthetic `/lio/cloud_registered` instead. It must not be used with physical motors.

**Robot radius 0.35 m and speed limits are placeholders**; measure the actual AGV footprint and Ackermann turning limits. The default Nav2 DWB controller is not validated for this AGV's steering kinematics or stopping distance.

## Build

```bash
cd /media/sirobo/HDD/slam3d-livox-mid360s
./build.sh
source env.sh
```

Seven packages should build: `livox_ros_driver2`, `livox_mid360_manager`, `kiss_icp`, `fast_lio`, `slam3d_pose_graph`, `slam3d_relocalizer`, `slam3d_nav2_bridge`.

## Offline fixture — no motor connection

Uses the latest saved paired Stage4 fixture PCD/CSV and P5 fixture data. **Only here** is `simulation_only_override` enabled.

```bash
ros2 launch slam3d_nav2_bridge stage6_fixture.launch.py \
  frames:=160 fixture_rate:=5.0 blackout_start:=95 blackout_frames:=35
```

The guard still requires P5 TRACKING, fresh TF and odometry, and explicit ARM. It never subscribes to a hardware interface.

Test with Nav2 lifecycle as well (still no motor):

```bash
ros2 launch slam3d_nav2_bridge stage6_nav2_fixture.launch.py \
  frames:=180 fixture_rate:=5.0
```

Nav2 launch defaults are deliberately separate from production. Do not send real drive commands when running this fixture.

## Real stack — only after calibration

```bash
source /media/sirobo/HDD/slam3d-livox-mid360s/env.sh
ros2 run livox_mid360_manager livox_discover --interface auto --timeout 5

ros2 launch slam3d_nav2_bridge stage6.launch.py \
  map_pcd:=/path/to/validated_stage4_map.pcd \
  poses_csv:=/path/to/matched_stage4_poses.csv \
  start_sensor:=true start_lio:=true start_nav2:=false
```

`start_nav2:=false` is the safe default. When configuration/calibration and physical safety have been independently verified, Nav2 can be activated with `start_nav2:=true`. P6's normal launch **refuses** this when `config/extrinsics.yaml` still reports uncalibrated. A simulation override with live sensor/LIO launch is explicitly blocked.

The guard requires operator ARM and remains a ROS2 **software safety layer**, not safety-rated protection.

## Inspection

```bash
ros2 topic echo /p5/status
ros2 topic echo /p6/safety_state
ros2 topic echo /cmd_vel_safe
ros2 topic echo /odom
ros2 topic echo /map
ros2 run tf2_ros tf2_echo map base_link
```

Do not start `stage4.launch.py` at the same time as P5/P6, because both P4 and P5 can publish `map -> lio_odom`.

## Completion status

P6 software package and Nav2 lifecycle fixture are implemented and tested. Physical E-STOP, motor connection, measured extrinsics, sensor power-on acceptance, Ackermann control tuning, robot footprint, terrain/drivable area validation and actual driving remain pending. Consult `STAGE6_AUDIT_20261008.md`.
