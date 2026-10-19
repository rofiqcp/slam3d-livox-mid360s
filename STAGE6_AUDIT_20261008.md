# P6 acceptance audit — 2026-10-08

Workspace: `/media/sirobo/HDD/slam3d-livox-mid360s`, remote Ubuntu 22 / ROS2 Humble.

## Implemented

Added ROS2 C++ package `slam3d_nav2_bridge`:
- `odom_base_bridge`: P3 Livox-frame odometry to base_link odometry, publishes /odom and TF lio_odom -> base_link, plus static base_link -> livox_frame from explicit extrinsics.
- FAST-LIO `publish_odom_tf` parameter: by default true for legacy P3-P5 workflows, false in integrated P6 to avoid duplicate TF parent for sensor.
- `localization_stop_gate`: fail-closed Nav2 command path `/cmd_vel` to `/cmd_vel_safe`, zero output timer 20 Hz while unsafe, P5 TRACKING/status freshness, /odom freshness, map->base_link TF freshness, operator ARM, software E-STOP, speed caps, invalid command rejection.
- `pcd_occupancy_map`: P4 PCD projected to conservative 2D OccupancyGrid /map for Nav2 static layer; unknown occupancy remains unknown.
- `stage6.launch.py`: P5 + base bridge + guard + occupancy map, Nav2 disabled by default, rejects Nav2 activation until base_link->livox_frame is calibrated. Forbids simulation override together with live sensor/LIO.
- `stage6_fixture.launch.py`: software-only P5 fixture plus bridge/guard, no physical motor.
- `stage6_nav2_fixture.launch.py`: adds Nav2 Humble navigation lifecycle to fixture.
- `nav2_mid360s.yaml`: conservative Nav2 PointCloud2 costmap configuration on /livox/lidar.
- `nav2_mid360s_fixture.yaml`: simulated registered cloud used for fixture; strictly not production.
- `STAGE6.md`: commands, safety architecture, limits, and operator preconditions.

## Build

The complete workspace `./build.sh` compiled all **7/7** ROS2 packages:
- fast_lio, kiss_icp, livox_mid360_manager, livox_ros_driver2
- slam3d_pose_graph, slam3d_relocalizer, slam3d_nav2_bridge.

Subsequent package-specific clean incremental builds passed, including `slam3d_nav2_bridge` after the command-timeout safety change and the Nav2 fixture argument fix.

## Interlock end-to-end fixture

Simulation: P4 saved fixture PCD (60 keyframes) -> P5 relocalizer -> P6 odom base bridge -> safety gate, ROS_DOMAIN_ID=118.

1. `/p5/status=TRACKING`, fresh odometry and TF, explicit `/p6/arm`: ARM returned success=true.
2. Inject `/cmd_vel` Twist linear.x=0.15 m/s, angular.z=0.1 rad/s. `/p6/safety_state=READY` and `/cmd_vel_safe` showed exactly 0.15 m/s and 0.1 rad/s.
3. `/p6/estop` true: returned success=true, state `E_STOP` and `/cmd_vel_safe` zero for linear and angular components.
4. Clear E-STOP: returned success=true while disarmed. Rearm during a stale-TF interval was correctly denied: `Arm denied: TF_STALE`.
5. Synthetic LiDAR blackout (35 frames at 5 Hz): guard transitioned to `TF_STALE` and then `LOCALIZATION_LOST`, issuing zeros throughout.
6. P5 successfully performed a second relocalization after scan return. The guard remained disarmed until an explicit new ARM.
7. All six fixture processes exited cleanly with launch result 0.

## Command freshness rearm regression

After changing P6 to disarm on an expired command timeout, a second fixture test (ROS_DOMAIN_ID=125) verified:

- ARM granted after P5 tracking and healthy TF.
- After stopping the `/cmd_vel` publisher, the guard transitioned `COMMAND_TIMEOUT` then `OPERATOR_DISARMED`.
- `/cmd_vel_safe` linear and angular output both exactly zero.
- Restarting the raw `/cmd_vel` publisher **without** operator rearm did not resume motion; state remained `OPERATOR_DISARMED`.
- Fixture ended with exit code 0.

## Nav2 Humble integration smoke test

The first Nav2 fixture launch failed due to PythonExpression evaluating lowercase `false`; corrected `use_composition` and `use_respawn` to Python `False` in the launch arguments, without changing installed Nav2 source.

Second fixture ran Nav2 lifecycle; logs verified successful configuration and activation for:
- controller_server
- smoother_server
- planner_server
- behavior_server
- bt_navigator
- waypoint_follower
- velocity_smoother

Nav2 lifecycle manager logged `Managed nodes are active`. No ROS node crashes, and launch exited status 0 after requested SIGINT.

The ROS2 CLI lifecycle inspection itself encountered a stale XMLRPC/daemon `!rclpy.ok()` error and is NOT counted as a separate successful status query. The lifecycle-manager startup log is the basis of the activation result.

No Nav2 route-following action was executed against physical or simulated motors. Do not claim trajectory following or Ackermann steering has been validated.

## Explicit production safety lock

`stage6.launch.py ... start_nav2:=true` with uncalibrated extrinsics was tested against an existing P4 fixture PCD and correctly refused launch with RC=1:

`P6 Nav2 launch is blocked until base_link->livox_frame is physically calibrated.`

No long-running P6/fastlio/Nav2 processes remained after the calibration-lock test.

## Limitations and next physical acceptance

1. Physical MID-360S remained unpowered in these acceptance runs.
2. Real base_link-to-Livox extrinsics are still identity/zero placeholders and must be measured.
3. Nav2 robot radius 0.35 m, tuning and speed limits are example values, not AGV measurements.
4. The user's rover appears to require Ackermann steering; this fixture uses Nav2's default differential-drive-style DWB outputs. Turning constraints, minimum steering radius and motor gateway integration must be designed and tested before any actual motion.
5. No motor/ESC/controller subscribed to /cmd_vel_safe in these tests. The bridge must be the **only** motion command source, and motor controller must enforce an independent hardware timeout, physical hardwired E-STOP and watchdog on lost host connection.
6. The PCD occupancy projection cannot prove a drivable floor without ground segmentation, inspection and clearance assessment; all unknown cells remain unknown.
7. Software E-STOP cannot substitute for independently safety-rated physical emergency stop.
8. Required physical tests: LiDAR powered and discovered, live packet/IMU rate and timing, extrinsic and gravity alignment, Nav2 TF continuity, safety STOP propagation to motor driver, command watchdog loss, steering limits, obstacle avoidance and controlled route repeatability.

**Acceptance status:** P6 software implementation, ROS2 command gate/TF/2D map and Nav2 lifecycle fixtures: PASS. Motor/hardware field readiness: NOT VERIFIED.
