# MID-360S → Stock PX4 Safety Tuning (2026-10-09)
**Stage: loopback-only software staging, not live actuator control.**

## Sites and topics
- Existing 3D mapping dashboard: http://127.0.0.1:5001 (DO NOT terminate production mapping to launch tuning).
- New standalone local safety-tuning web: http://127.0.0.1:5002 (port bound to loopback only).
- Existing Foxglove ROS bridge WebSocket: ws://127.0.0.1:8765.
- For access from another PC, use SSH local port forwarding instead of binding tuning to 0.0.0.0, e.g. `ssh -L 5002:localhost:5002 sirobo`.
- ROS2 input telemetry: /pixhawk/status, /pixhawk/rover/feedback, /pixhawk/imu, /pixhawk/avoidance/advisory.
- ROS2 output **diagnostic only**: /pixhawk/stock/status and /pixhawk/stock/advisory.
- Existing ROS2 /pixhawk/avoidance/advisory is an obstacle **suggestion**, not true PX4 stock command.
- Map /lio/cloud_registered /p4/* and real MID360 sensors continue through existing pipelines.

## Launch
```bash
cd /media/sirobo/HDD/pixhawk
./build.sh
source env.sh
ros2 run stock_px4_supervisor stock_px4_supervisor --ros-args --params-file src/stock_px4_supervisor/config/supervisor.yaml
# In a separate shell:
python3 /media/sirobo/HDD/slam3d-livox-mid360s/tools/stock_px4_tuning_server.py
```
Never enable physical motion/mission writes for this tuning panel.
The supervisor defaults to simulation_only=false, calibration_verified=false, rc_priority_verified=false and steering_feedback_verified=false. Thus hardware stays advisory UNKNOWN/STOP unless independent evidence and future code explicitly support real operation. This is deliberate.

## UI parameter guide
| Parameter | Units | Meaning |
|---|---|---|
| stop_distance_m, slow_distance_m | m | Fixed STOP floor and graduated caution range; slow > stop |
| cruise_limit_mps, avoid_limit_mps | m/s | Proposed speed caps (not PX4 speed commands) |
| deceleration_mps2, reaction_time_s, safety_margin_m | SI | Dynamic STOP = max(stop_floor, v²/(2a) + reaction*v + margin) |
| left_clearance_m, right_clearance_m | m | Future candidate path clearance thresholds, currently staging only |
| wall_kp, wall_ki, wall_kd | -, s, 1/s | Stage PID gains; current pure-C++ engine uses Kp only; Ki/Kd are not active |
| yaw_limit_radps | rad/s | Absolute planner suggestion yaw-rate upper bound |
| wheelbase_m, minimum_turn_radius_m, max_steer_rad | m, m, rad | Ackermann geometric yaw bound; steering PWM is NOT angle feedback |
| clear_confirm_frames | scans | Minimum 3 distinct fresh advisory observations plus operator acknowledgement after critical latch |
| advisory_timeout_ms, mav_timeout_ms | ms | Watchdog staging; C++ core currently uses safe defaults until full schema binding |
| preferred_wall_side | AUTO/LEFT/RIGHT | Steering direction candidate preference; not hardware motion |
| imu_timeout_ms | ms | Fixed 400 ms in current C++ implementation (not yet UI exposed) |

**Boundaries**: web Save YAML stores a local staged profile in `config/stock_px4_tuning.yaml`; Apply to Simulation writes `config/stock_px4_simulation.yaml` for future simulator launch; Rollback restores the preceding staged YAML. These actions do NOT change a running ROS node or PX4. Explicitly restart an isolated simulator with the staged profile if desired; there is currently no hot-apply consumption in the C++ node. None of these operations write Cube parameters.

## Diagnostic semantics
- ESTOP: highest priority external emergency input (hardware implementation and monitoring not yet verified).
- MANUAL_RC: RC takeover is *simulated* only unless actual PX4 mode/RC evidence is available. The production ROS bridge publishes channel values, which alone do not prove RC mode.
- UNKNOWN: stale/missing/invalid sensor, localisation, RC mode, steering feedback, PX4 mode or unsupported runtime interface. Output remains STOP advisory.
- STOP: near obstacle or insufficient stopping distance.
- AVOID_PROPOSED: wall side/yaw computed but no collision-checked full path and no proven stock PX4 interface; commanded speed remains zero.
- LATCHED: 3 fresh observations plus explicit operator reset required to clear a hazard. A companion process cannot enforce this latch on a disconnected Cube without independent safety hardware.
- CLEAR: advisory positive on validated synthetic inputs; no hardware command.
- Foxglove panel status `hardware_command_enabled=false` is immutable in this implementation. `stock_firmware_interface=UNPROVEN` is an honest capability boundary.

## MCAP & fault replay
Browser MCAP Start/Stop buttons intentionally return a supported error rather than silently starting unattended recording. To record using ROS2 CLI **only if mcap storage is installed**:
```bash
source /media/sirobo/HDD/pixhawk/env.sh
mkdir -p /media/sirobo/HDD/slam3d-livox-mid360s/results/stock_px4_mcap
ros2 bag record -s mcap -o /media/sirobo/HDD/slam3d-livox-mid360s/results/stock_px4_mcap/diagnostics \
 /pixhawk/stock/status /pixhawk/stock/advisory /pixhawk/avoidance/advisory \
 /pixhawk/status /pixhawk/rover/feedback
```
Use Ctrl+C for a complete bag close. Replay must be in a separate ROS_DOMAIN_ID and with no real hardware bridge connected; do not replay control topics into live robot.

## Browser evidence
- Desktop screenshot: `results/stock_px4_browser/desktop.png`
- Mobile screenshot: `results/stock_px4_browser/mobile.png`
- Playwright: 19 numeric configuration controls; desktop 1440x960 and mobile 390x844 rendered; no 390px horizontal overflow.
- Mapping localhost:5001 and tuning localhost:5002 returned HTTP 200.
- Staging Save YAML and Apply to Simulation actions work as local file stages. Rollback requires a previous profile; never restore firmware or Cube parameters through UI.
- As of implementation, the web status API is tied to read-only local ROS supervisor status snapshot and must report UNKNOWN when the snapshot becomes stale. Calibration remains unverified.

## Unfinished items, not to label PASS
1. Live PX4 actual firmware version, MAVLink RTT/packet loss, steering angle feedback, mission index + state + command ACK in 5002 UI.
2. 50 Hz LiDAR sector perception/ground segmentation, trajectory collision validation, gain-integral/differential anti-windup and speed jerk constraints.
3. Simulator consuming staged YAML with real obstacle trajectories, version-matched PX4 SITL and browser-supported fault injection; ROS2 MCAP replay test.
4. Dedicated hardware E-STOP, independent loss watchdog/hazard latch, autonomous mission safety with companion hotplug, and RC takeover hardware proof.

## Safe rollback
Stop only the added stock_px4_supervisor and optional stock_px4_tuning_server.py. Existing `pixhawk_bridge`, mapping/FAST-LIO, Foxglove ROS bridge, and mission data are not touched. Restore staged config with Rollback or remove the staging files after backing them up; no physical controller changes occurred.

## Verified session addendum (2026-10-09)
- Foxglove-adjacent tuning served on local port 5002, with ROS safety JSON read from `/home/sirobo/.cache/mid360_pixhawk_ros/stock_px4_latest.json` (atomic snapshot written by ROS2 stock supervisor). The status API reports UNKNOWN when source is stale; live test showed `RC_PRIORITY_UNVERIFIED` and `hazard_latched=true`.
- Playwright desktop + mobile rerun: state UNKNOWN, Save YAML returned success, Rollback returned success, mobile width 390px without overflow. Screenshots updated at `results/stock_px4_browser/{desktop,mobile}.png`.
- MCAP recorded via CLI, not web: `results/stock_px4_mcap/read_only_20261009/` contains a valid 111-message, 5.2 s bag, playback in separate ROS_DOMAIN_ID=77 exit 0. Simulation synthetic LiDAR test ran separately in domain 78: CLEAR / AVOID / STOP / CLOUD_STALE PASS.
- Unified ROS2 build 12 packages PASS, CTest 1 PASS with 242 assertions. PTY MAVLink simulator PASS mission/parameters/mode/cancel. These PASS items do not establish physical rover autonomy, nor is the installed Cube firmware version yet identified.
- Resource baseline: `/home/sirobo/.cache/mid360_pixhawk_ros/stock_resource_profile.json`; no loaded vehicle dynamics or end-to-end latency certification.

## RC operator take-over UI verification (2026-10-09)
The loopback tuning panel now reports manual-mode observation, RC receiver telemetry freshness and PX4 main/submode only from available read-only ROS status; default is **not verified**. Playwright (desktop/mobile) reported 19 inputs, zero JS errors, mobile 390px without horizontal overflow. Screenshots `results/stock_px4_browser/rc_takeover_desktop.png` and `rc_takeover_mobile.png`. Source of truth is PX4 flight-mode heartbeat; merely seeing RC_CHANNELS or a nonzero RSSI is not proof of effective physical steering authority. The hardware serial bridge still runs from the older cache; missing RC data remains explicitly unknown.
