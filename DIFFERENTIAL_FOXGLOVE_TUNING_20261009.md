# Foxglove / MID-360S Differential Rover — Stock PX4, No Flash
Date: 2026-10-09 | Host: sirobo

## Read-only ROS diagnostics
- `/pixhawk/differential/diagnostics` (std_msgs/String JSON) publishes at 10Hz.
- `controller_type=DIFFERENTIAL_SKID_STEER`, `left_wheel_mps`, `right_wheel_mps`, `wheel_track_m`, `wheel_track_verified`, `preview_valid`, `reason`, `hardware_command_enabled=false`, `ros_motion_publisher_enabled=false`.
- `/pixhawk/stock/{status,advisory}` and `/pixhawk/avoidance/{diagnostics,advisory}` unchanged; original Foxglove bridge remains on websocket `ws://127.0.0.1:8765`.
- The physical Cube may not have differential Rover code. The ROS preview is not a physical motor controller and does not write any hardware commands.

## Staged web UI
- Mapping: `http://127.0.0.1:5001`.
- Safety tuning: `http://127.0.0.1:5002` (localhost loopback only).
- Differential form has `wheel_track_m` (0 = UNKNOWN / not physically measured) and `max_wheel_speed_mps`. The old Ackermann fields are removed.
- Save YAML writes only `config/stock_px4_tuning.yaml`; Apply to Simulation writes only `config/stock_px4_simulation.yaml`; Rollback restores previous local staging. No live ROS node receives parameters from these web actions, and Cube is never updated.
- Differential ROS node config (separate from web staged config): `/media/sirobo/HDD/pixhawk/src/stock_px4_supervisor/config/differential.yaml`. It intentionally sets `simulation_only=false`, `wheel_track_verified=false`, `wheel_track_m=0` and emits zero wheel speed preview.
- On the simulator ONLY, an explicitly measured/known synthetic test wheel track (0.6m in the fixture) can be used; this value must not be copied to an actual rover without measurement.

## Tests
- CTest `2/2 PASS`; ROS2 topic-level velocity preview tests PASS (all outputs remain gated; RC takeover and stale data produce zero).
- Playwright desktop/mobile screenshots: `results/stock_px4_browser/differential_px4_desktop.png` and `differential_px4_mobile.png`; 18 numeric inputs, no JS errors, 390px mobile width.
- UI buttons Save YAML, Apply to Simulation, Rollback PASS as LOCAL FILE actions.
- Physical motor or Rover AUTO tests NOT RUN.

## Safety first
Emergency-stop hardware, RC manual priority, exact installed PX4 Rover variant, physical left/right motor pin mapping, motor driver protocol and wheel-track calibration remain BLOCKED. The wheel velocity preview must not be connected to PX4 output channels or direct motor PWM. The present Cube firmware was not flashed or changed.
