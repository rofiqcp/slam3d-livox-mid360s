# Stage 5 Acceptance Audit — 2026-10-08

Workspace: /media/sirobo/HDD/slam3d-livox-mid360s
Remote machine: sirobo-rdc1 / Ubuntu 22.04 / ROS2 Humble

## Implementation

Added `src/slam3d_relocalizer` (C++ ROS2) with:

- P4 binary PCD map loading and paired keyframe-pose CSV loading
- Finite-point filtering, map and live-scan voxelization
- Frame-ID and cloud/odom timestamp validation
- Source scan extraction into sensor-local frame from P3 odom/world-registered cloud
- Coarse-to-fine PCL ICP seed matching against bounded local target map
- Optional `/initialpose` prior, automatic seeded search using P4 saved keyframe poses
- Fitness and overlap acceptance checks; rejects poorly supported alignments
- State machine UNINITIALIZED / SEARCHING / TRACKING / LOST
- Predicted map-frame odometry at input rate, periodically revalidated with fast tracking ICP
- Recoverable LOST state on stalled cloud stream or registration failures
- Outputs /p5/reference_map, /p5/localized_odometry, /p5/localized_path, /p5/aligned_scan, /p5/status and /diagnostics
- TF map -> lio_odom; P3 continues publishing lio_odom -> livox_frame
- /p5/relocalize, /p5/reload_map, /p5/save_pose ROS2 services
- Independent stage5, stage5_fixture and stage5_bag launch files
- No simultaneous P4 pose graph/TF publication in P5 mode
- Explicit map file path required; source will NOT silently select P4 fixture maps in live mode.

Uses Ubuntu ROS2 system PCL 1.12 and avoids mixing PCL 1.15. No ROS1 code.

## ROS2 build

Verified full `./build.sh` successful with **six** ROS2 packages:
1. fast_lio
2. kiss_icp
3. livox_mid360_manager
4. livox_ros_driver2
5. slam3d_pose_graph
6. slam3d_relocalizer

`src/slam3d_relocalizer` builds both `relocalizer_node` and `map_fixture`.

## P4 saved fixture localization acceptance

Input:
- P4 fixture binary PCD: 11,349 stored points (4,884 after P5 voxel reduction)
- Matching CSV: 60 saved P4 keyframe poses
- Synthetic robot motion from map-matched P4 fixture trajectory, lidar/odom topics frame lio_odom

Continuous tracking result:
- Initially relocalized once with ICP fitness ~0.03275 and overlap 1.000
- Maintained TRACKING and published at 5 Hz
- 106 localized poses
- Translation RMSE vs fixture ground truth **0.01638332841 m**
- 0 jumps, 0 nonmonotonic stamps
- Z span ~0.0020951 m
- /p5/save_pose returned success=true and wrote timestamped YAML

Initial implementation exposed a real-time issue: full multiseed ICP was too slow for every scan; the watchdog marked LOST immediately after a slow search. The final implementation:
- Measures last accepted wall clock AFTER ICP completion,
- Uses a keyframe-seed-based coarse/fine ICP only for global relocalization,
- Uses faster ICP for periodic tracking refinement,
- Publishes extrapolated poses from the previous accepted correction on intermediate scans,
- Retains quality checks and truthful LOST status.

## Sensor blackout / automatic recovery

Injected an intentional point-cloud blackout of 25 frames at 5 Hz (~5 seconds), while the simulated rover continued moving.

- First global relocalization: seed 0, fitness ~0.03275
- After blackout P5 entered LOST; on restored cloud stream it relocalized again (seed 11, fitness ~0.03157)
- Tracking resumed, 74 published poses
- Translation RMSE vs synthetic ground truth **0.01596996495 m**
- 0 nonmonotonic stamps, zero permanently dropped cloud frames reported
- Evaluator reports 1 position jump at the reacquisition boundary. This is expected: the robot moved several meters while localized scan data was absent. A navigation controller must suspend operation in LOST and replan after recovery, not try to smooth or hide the discontinuity.
- All processes exited normally.

## Services and forced recovery

Separate fixture regression successfully called:
- /p5/reload_map: returned success=true and reset localization
- /p5/relocalize: returned success=true and scheduled a new search
- /p5/save_pose: returned success=true while tracking; YAML written.

Three confirmed relocalization events after reload/manual request:
- 87 pose outputs, 0 jumps, 0 nonmonotonic stamps
- Ground truth synthetic translation RMSE **0.01507250393 m**
- All ROS2 nodes exited cleanly.

## Safe failure testing

- Starting stage5.launch.py with no `map_pcd` returns launch RC=1, with explicit message requiring the saved P4 map; no accidental synthetic map used.
- P4 snapshot generated from a short open-route P3 bag had only 4 keyframes (3,592 raw PCD points; P5 map 3,267 voxel points).
- Replay of the 8-second P3 raw cloud+IMU bag correctly rebuilt P3 FAST-LIO odometry (~77 poses) but P5 declined to initialize on this sparse snapshot. No accepted pose was published, status remained LOST, /p5/save_pose returned success=false ("Not localized").
- Do not count this sparse-map replay as a successful P5 localization. It demonstrates bounded, truthful rejection when map/keyframe survey coverage is insufficient. Capture a denser P4 map with genuine revisit and more keyframes instead of disabling quality gates.
- Added warnings for fewer than six saved keyframes and attempted candidate fitness/overlap diagnostics.

## GUI and binary runtime

- Stage5 RViz opened the saved fixture reference map using OpenGL 4.5, started and shut down with clean ROS2 node exits.
- Stage5 loaded a map with no sensor active and correctly displayed UNINITIALIZED.
- PCL dependency: system libpcl_common.so.1.12; no libpcl 1.15 linked.
- No relocalizer, fixture, FAST-LIO or RViz process left active after tests.

## Remaining acceptance / production blockers

MID-360S power remained off; **no physical hardware accuracy claim is made**.
The P3 LiDAR–IMU extrinsic is still a baseline identity, not validated against the real mounting geometry.

Before field-level readiness:
1. Power and rediscover MID-360S; verify raw sensor timestamps, QoS, packet rate and frame IDs.
2. Calibrate sensor/LiDAR/IMU extrinsic and confirm stationary Z/gravity stability.
3. Build a dense validated real P4 map with loop closure and multiple P4 keyframes, and save matched PCD/CSV snapshot.
4. Restart P3 odometry in a different location and verify P5 localization/recovery, false positives, map alignment and CPU load.
5. Test interrupted sensor/odometry streams, repeated corridors and changes in scene geometry; log rejected registrations.
6. Only then connect P5 global pose into Nav2 with a calibrated base_link TF and a LOST-based motion stop interlock.

P5 provides saved-map ICP localization, pose recovery and persistence, NOT proven fully global relocalization from arbitrary unseen scenes, autonomous navigation or learned place recognition. These are future stages.
