# Stage 4 — Loop Closure + Pose Graph Optimization + Corrected 3D Map

Workspace: `/media/sirobo/HDD/slam3d-livox-mid360s`.

## Architecture

P3 FAST-LIO stays isolated and keeps publishing `/lio/odometry` (odometry in frame `lio_odom`) and `/lio/cloud_registered` (PointCloud2 already transformed into `lio_odom`). P4 adds a separate pure C++ ROS2 package `slam3d_pose_graph`, with GTSAM iSAM2 and PCL 1.12.

P4 does **not** replace the P3 state estimator or alter the raw LiDAR/IMU driver.

Main processing stages:

1. Timestamp match P3 odometry and global-frame registered point cloud (30 ms tolerance by default).
2. Validate correct input frame `lio_odom` and discard invalid cloud records.
3. Downsample each keyframe and convert its registered world cloud into the local LiDAR frame using the P3 pose inverse.
4. Select a keyframe after minimum translation or rotation.
5. Build a lightweight ring/sector (Scan Context-style) descriptor for loop candidate search.
6. Require sufficient keyframe/time separation and approximate spatial proximity.
7. Verify the candidate by ICP with an initial relative pose from LIO, bounded fitness, translation, rotation, and max correspondence distance. Reject invalid closures.
8. Build GTSAM pose graph using a strong first-pose prior, odometry BetweenFactors, robust Huber loop BetweenFactors, then incremental iSAM2 optimization.
9. Publish optimized poses and reconstruct the global point cloud by transforming keyframe-local clouds using corrected poses. Apply voxel downsample and point cap.
10. Export the optimized PCD and CSV trajectory via ROS2 Trigger service.

The Stage4 loop backend has independent keyframe and graph sizes and avoids inserting repeated accepted loops within the configured keyframe cooldown.

## ROS topics and TF

Inputs:
- `/lio/odometry` — `nav_msgs/Odometry` (frame `lio_odom`)
- `/lio/cloud_registered` — `sensor_msgs/PointCloud2` (frame `lio_odom`)

Outputs:
- `/p4/optimized_odometry` — corrected odometry in frame `map`
- `/p4/optimized_path` — corrected keyframe path
- `/p4/optimized_map` — corrected point cloud in frame `map`
- `/p4/evaluation_path` — evaluator path for P4
- `/diagnostics` — keyframe, loop, ICP and health counters

TF uses `map → lio_odom`, while P3 continues publishing `lio_odom → livox_frame`. In navigation, use the drift-smooth raw odom frame locally and the P4 corrected map frame globally. The corrected global frame can move when a loop closes; this is expected in pose-graph SLAM.

## Build and run

```bash
cd /media/sirobo/HDD/slam3d-livox-mid360s
./build.sh
source env.sh

# Fully synthetic P3 + P4, does not require MID-360S
ros2 launch livox_mid360_manager stage4_synthetic.launch.py rviz:=true

# Loop closure verification fixture with intentional rover odometry drift
ros2 launch slam3d_pose_graph loop_fixture.launch.py

# Live MID-360S, after power is on and network discovery has succeeded
ros2 run livox_mid360_manager livox_discover --interface auto --timeout 5
ros2 launch livox_mid360_manager stage4.launch.py rviz:=true

# Replay recorded LiDAR+IMU through P3 and P4; recorded old LIO odom is NOT replayed
ros2 launch livox_mid360_manager stage4_bag.launch.py \
  bag:=/path/to/rosbag rviz:=false
```

Map saving while P4 is running:

```bash
ros2 service call /p4/save_map std_srvs/srv/Trigger "{}"
```

Results directory defaults to `results/stage4/maps`. Each call creates timestamped `stage4_map_*.pcd` (binary point cloud) and `stage4_poses_*.csv`. The P4 evaluator writes `results/stage4/runs/<timestamp>/stage4_corrected.csv` and a summary YAML.

## Configuration

Edit `src/slam3d_pose_graph/config/stage4.yaml` (installed with ROS2 package). Important settings:
- `keyframe_distance_m`, `keyframe_rotation_deg`
- `sync_tolerance_s`
- `min_loop_separation_s`, `min_loop_keyframes`, `min_keyframes_between_accepted_loops`
- `candidate_radius_m`, `descriptor_max_distance`
- `icp_max_correspondence_m`, `icp_fitness_threshold`
- `max_loop_correction_m`, `max_loop_rotation_deg`
- `scan_voxel_m`, `map_voxel_m`, `max_map_points`
- `publish_tf`, `enable_loop_closure`

To isolate baseline odometry + map without loops:

```bash
ros2 launch livox_mid360_manager stage4.launch.py enable_loop_closure:=false
```

## Verification fixture

The built-in closed-loop fixture publishes registered world point clouds and odometry on the *exact P3 topics*, with a known circular true route and artificial drift at the final position. It tests real ROS2 subscriptions, descriptor matching, ICP registration, iSAM2 optimization, TF, map and CSV saving.

Initial test (2026-10-08) with two loop factors: 60 keyframes, final positional error 0.040 m (raw end error 0.696 m). After conservative loop cooldown, second test verified **one** loop, 60 keyframes and a final error 0.177 m (raw end error 0.696 m). The test fixture excludes LiDAR driver, ESKF physics and real environment effects.

## Real hardware prerequisites

The MID-360S was powered off during P4 development. No claim is made about field-level loop precision, real 3D-map accuracy or indoor/outdoor localization performance. Prior to using P4 for navigation, first validate live P3 timestamp QoS, extrinsic LiDAR–IMU calibration and steady-state drift, then record an overlapping, closed rover route in MCAP and replay it.

Loop closure can be wrong in repetitive environments. Keep ICP verification and robust factors enabled; do not enable overly permissive thresholds for autonomous operation without evaluating loop false positives.

Loop closure and PCD export are complete Stage4 software functions; persistent map-based relocalization and Nav2 global localization are later stages.


## Final integrated regression

- The final \`./build.sh\` builds 5/5 ROS2 packages.
- Confirmed \`stage4_synthetic.launch.py\` runs the entire P3+P4 synthetic stack and shuts down without process crashes.
- Conservative single-loop fixture: 60 keyframes, one verified ICP closure (score 0.167, fitness 0.0107), 0 dropped clouds and no false duplicate constraint. Latest regression: 0 non-monotonic corrected-odometry stamps.
- P3 raw-sensor bag replay through P4: 4 keyframes, 0 falsely accepted loops, 77 corrected poses, 0 jump/invalid stamps, 0.02693 m synthetic GT RMSE.
- P4 and P3 evaluator output directories are separate, preventing shared-argument collisions.
- RViz \`map\` fixed frame / corrected map visualization opened OpenGL 4.5 and shut down cleanly (exit 0).
- The saved map snapshot filenames use wall-clock microsecond timestamps, preventing normal repeat save requests from overwriting the same snapshot.

The reported 0.02693 m RMSE is from an *open synthetic path with no loop*. It is not a live accuracy claim and does not measure the final retroactively optimized keyframe trajectory. Do not use it to infer loop-closure accuracy on real hardware.
