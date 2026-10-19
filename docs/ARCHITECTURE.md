# ROS 2 SLAM architecture

```text
Livox MID-360S
  |                         |
  v                         v
lidar point cloud        IMU samples
  |                         |
  +--> KISS-ICP (baseline)   +--> FAST-LIO (Lidar/IMU)
                     |                      |
                     +------ odometry ------+
                               |
                               v
                        Pose graph / maps
                               |
                          Relocalization
                               |
                   Pose validity + TF gate
                               |
                       Nav2 integration
                               |
                    Dashboard / Foxglove
```

The pipeline uses multiple alternative odometry backends for comparison, not simultaneously authoritative sources. Only publish a consistent, timestamped transform chain and an explicit quality state to navigation consumers.

## Module boundaries

- `livox_mid360_manager`: runtime acquisition, network discovery, diagnostics and launch orchestration.
- `livox_ros_driver2`: upstream-based Livox point cloud/IMU transport.
- `fast_lio`, `kiss_icp`: odometry backends; CT-ICP and MAD-ICP are optional in `external/` and `third_party/`.
- `slam3d_pose_graph`: map alignment and optimization.
- `slam3d_relocalizer`: map-frame pose recovery with validity checks.
- `slam3d_nav2_bridge`: costmap/occupancy conversion, frame bridging and stop gate.
- `slam3d_mid360_dashboard`: operator view, not a safety-critical controller.

## Runtime inputs and portability

Live network and extrinsic settings come from YAML in `config/` and the package's `config/`, not from script-specific user paths. Build outputs and recorded maps are host-local. Do not commit sensor serial numbers, personalized addresses, captured environments or dataset files.

## Validation

Stage checks progress from static inspection and offline synthetic inputs, through ROS launch/build checks, to live read-only acquisition. Motion-enabled Nav2 checks require explicit authorization and verified independent stop/failsafe systems. Offline fixture success must never be presented as a real sensor validation.
