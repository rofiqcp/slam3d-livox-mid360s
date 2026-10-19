# Livox MID-360S — 3D SLAM for ROS 2

ROS 2 Humble workspace for Livox MID-360/MID-360S acquisition, LiDAR(-IMU) odometry, 3D mapping, pose-graph correction, relocalization, Nav2 bridging and dashboard visualization. This is **source code and reference configuration**, not a repository of recorded point clouds or deployed maps.

## Core packages

| Package | Responsibility |
| --- | --- |
| `livox_mid360_manager` | Device discovery, diagnostics, calibration helpers and launch orchestration |
| `livox_ros_driver2` | Livox driver interface and ROS 2 messages |
| `fast_lio` | LiDAR–IMU odometry and mapping |
| `kiss_icp` | LiDAR-only odometry baseline |
| `slam3d_pose_graph` | Loop closure and map optimization |
| `slam3d_relocalizer` | Localization against a saved map |
| `slam3d_nav2_bridge` | Costmap/TF bridge and fail-closed localization gate |
| `slam3d_mid360_dashboard` | 3D mapping status and local dashboard |

Optional third-party source under `external/` and `third_party/` supports CT-ICP and MAD-ICP workflows. Keep upstream license notices and do not delete or refactor bundled vendor sources during documentation cleanup. See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Checkout paths

| Machine | Workspace |
| --- | --- |
| Sirobo | `/media/sirobo/HDD/Project/slam3d-livox-mid360s` |
| MiniPC1 | `/home/otomasi/Project/slam3d-livox-mid360s` |

The checked-in wrappers resolve their own project directory. ROS 2 Humble and the relevant Livox SDK, CMake, Eigen and SLAM dependencies must already be available. MiniPC1's low-memory build wrapper places colcon outputs in a host-local cache; code and dependency sources remain in the checkout.

## Build and safe start

```bash
cd /path/to/slam3d-livox-mid360s
./build.sh
source ./env.sh
```

Inspect `config/livox_runtime.yaml` and the sensor interface/IP before any device setup. The tracked YAML is a **starting reference**, not proof that the current sensor has the same IP address. Do not modify network interfaces without authorization.

Read-only network discovery (when the hardware is connected):

```bash
ros2 run livox_mid360_manager livox_discover --interface auto --timeout 5
```

The discovery utility may update runtime YAML after it identifies a sensor. Review `git status` afterward and keep per-host configuration differences local.

For a configured LiDAR:

```bash
ros2 launch livox_mid360_manager sensor.launch.py
ros2 topic list | grep livox
```

The driver and monitor normally expose `/livox/lidar` and `/livox/imu` (confirm against current launch configuration).

## Mapping and navigation

Maintained launch and configuration entry points are in `src/livox_mid360_manager/launch/`, `src/fast_lio/launch/`, `src/slam3d_pose_graph/launch/`, `src/slam3d_relocalizer/launch/` and `src/slam3d_nav2_bridge/launch/`.

- Stage 1: sensor quality, time alignment and recording.
- Stage 2: LiDAR-only odometry baselines (KISS-ICP, optional CT/MAD-ICP).
- Stage 3: LiDAR–IMU fusion with FAST-LIO.
- Stage 4: pose graph and loop closure.
- Stage 5: saved-map localization and recovery.
- Stage 6: TF/costmap/Nav2 integration with a fail-closed motion gate.
- Stage 7: dashboard and visualization.

Detailed design specifications are organized in [docs/stages/](docs/stages/). For a concise overview and next steps see [ROADMAP.md](ROADMAP.md).

**Localization quality is not a guarantee of actuator safety.** Nav2 commands must remain inhibited if required TF, sensor freshness, map alignment or pose confidence are invalid. Running a mapping demo is not equivalent to proving safe autonomous driving.

## Git and dataset policy

Only maintained C++/Python source, build definitions, checked-in sample configurations, necessary upstream source and documentation belong on GitHub. Recorded bags, maps, LiDAR point clouds, generated configs, build artifacts, calibration captures and research reports remain local. Unit tests supplied with required upstream dependencies are kept when needed for reproducible builds; ad-hoc runs and results are not committed.

See [AGENTS.md](AGENTS.md) for AI rules, [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) for validation and [SYNC_GUIDE.md](SYNC_GUIDE.md) for two-PC Git usage.
