# 3D SLAM roadmap

The target is a maintainable, reproducible Livox MID-360S ROS 2 pipeline for mapping, relocalization and safe Nav2 integration. **Previous stage reports are historical evidence, not a current guarantee of working sensor hardware or deployment readiness.**

| Stage | Maintained design document | Scope |
| --- | --- | --- |
| 1 | [Stage 1](docs/stages/STAGE1.md) | Sensor acquisition, IMU baseline, time synchronization |
| 2 | [Stage 2](docs/stages/STAGE2.md) | LiDAR-only odometry, baseline comparisons |
| 3 | [Stage 3](docs/stages/STAGE3.md) | FAST-LIO odometry and mapping |
| 4 | [Stage 4](docs/stages/STAGE4.md) | Loop closure, pose graph and corrected 3D map |
| 5 | [Stage 5](docs/stages/STAGE5.md) | Map loading and relocalization |
| 6 | [Stage 6](docs/stages/STAGE6.md) | Nav2 integration, localization validity and fail-closed gating |
| 7 | [Stage 7](docs/stages/STAGE7.md) | Operator dashboard and visual diagnostics |

## Next maintenance priorities

1. Revalidate dependencies and compile on both Linux hosts; document separate results instead of copying historical build claims.
2. Make runtime interface/IP and hardware-specific calibration configurable rather than fixed to one host.
3. Keep CI-level syntax, dependency and repository hygiene checks lightweight; use ROS-specific builds on prepared hosts.
4. Separate maps, rosbags, calibration datasets, performance plots, temporary test fixtures, and vendor artifacts from source control.
5. Validate failure behavior for stale point clouds, stale IMU, lost TF and relocalization uncertainty before authorizing motion.

Do not enable motion, change physical network settings or overwrite persistent map datasets as part of Git housekeeping.
