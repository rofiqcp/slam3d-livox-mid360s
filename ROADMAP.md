# Development Roadmap

Reference root: `/media/sirobo/HDD/Referensi-slam3d`

## Stage 0 - Network, driver, raw sensor and RViz

Status: implemented in this workspace.

Goal:
- automatic device/interface discovery;
- persistent YAML;
- PointCloud2 + IMU;
- health/rate monitoring;
- RViz.

References:
- `14_SENSOR_DRIVERS_LIVOX_MID360/Livox-SDK2`
- `14_SENSOR_DRIVERS_LIVOX_MID360/livox_ros_driver2_official`
- previous validated runtime under `16_END_TO_END_REFERENCE_STACKS/_mid360s_local_runtime`

Acceptance:
- stable `/livox/lidar`;
- stable `/livox/imu`;
- correct timestamps/frame;
- no packet starvation;
- RViz continuously displays the scan.

## Stage 1 - Recording, diagnostics and calibration baseline

Status: **implemented and offline-validated**. Live hardware acceptance is pending because the MID-360S is powered off.

Add:
- rosbag2 record/play presets;
- packet/topic-rate watchdog;
- sensor timestamp diagnostics;
- TF/extrinsic YAML;
- stationary IMU statistics;
- optional LiDAR-IMU calibration workflow.

References:
- `10_CALIBRATION_INITIALIZATION/LiDAR_IMU_Init`
- `10_CALIBRATION_INITIALIZATION/lidar_imu_calib`
- `10_CALIBRATION_INITIALIZATION/OA-LICalib`
- `13_EVALUATION_BENCHMARK/evo`

## Stage 2 - LiDAR-only odometry baseline

Status: **implemented and synthetic-validated** with KISS-ICP ROS2, MAD-ICP C++ and CT-ICP continuous-time C++. KISS-ICP is the live default, MAD-ICP the offline accuracy cross-check, and CT-ICP a working research comparator. Live MID-360S acceptance is pending because the sensor is powered off.

Purpose: separate LiDAR geometry quality from IMU/filter issues.

Recommended first:
- `01_LIDAR_ODOMETRY_SLAM/kiss-icp`
- `01_LIDAR_ODOMETRY_SLAM/ct_icp`
- `01_LIDAR_ODOMETRY_SLAM/mad-icp`

Measure:
- odometry smoothness;
- CPU;
- drift;
- behavior during fast turns and feature-poor scenes.

## Stage 3 - LiDAR-Inertial Odometry

Status: **FAST-LIO ROS2 implemented and synthetic + rosbag validated**. MID-360S live acceptance/extrinsic calibration pending.

Recommended order for MID-360S:
1. `02_LIDAR_INERTIAL_LIO/FAST_LIO`
2. `02_LIDAR_INERTIAL_LIO/Point-LIO`
3. `02_LIDAR_INERTIAL_LIO/glim_ros2`
4. `02_LIDAR_INERTIAL_LIO/LIO-SAM_MID360_ROS2`
5. `02_LIDAR_INERTIAL_LIO/Voxel-SLAM`

Acceptance:
- stable 6DoF odometry;
- map persists as the robot moves;
- no runaway Z while stationary;
- correct gravity alignment;
- repeatable restart behavior.

## Stage 4 - Loop closure and globally consistent mapping

**Software status: implemented and validated with synthetic loop fixture + raw rosbag. Live MID-360S acceptance pending.**

Add loop closure only after Stage 3 is stable.

References:
- `07_LOOP_CLOSURE_PLACE_RECOGNITION/Fast-LIO2_SC_EGO_ROS2`
- `07_LOOP_CLOSURE_PLACE_RECOGNITION/SC-LIO-SAM`
- `07_LOOP_CLOSURE_PLACE_RECOGNITION/STD`
- `07_LOOP_CLOSURE_PLACE_RECOGNITION/scancontext_tro`
- `06_GRAPH_OPTIMIZATION_BACKEND/gtsam`

Acceptance:
- revisiting an area closes the loop;
- old map and current scan realign;
- no large discontinuity after loop optimization.

## Stage 5 - Persistent 3D map and export

**P5 software status: implemented and fixture-validated; physical MID-360S validation pending.** See STAGE5.md and STAGE5_AUDIT_20261008.md.

Add:
- save/load point-cloud map;
- PCD/PLY export;
- optional voxel/octree representation;
- map metadata;
- session naming/versioning.

References:
- `09_DENSE_MAPPING_RECONSTRUCTION/octomap`
- `09_DENSE_MAPPING_RECONSTRUCTION/voxblox`
- `09_DENSE_MAPPING_RECONSTRUCTION/nvblox`
- `02_LIDAR_INERTIAL_LIO/glim`
- `02_LIDAR_INERTIAL_LIO/Voxel-SLAM`

## Stage 6 - Localization against a saved map

References:
- `05_REGISTRATION_LOCALIZATION/FAST_LIO_LOCALIZATION`
- `05_REGISTRATION_LOCALIZATION/fast_lio2_mapping_and_localization`
- `05_REGISTRATION_LOCALIZATION/hdl_global_localization`
- `05_REGISTRATION_LOCALIZATION/small_gicp`

Acceptance:
- cold-start localization;
- relocalization after tracking loss;
- localization-only mode without modifying the reference map.

## Stage 7 - AGV/Nav2 integration

Add:
- finalized TF tree: map -> odom -> base_link -> livox_frame;
- 3D obstacle projection to Nav2-compatible costmaps;
- optional 2D occupancy export;
- launch composition with robot bring-up;
- lifecycle/error handling;
- automatic recovery if Livox disconnects/reconnects.

Useful references:
- `12_ROS_FRAMEWORKS_TOOLBOX/lidar_slam_ros2`
- `12_ROS_FRAMEWORKS_TOOLBOX/mola`
- `16_END_TO_END_REFERENCE_STACKS/dddmr_navigation`
- `16_END_TO_END_REFERENCE_STACKS/lightning-lm-deep-robotics`

## Stage 8 - Quantitative validation

Use recorded repeatable routes.

Measure:
- APE/RPE with `evo`;
- point-cloud and IMU rate/jitter;
- CPU/RAM;
- map consistency;
- loop-closure correction magnitude;
- localization recovery time;
- startup/reconnect robustness.

Only promote a later stage when the previous stage has a repeatable acceptance test.


## P6 implemented — Nav2 bridge and motion guard

**Software status: implemented, full workspace 7/7 packages build, Nav2 Humble lifecycle fixture and software interlock fixture validated. Hardware field acceptance PENDING.**

P6 adds `slam3d_nav2_bridge`, corrects the TF chain for base_link, projects Stage4 PCD to Nav2's 2D occupancy, uses MID-360S PointCloud2 as a costmap source and gates motor-intended `/cmd_vel_safe` by P5 tracking, TF freshness, odometry, operator ARM, software E-STOP and watchdogs.

The existing Stage 7 roadmap entry for AGV/Nav2 integration remains a **physical/hardware completion** requirement: verify real rover footprint, Ackermann steering behavior, speed envelope, hardwired E-STOP, motor watchdog and a live MID-360S route before operating the actual AGV. See STAGE6.md and STAGE6_AUDIT_20261008.md.
