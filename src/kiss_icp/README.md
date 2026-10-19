# KISS-ICP ROS 2 baseline

Vendored from /media/sirobo/HDD/Referensi-slam3d/01_LIDAR_ODOMETRY_SLAM/kiss-icp at commit 1ffa7d7512f10bfc8b1185095011fa31184019e3 (v1.3.0-1-g1ffa7d7).

This copy is intentionally offline/deterministic: the C++ core is vendored locally and no FetchContent download is used. Robin-map headers are sourced from the local Traj-LO reference corpus.

Livox MID-360S input: /livox/lidar, PointCloud2 xfer_format=0. The Livox timestamp FLOAT64 field is recognized directly by the KISS ROS2 wrapper, so scan deskewing remains enabled.

Output topics include /kiss/odometry, /kiss/frame, /kiss/keypoints and /kiss/local_map. TF uses odom_lidar -> livox_frame for the pure LiDAR baseline.
