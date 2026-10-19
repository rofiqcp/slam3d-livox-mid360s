# livox_ros_driver2 - ROS 2 MID-360S only

Clean ROS 2 Humble Livox driver used by /media/sirobo/HDD/slam3d-livox-mid360s.

Supported target: Livox MID-360S, SDK device type 35. ROS 1 code, ROS 1 launch files and configs for other Livox models are intentionally removed.

Output topics in single-LiDAR mode:
- /livox/lidar
- /livox/imu
- frame: livox_frame

Transfer formats kept for standard Livox compatibility:
- xfer_format 0: Livox PointCloud2 XYZRTLT, default for this workspace
- xfer_format 1: Livox CustomMsg with per-point offsets
- xfer_format 2: standard PointCloud2 XYZI

Custom messages are retained because Livox-oriented LIO/calibration algorithms can require per-point timing.

IMU units verified from source:
- angular velocity: rad/s
- linear acceleration: raw Livox g
The Stage 1 tools in the parent workspace measure the real stationary scale before later estimators use it.

Standalone build/run:
  cd /media/sirobo/HDD/slam3d-livox-mid360s
  ./build.sh
  source env.sh
  ros2 launch livox_ros_driver2 mid360s.launch.py

The package-local example config uses the last known working network host 192.168.1.10 and MID-360S 192.168.1.197. Normal project operation should use livox_mid360_manager, which discovers the actual network and generates runtime JSON automatically.

All C++ files remaining under src are compiled into the ROS 2 driver. The JSON parser uses the system rapidjson-dev package.

Removed intentionally: ROS 1 headers and source branches, ROS 1 launch files, duplicate vendor manifests, non-MID360S configs/launches, vendor build wrapper, unused APR and rosbag dependencies, and generic multi-device vendor documentation.
