cmake_minimum_required(VERSION 3.8)
project(livox_mid360_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic)

find_path(LIVOX_SDK_INCLUDE_DIR
  NAMES livox_lidar_api.h
  PATHS /usr/local/include
  REQUIRED)
find_library(LIVOX_SDK_LIBRARY
  NAMES livox_lidar_sdk_shared
  PATHS /usr/local/lib
  REQUIRED)

add_executable(livox_discover src/livox_discover.cpp)
target_include_directories(livox_discover PRIVATE ${LIVOX_SDK_INCLUDE_DIR})
target_link_libraries(livox_discover ${LIVOX_SDK_LIBRARY} Threads::Threads)

add_executable(livox_network_setup src/livox_network_setup.cpp)
target_include_directories(livox_network_setup PRIVATE include)
ament_target_dependencies(livox_network_setup ament_index_cpp)

add_executable(livox_monitor src/livox_monitor.cpp)
ament_target_dependencies(livox_monitor rclcpp sensor_msgs)

add_executable(livox_diagnostics src/livox_diagnostics.cpp)
ament_target_dependencies(livox_diagnostics rclcpp sensor_msgs diagnostic_msgs)

add_executable(imu_stationary_stats src/imu_stationary_stats.cpp)
ament_target_dependencies(imu_stationary_stats rclcpp sensor_msgs)

add_executable(calibration_prepare src/calibration_prepare.cpp)
target_include_directories(calibration_prepare PRIVATE include)

add_executable(stage2_odom_evaluator src/stage2_odom_evaluator.cpp)
ament_target_dependencies(stage2_odom_evaluator rclcpp nav_msgs geometry_msgs diagnostic_msgs)

add_executable(cloud_to_kitti src/cloud_to_kitti.cpp)
ament_target_dependencies(cloud_to_kitti rclcpp sensor_msgs)

add_executable(cloud_to_ct_ply src/cloud_to_ct_ply.cpp)
ament_target_dependencies(cloud_to_ct_ply rclcpp sensor_msgs)

add_executable(stage2_synthetic_lidar src/stage2_synthetic_lidar.cpp)
ament_target_dependencies(stage2_synthetic_lidar rclcpp sensor_msgs nav_msgs geometry_msgs)

add_executable(stage3_synthetic_imu src/stage3_synthetic_imu.cpp)
ament_target_dependencies(stage3_synthetic_imu rclcpp sensor_msgs)

install(TARGETS
  livox_discover
  livox_network_setup
  livox_monitor
  livox_diagnostics
  imu_stationary_stats
  calibration_prepare
  stage2_odom_evaluator
  cloud_to_kitti
  cloud_to_ct_ply
  stage2_synthetic_lidar
  stage3_synthetic_imu
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
