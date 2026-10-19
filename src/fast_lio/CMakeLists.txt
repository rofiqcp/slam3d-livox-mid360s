cmake_minimum_required(VERSION 3.16)
project(fast_lio VERSION 1.0.0 LANGUAGES C CXX)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl" CACHE PATH "Match ROS 2 Humble PCL 1.12 ABI" FORCE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(OpenMP)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Pose6D.msg")
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(fastlio_mapping
  src/laserMapping.cpp
  src/preprocess.cpp
  include/ikd-Tree/ikd_Tree.cpp
)
target_include_directories(fastlio_mapping PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${CMAKE_CURRENT_BINARY_DIR}"
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(fastlio_mapping
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  ${cpp_typesupport_target}
)
if(OpenMP_CXX_FOUND)
  target_compile_definitions(fastlio_mapping PRIVATE MP_EN)
  target_link_libraries(fastlio_mapping OpenMP::OpenMP_CXX)
endif()
ament_target_dependencies(fastlio_mapping
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs
  visualization_msgs pcl_conversions livox_ros_driver2 tf2_ros
)

install(TARGETS fastlio_mapping DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
