cmake_minimum_required(VERSION 3.16)
project(kiss_icp VERSION 1.3.0 LANGUAGES CXX)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Sophus REQUIRED NO_MODULE)
find_package(TBB REQUIRED)

add_library(kiss_vendor_robin INTERFACE)
target_include_directories(kiss_vendor_robin INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/vendor/robin_map/include)
add_library(tsl::robin_map ALIAS kiss_vendor_robin)

add_subdirectory(vendor/kiss_icp EXCLUDE_FROM_ALL)

add_library(odometry_component SHARED src/OdometryServer.cpp)
target_compile_features(odometry_component PUBLIC cxx_std_20)
target_include_directories(odometry_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(
  odometry_component
  PRIVATE kiss_icp_pipeline
          rclcpp::rclcpp
          rclcpp_components::component
          rcutils::rcutils
          tf2_ros::tf2_ros
          ${geometry_msgs_TARGETS}
          ${nav_msgs_TARGETS}
          ${sensor_msgs_TARGETS}
          ${std_msgs_TARGETS}
          ${std_srvs_TARGETS})

rclcpp_components_register_node(
  odometry_component
  PLUGIN "kiss_icp_ros::OdometryServer"
  EXECUTABLE kiss_icp_node)

install(TARGETS odometry_component LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  add_executable(timestamps_test tests/timestamps_test.cpp)
  target_compile_features(timestamps_test PRIVATE cxx_std_20)
  target_include_directories(timestamps_test PRIVATE src)
  target_link_libraries(timestamps_test Sophus::Sophus rclcpp::rclcpp ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS})
  add_test(NAME timestamps_regression COMMAND timestamps_test)
endif()
ament_package()
