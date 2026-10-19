cmake_minimum_required(VERSION 3.16)
project(mad_icp_mid360_baseline LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)
link_libraries(OpenMP::OpenMP_CXX)

add_compile_options(-Wall -Wextra -Wpedantic)
include_directories(${PROJECT_SOURCE_DIR}/src)

add_subdirectory(src/tools)
add_subdirectory(src/odometry)
add_subdirectory(apps/cpp_runners)
