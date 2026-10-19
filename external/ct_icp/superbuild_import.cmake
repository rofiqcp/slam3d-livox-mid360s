find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(nanoflann REQUIRED)

if(NOT TARGET tsl::robin_map)
  add_library(ct_tsl_robin INTERFACE)
  target_include_directories(ct_tsl_robin INTERFACE /usr/include)
  add_library(tsl::robin_map ALIAS ct_tsl_robin)
endif()

if(NOT TARGET cereal)
  add_library(cereal INTERFACE)
  target_include_directories(cereal INTERFACE /usr/include)
endif()

if(NOT TARGET tclap::tclap)
  add_library(ct_tclap INTERFACE)
  target_include_directories(ct_tclap INTERFACE /usr/include)
  add_library(tclap::tclap ALIAS ct_tclap)
endif()

if(NOT TARGET tinyply::tinyply)
  add_library(ct_tinyply STATIC
    ${CMAKE_CURRENT_LIST_DIR}/vendor/tinyply/tinyply.cpp)
  target_include_directories(ct_tinyply PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/vendor/tinyply/include
    ${CMAKE_CURRENT_LIST_DIR}/vendor/tinyply/include/tinyply)
  add_library(tinyply::tinyply ALIAS ct_tinyply)
endif()

if(NOT TARGET colormap::colormap)
  add_library(ct_colormap INTERFACE)
  add_library(colormap::colormap ALIAS ct_colormap)
endif()
