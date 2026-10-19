#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "livox_mid360_manager/yaml_utils.hpp"

namespace fs = std::filesystem;

int main(int argc, char **argv) {
  std::string stationary;
  std::string output;

  const char *root = std::getenv("LIVOX_MID360_ROOT");
  if (root && *root) {
    stationary = std::string(root) + "/config/imu_stationary.yaml";
    output = std::string(root) + "/config/li_init_mid360_generated.yaml";
  } else {
    stationary = "imu_stationary.yaml";
    output = "li_init_mid360_generated.yaml";
  }

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&](const char *opt) {
      if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + opt);
      return std::string(argv[++i]);
    };
    if (a == "--stationary") stationary = need("--stationary");
    else if (a == "--output") output = need("--output");
    else if (a == "-h" || a == "--help") {
      std::cout << "Usage: calibration_prepare [--stationary YAML] [--output YAML]\n";
      return 0;
    }
  }

  const auto mean_acc =
    livox_mid360_manager::yaml_scalar(stationary, "imu_stationary", "accel_norm_mean_g");
  const auto stationary_ok =
    livox_mid360_manager::yaml_scalar(stationary, "imu_stationary", "stationary_ok");

  if (!mean_acc) {
    std::cerr << "Missing imu_stationary.accel_norm_mean_g in " << stationary << "\n"
              << "Run imu_stationary_stats first with the real sensor powered.\n";
    return 2;
  }

  double acc_norm = 0.0;
  try {
    acc_norm = std::stod(*mean_acc);
  } catch (...) {
    std::cerr << "Invalid accel_norm_mean_g: " << *mean_acc << "\n";
    return 3;
  }

  if (!stationary_ok || *stationary_ok != "true") {
    std::cerr << "Stationary IMU validation must pass before calibration\n";
    return 4;
  }
  if (!std::isfinite(acc_norm) || acc_norm < 0.8 || acc_norm > 1.2) {
    std::cerr << "accel_norm_mean_g must be finite and within [0.8, 1.2]\n";
    return 4;
  }

  const auto parent = fs::path(output).parent_path();
  if (!parent.empty()) fs::create_directories(parent);

  std::ofstream out(output);
  if (!out) {
    std::cerr << "Cannot write " << output << "\n";
    return 5;
  }

  out << "# Generated from real stationary IMU statistics.\n";
  out << "# IMPORTANT: upstream LiDAR_IMU_Init is ROS1. For lidar_type=1 it expects\n";
  out << "# livox_ros_driver/CustomMsg, while this ROS2 workspace publishes PointCloud2.\n";
  out << "# Use this as calibrated parameter input after a verified ROS2 port/adapter.\n";
  out << "common:\n";
  out << "  lid_topic: /livox/lidar\n";
  out << "  imu_topic: /livox/imu\n\n";
  out << "preprocess:\n";
  out << "  lidar_type: 1\n";
  out << "  feature_extract_en: false\n";
  out << "  scan_line: 6\n";
  out << "  blind: 1.0\n\n";
  out << "initialization:\n";
  out << "  cut_frame_num: 5\n";
  out << "  orig_odom_freq: 10\n";
  out << "  mean_acc_norm: " << acc_norm * 9.80665 << "\n";
  out << "  online_refine_time: 20\n";
  out << "  data_accum_length: 500\n";
  out << "  Rot_LI_cov: [0.00005, 0.00005, 0.00005]\n";
  out << "  Trans_LI_cov: [0.00001, 0.00001, 0.00001]\n\n";
  out << "mapping:\n";
  out << "  filter_size_surf: 0.05\n";
  out << "  filter_size_map: 0.15\n";
  out << "  gyr_cov: 2.0\n";
  out << "  acc_cov: 2.0\n";
  out << "  b_acc_cov: 0.0001\n";
  out << "  b_gyr_cov: 0.0001\n";
  out << "  det_range: 100.0\n\n";
  out << "compatibility:\n";
  out << "  source_ros: ROS2 Humble\n";
  out << "  source_lidar_message: sensor_msgs/PointCloud2\n";
  out << "  source_imu_message: sensor_msgs/Imu\n";
  out << "  upstream_calibrator_ros: ROS1\n";
  out << "  upstream_livox_message: livox_ros_driver/CustomMsg\n";
  out << "  ros2_adapter_or_port_required: true\n";
  out << "  stationary_baseline_ok: "
      << (stationary_ok ? *stationary_ok : "unknown") << "\n";

  std::cout << "Generated calibration parameter bridge: " << output << "\n";
  std::cout << "mean_acc_norm=" << acc_norm * 9.80665 << "\n";
  std::cout << "NOTE: do not run upstream LI-Init directly on ROS2 PointCloud2 without a verified adapter/port.\n";
  return 0;
}
