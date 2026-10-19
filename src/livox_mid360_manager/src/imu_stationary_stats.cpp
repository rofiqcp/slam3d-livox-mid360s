#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

struct Stats {
  uint64_t n{0};
  double mean{0.0};
  double m2{0.0};

  void add(double x) {
    ++n;
    const double d = x - mean;
    mean += d / static_cast<double>(n);
    const double d2 = x - mean;
    m2 += d * d2;
  }

  double stddev() const {
    return n > 1 ? std::sqrt(m2 / static_cast<double>(n - 1)) : 0.0;
  }
};

class ImuStationaryStats : public rclcpp::Node {
 public:
  ImuStationaryStats() : Node("livox_imu_stationary_stats") {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
    duration_s_ = declare_parameter<double>("duration_seconds", 10.0);
    min_samples_ = declare_parameter<int>("min_samples", 200);
    wait_timeout_s_ = declare_parameter<double>("wait_timeout_seconds", 30.0);
    gyro_mean_limit_ = declare_parameter<double>("gyro_norm_mean_limit", 0.05);
    accel_std_limit_ = declare_parameter<double>("accel_norm_std_limit", 0.03);
    gyro_std_limit_ = declare_parameter<double>("gyro_norm_std_limit", 0.02);

    std::string default_output;
    if (const char *cfg = std::getenv("LIVOX_MID360_CONFIG"); cfg && *cfg) {
      const auto parent = std::filesystem::path(cfg).parent_path();
      default_output = (parent / "imu_stationary.yaml").string();
    } else {
      default_output = "imu_stationary.yaml";
    }
    output_path_ = declare_parameter<std::string>("output_path", default_output);

    node_start_ = std::chrono::steady_clock::now();

    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ImuStationaryStats::on_imu, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&ImuStationaryStats::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "Keep the LiDAR/IMU completely still. Collecting %.1f s from %s -> %s",
      duration_s_, imu_topic_.c_str(), output_path_.c_str());
  }

 private:
  void on_imu(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    if (finished_) return;

    if (!started_) {
      started_ = true;
      first_sample_ = std::chrono::steady_clock::now();
      first_stamp_ = rclcpp::Time(msg->header.stamp);
    }

    const double ax = msg->linear_acceleration.x / 9.80665;
    const double ay = msg->linear_acceleration.y / 9.80665;
    const double az = msg->linear_acceleration.z / 9.80665;
    const double gx = msg->angular_velocity.x;
    const double gy = msg->angular_velocity.y;
    const double gz = msg->angular_velocity.z;

    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az) ||
        !std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz)) {
      ++invalid_samples_; return;
    }
    const auto stamp = rclcpp::Time(msg->header.stamp);
    if (samples_ && ((stamp-last_stamp_).seconds() <= 0 ||
                     (stamp-last_stamp_).seconds() > 0.1)) ++sample_gaps_;
    ax_.add(ax);
    ay_.add(ay);
    az_.add(az);
    gx_.add(gx);
    gy_.add(gy);
    gz_.add(gz);

    accel_norm_.add(std::sqrt(ax * ax + ay * ay + az * az));
    gyro_norm_.add(std::sqrt(gx * gx + gy * gy + gz * gz));

    ++samples_;
    last_stamp_ = rclcpp::Time(msg->header.stamp);
  }

  void on_timer() {
    if (finished_) return;

    const auto now_steady = std::chrono::steady_clock::now();
    const double node_elapsed =
      std::chrono::duration<double>(now_steady - node_start_).count();

    if (!started_) {
      if (node_elapsed > wait_timeout_s_) {
        RCLCPP_ERROR(
          get_logger(), "No IMU data received within %.1f s. Nothing written.", wait_timeout_s_);
        finished_ = true;
        rclcpp::shutdown();
      }
      return;
    }

    const double collect_elapsed =
      std::chrono::duration<double>(now_steady - first_sample_).count();

    if (collect_elapsed >= duration_s_) {
      finish(collect_elapsed);
    }
  }

  void finish(double elapsed) {
    finished_ = true;

    const bool enough = static_cast<int>(samples_) >= min_samples_;
    const bool stationary =
      enough && invalid_samples_ == 0 && sample_gaps_ == 0 &&
      accel_norm_.mean >= 0.8 && accel_norm_.mean <= 1.2 &&
      gyro_norm_.mean <= gyro_mean_limit_ &&
      gyro_norm_.stddev() <= gyro_std_limit_ &&
      accel_norm_.stddev() <= accel_std_limit_;

    const double gravity_scale =
      accel_norm_.mean > 1e-9 ? 9.80665 / accel_norm_.mean : 0.0;

    const auto out_path = std::filesystem::path(output_path_);
    if (!out_path.parent_path().empty()) {
      std::filesystem::create_directories(out_path.parent_path());
    }

    std::ofstream out(output_path_);
    if (!out) {
      RCLCPP_ERROR(get_logger(), "Cannot write %s", output_path_.c_str());
      rclcpp::shutdown();
      return;
    }

    out << std::setprecision(10);
    out << "# Auto-generated stationary baseline for Livox built-in IMU.\n";
    out << "# Input uses m/s^2 and rad/s; acceleration statistics below are converted to g.\n";
    out << "imu_stationary:\n";
    out << "  topic: " << imu_topic_ << "\n";
    out << "  samples: " << samples_ << "\n";
    out << "  wall_duration_s: " << elapsed << "\n";
    out << "  sensor_duration_s: "
        << ((last_stamp_ - first_stamp_).seconds()) << "\n";
    out << "  acceleration_unit: g\n";
    out << "  angular_velocity_unit: rad/s\n";
    out << "  stationary_ok: " << (stationary ? "true" : "false") << "\n";
    out << "  accel_mean_g: [" << ax_.mean << ", " << ay_.mean << ", " << az_.mean << "]\n";
    out << "  accel_std_g: [" << ax_.stddev() << ", " << ay_.stddev() << ", " << az_.stddev() << "]\n";
    out << "  accel_norm_mean_g: " << accel_norm_.mean << "\n";
    out << "  accel_norm_std_g: " << accel_norm_.stddev() << "\n";
    out << "  gyro_mean_rad_s: [" << gx_.mean << ", " << gy_.mean << ", " << gz_.mean << "]\n";
    out << "  gyro_std_rad_s: [" << gx_.stddev() << ", " << gy_.stddev() << ", " << gz_.stddev() << "]\n";
    out << "  gyro_norm_mean_rad_s: " << gyro_norm_.mean << "\n";
    out << "  gyro_norm_std_rad_s: " << gyro_norm_.stddev() << "\n";
    out << "  accel_multiplier_to_mps2: " << gravity_scale << "\n";
    out << "  li_init_mean_acc_norm: " << accel_norm_.mean * 9.80665 << "\n";
    out << "thresholds:\n";
    out << "  min_samples: " << min_samples_ << "\n";
    out << "  gyro_norm_mean_limit_rad_s: " << gyro_mean_limit_ << "\n";
    out << "  gyro_norm_std_limit_rad_s: " << gyro_std_limit_ << "\n";
    out << "  accel_norm_std_limit_g: " << accel_std_limit_ << "\n";

    out.close();

    RCLCPP_INFO(
      get_logger(),
      "Saved %lu samples. accel_norm=%.6f g std=%.6f, gyro_norm=%.6f rad/s std=%.6f, stationary=%s",
      static_cast<unsigned long>(samples_), accel_norm_.mean, accel_norm_.stddev(),
      gyro_norm_.mean, gyro_norm_.stddev(), stationary ? "YES" : "NO");
    RCLCPP_INFO(
      get_logger(), "Recommended acceleration conversion multiplier: %.6f m/s^2 per raw g-unit",
      gravity_scale);
    RCLCPP_INFO(get_logger(), "Output: %s", output_path_.c_str());

    rclcpp::shutdown();
  }

  std::string imu_topic_;
  std::string output_path_;
  double duration_s_{10.0};
  int min_samples_{200};
  double wait_timeout_s_{30.0};
  double gyro_mean_limit_{0.05};
  double accel_std_limit_{0.03};
  double gyro_std_limit_{0.02};

  bool started_{false};
  bool finished_{false};
  uint64_t samples_{0};

  size_t invalid_samples_{0}, sample_gaps_{0};
  Stats ax_, ay_, az_, gx_, gy_, gz_, accel_norm_, gyro_norm_;

  std::chrono::steady_clock::time_point node_start_;
  std::chrono::steady_clock::time_point first_sample_;
  rclcpp::Time first_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuStationaryStats>());
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
