#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class LivoxMonitor : public rclcpp::Node {
 public:
  LivoxMonitor() : Node("livox_mid360_monitor") {
    point_topic_ = declare_parameter<std::string>("point_topic", "/livox/lidar");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
    report_period_ = declare_parameter<double>("report_period", 2.0);

    auto qos = rclcpp::SensorDataQoS();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      point_topic_, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        ++cloud_count_;
        last_points_ = static_cast<uint64_t>(msg->width) *
                       static_cast<uint64_t>(msg->height);
        last_frame_ = msg->header.frame_id;
        last_cloud_time_ = now();
      });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr) {
        ++imu_count_;
        last_imu_time_ = now();
      });

    last_report_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(report_period_ * 1000.0)),
      std::bind(&LivoxMonitor::report, this));
  }

 private:
  void report() {
    const auto current = now();
    const double dt = std::max(1e-6, (current - last_report_).seconds());
    const auto clouds = cloud_count_.exchange(0);
    const auto imus = imu_count_.exchange(0);

    const double cloud_age =
      last_cloud_time_.nanoseconds() == 0 ? -1.0 : (current - last_cloud_time_).seconds();
    const double imu_age =
      last_imu_time_.nanoseconds() == 0 ? -1.0 : (current - last_imu_time_).seconds();

    RCLCPP_INFO(
      get_logger(),
      "cloud=%.2f Hz age=%.2fs points=%lu frame=%s | imu=%.2f Hz age=%.2fs",
      clouds / dt, cloud_age, static_cast<unsigned long>(last_points_),
      last_frame_.empty() ? "<none>" : last_frame_.c_str(),
      imus / dt, imu_age);

    last_report_ = current;
  }

  std::string point_topic_;
  std::string imu_topic_;
  std::string last_frame_;
  double report_period_{2.0};

  std::atomic<uint64_t> cloud_count_{0};
  std::atomic<uint64_t> imu_count_{0};
  uint64_t last_points_{0};

  rclcpp::Time last_cloud_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_report_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxMonitor>());
  rclcpp::shutdown();
  return 0;
}
