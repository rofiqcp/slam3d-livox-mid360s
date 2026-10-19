#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

class Stage3SyntheticImu : public rclcpp::Node {
 public:
  Stage3SyntheticImu() : Node("stage3_synthetic_imu") {
    rate_hz_ = declare_parameter<double>("rate_hz", 200.0);
    yaw_rate_ = declare_parameter<double>("yaw_rate_rad_s", 0.03);
    lateral_amplitude_ = declare_parameter<double>("lateral_amplitude_m", 0.4);
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
    frame_id_ = declare_parameter<std::string>("frame_id", "livox_frame");
    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::QoS(rclcpp::KeepLast(100)).reliable());
    first_ = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&Stage3SyntheticImu::publish, this));
    RCLCPP_INFO(get_logger(), "Synthetic IMU at %.1f Hz, gyro rad/s, accel m/s^2", rate_hz_);
  }
 private:
  void publish() {
    const double t = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - first_).count();
    const double yaw = yaw_rate_ * t;
    const double ay_world = -lateral_amplitude_ * 0.25 * 0.25 *
                            std::sin(0.25 * t);
    constexpr double g = 9.80665;
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = frame_id_;
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity.z = yaw_rate_;
    imu.linear_acceleration.x = std::sin(yaw) * ay_world;
    imu.linear_acceleration.y = std::cos(yaw) * ay_world;
    imu.linear_acceleration.z = g;
    publisher_->publish(imu);
  }
  double rate_hz_{200.0};
  double yaw_rate_{0.03};
  double lateral_amplitude_{0.4};
  std::string imu_topic_;
  std::string frame_id_;
  std::chrono::steady_clock::time_point first_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Stage3SyntheticImu>());
  rclcpp::shutdown();
  return 0;
}
