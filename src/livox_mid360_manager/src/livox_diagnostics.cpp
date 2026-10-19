#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

struct RunningStats {
  uint64_t n{0};
  double mean{0.0};
  double m2{0.0};

  void add(double x) {
    ++n;
    const double delta = x - mean;
    mean += delta / static_cast<double>(n);
    const double delta2 = x - mean;
    m2 += delta * delta2;
  }

  double stddev() const {
    return n > 1 ? std::sqrt(m2 / static_cast<double>(n - 1)) : 0.0;
  }

  void reset() {
    n = 0;
    mean = 0.0;
    m2 = 0.0;
  }
};

struct StreamState {
  uint64_t window_count{0};
  uint64_t total_count{0};
  uint64_t nonmonotonic_stamp_count{0};
  uint64_t gap_count{0};
  bool have_arrival{false};
  bool have_stamp{false};
  std::chrono::steady_clock::time_point last_arrival{};
  rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ros_arrival{0, 0, RCL_ROS_TIME};
  RunningStats arrival_dt;
  RunningStats stamp_dt;
};

class LivoxDiagnostics : public rclcpp::Node {
 public:
  LivoxDiagnostics() : Node("livox_mid360_diagnostics") {
    point_topic_ = declare_parameter<std::string>("point_topic", "/livox/lidar");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
    expected_cloud_hz_ = declare_parameter<double>("expected_cloud_hz", 10.0);
    expected_imu_hz_ = declare_parameter<double>("expected_imu_hz", 200.0);
    report_period_s_ = declare_parameter<double>("report_period", 1.0);
    startup_grace_s_ = declare_parameter<double>("startup_grace", 3.0);
    stale_timeout_s_ = declare_parameter<double>("stale_timeout", 1.0);
    warn_rate_ratio_ = declare_parameter<double>("warn_rate_ratio", 0.80);
    error_rate_ratio_ = declare_parameter<double>("error_rate_ratio", 0.50);
    gap_factor_ = declare_parameter<double>("gap_factor", 3.0);

    start_steady_ = std::chrono::steady_clock::now();
    report_steady_ = start_steady_;

    auto qos = rclcpp::SensorDataQoS();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      point_topic_, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        update_stream(cloud_, msg->header.stamp, expected_cloud_hz_);
        latest_cloud_points_ =
          static_cast<uint64_t>(msg->width) * static_cast<uint64_t>(msg->height);
        latest_cloud_frame_ = msg->header.frame_id;
      });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        update_stream(imu_, msg->header.stamp, expected_imu_hz_);
        latest_imu_frame_ = msg->header.frame_id;
      });

    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(report_period_s_ * 1000.0)),
      std::bind(&LivoxDiagnostics::report, this));
  }

 private:
  static std::string f3(double x) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(3) << x;
    return os.str();
  }

  static diagnostic_msgs::msg::KeyValue kv(const std::string &key, const std::string &value) {
    diagnostic_msgs::msg::KeyValue out;
    out.key = key;
    out.value = value;
    return out;
  }

  void update_stream(
      StreamState &s, const builtin_interfaces::msg::Time &stamp_msg, double expected_hz) {
    const auto steady_now = std::chrono::steady_clock::now();
    const rclcpp::Time stamp(stamp_msg);
    const auto ros_now = now();

    if (s.have_arrival) {
      const double dt =
        std::chrono::duration<double>(steady_now - s.last_arrival).count();
      if (dt > 0.0) s.arrival_dt.add(dt);
      if (expected_hz > 0.0 && dt > gap_factor_ / expected_hz) ++s.gap_count;
    }

    if (s.have_stamp) {
      const double dt = (stamp - s.last_stamp).seconds();
      if (dt <= 0.0) {
        ++s.nonmonotonic_stamp_count;
      } else {
        s.stamp_dt.add(dt);
      }
    }

    s.last_arrival = steady_now;
    s.last_ros_arrival = ros_now;
    s.last_stamp = stamp;
    s.have_arrival = true;
    s.have_stamp = true;
    ++s.window_count;
    ++s.total_count;
  }

  diagnostic_msgs::msg::DiagnosticStatus make_status(
      const std::string &name, const StreamState &s, double measured_hz,
      double expected_hz, const std::string &topic, const std::string &frame,
      uint64_t extra_value, const std::string &extra_key) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name;
    status.hardware_id = "livox_mid360s";

    const auto steady_now = std::chrono::steady_clock::now();
    const double uptime =
      std::chrono::duration<double>(steady_now - start_steady_).count();
    const double age = s.have_arrival
      ? std::chrono::duration<double>(steady_now - s.last_arrival).count()
      : -1.0;

    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";

    if (!s.have_arrival && uptime > startup_grace_s_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "no data";
    } else if (s.have_arrival && age > stale_timeout_s_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "stale data";
    } else if (s.have_arrival && expected_hz > 0.0 &&
               measured_hz < expected_hz * error_rate_ratio_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "rate too low";
    } else if (s.have_arrival && expected_hz > 0.0 &&
               measured_hz < expected_hz * warn_rate_ratio_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "rate below target";
    } else if (s.nonmonotonic_stamp_count > 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "non-monotonic sensor timestamp";
    }

    const double stamp_age = s.have_stamp ? (now() - s.last_stamp).seconds() : -1.0;

    status.values.push_back(kv("topic", topic));
    status.values.push_back(kv("frame_id", frame.empty() ? "<none>" : frame));
    status.values.push_back(kv("expected_hz", f3(expected_hz)));
    status.values.push_back(kv("measured_hz", f3(measured_hz)));
    status.values.push_back(kv("arrival_age_s", f3(age)));
    status.values.push_back(kv("arrival_period_mean_s", f3(s.arrival_dt.mean)));
    status.values.push_back(kv("arrival_jitter_std_s", f3(s.arrival_dt.stddev())));
    status.values.push_back(kv("sensor_period_mean_s", f3(s.stamp_dt.mean)));
    status.values.push_back(kv("sensor_jitter_std_s", f3(s.stamp_dt.stddev())));
    status.values.push_back(kv("sensor_stamp_minus_ros_now_s", s.have_stamp ? f3(-stamp_age) : "n/a"));
    status.values.push_back(kv("nonmonotonic_stamp_count", std::to_string(s.nonmonotonic_stamp_count)));
    status.values.push_back(kv("large_gap_count", std::to_string(s.gap_count)));
    status.values.push_back(kv("total_messages", std::to_string(s.total_count)));
    if (!extra_key.empty()) status.values.push_back(kv(extra_key, std::to_string(extra_value)));
    return status;
  }

  void report() {
    const auto steady_now = std::chrono::steady_clock::now();
    const double dt = std::max(
      1e-6, std::chrono::duration<double>(steady_now - report_steady_).count());

    const double cloud_hz = static_cast<double>(cloud_.window_count) / dt;
    const double imu_hz = static_cast<double>(imu_.window_count) / dt;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    auto cloud_status = make_status(
      "Livox MID360S/PointCloud", cloud_, cloud_hz, expected_cloud_hz_,
      point_topic_, latest_cloud_frame_, latest_cloud_points_, "latest_points");

    auto imu_status = make_status(
      "Livox MID360S/IMU", imu_, imu_hz, expected_imu_hz_,
      imu_topic_, latest_imu_frame_, 0, "");

    diagnostic_msgs::msg::DiagnosticStatus overall;
    overall.name = "Livox MID360S/Overall";
    overall.hardware_id = "livox_mid360s";
    overall.level = std::max(cloud_status.level, imu_status.level);
    if (overall.level == diagnostic_msgs::msg::DiagnosticStatus::OK) overall.message = "sensor streams healthy";
    else if (overall.level == diagnostic_msgs::msg::DiagnosticStatus::WARN) overall.message = "sensor stream warning";
    else overall.message = "sensor stream error";
    overall.values.push_back(kv("cloud_hz", f3(cloud_hz)));
    overall.values.push_back(kv("imu_hz", f3(imu_hz)));
    overall.values.push_back(kv("acceleration_unit", "m/s^2 (SI)"));
    overall.values.push_back(kv("angular_velocity_unit", "rad/s"));

    array.status.push_back(overall);
    array.status.push_back(cloud_status);
    array.status.push_back(imu_status);
    diag_pub_->publish(array);

    RCLCPP_INFO(
      get_logger(),
      "cloud %.2fHz jitter %.4fs gaps %lu | imu %.2fHz jitter %.5fs gaps %lu",
      cloud_hz, cloud_.arrival_dt.stddev(),
      static_cast<unsigned long>(cloud_.gap_count),
      imu_hz, imu_.arrival_dt.stddev(),
      static_cast<unsigned long>(imu_.gap_count));

    cloud_.window_count = 0;
    imu_.window_count = 0;
    cloud_.arrival_dt.reset();
    imu_.arrival_dt.reset();
    cloud_.stamp_dt.reset();
    imu_.stamp_dt.reset();
    report_steady_ = steady_now;
  }

  std::string point_topic_;
  std::string imu_topic_;
  std::string latest_cloud_frame_;
  std::string latest_imu_frame_;

  double expected_cloud_hz_{10.0};
  double expected_imu_hz_{200.0};
  double report_period_s_{1.0};
  double startup_grace_s_{3.0};
  double stale_timeout_s_{1.0};
  double warn_rate_ratio_{0.80};
  double error_rate_ratio_{0.50};
  double gap_factor_{3.0};

  uint64_t latest_cloud_points_{0};
  StreamState cloud_;
  StreamState imu_;

  std::chrono::steady_clock::time_point start_steady_;
  std::chrono::steady_clock::time_point report_steady_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxDiagnostics>());
  rclcpp::shutdown();
  return 0;
}
