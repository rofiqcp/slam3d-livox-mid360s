#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#pragma pack(push, 1)
struct LivoxSyntheticPoint {
  float x;
  float y;
  float z;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double timestamp;
};
#pragma pack(pop)

struct WorldPoint {
  double x;
  double y;
  double z;
  float intensity;
};

class SyntheticLidarMotion : public rclcpp::Node {
 public:
  SyntheticLidarMotion() : Node("stage2_synthetic_lidar") {
    topic_ = declare_parameter<std::string>("topic", "/stage2/synthetic/lidar");
    gt_topic_ = declare_parameter<std::string>("ground_truth_topic", "/stage2/synthetic/ground_truth");
    frame_id_ = declare_parameter<std::string>("frame_id", "livox_frame");
    world_frame_ = declare_parameter<std::string>("world_frame", "synthetic_world");
    rate_hz_ = declare_parameter<double>("rate_hz", 10.0);
    forward_speed_mps_ = declare_parameter<double>("forward_speed_mps", 0.5);
    yaw_rate_rad_s_ = declare_parameter<double>("yaw_rate_rad_s", 0.03);
    lateral_amplitude_m_ = declare_parameter<double>("lateral_amplitude_m", 0.4);
    max_range_m_ = declare_parameter<double>("max_range_m", 45.0);

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      topic_, rclcpp::SensorDataQoS());
    gt_pub_ = create_publisher<nav_msgs::msg::Odometry>(gt_topic_, 10);

    build_world();

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SyntheticLidarMotion::tick, this));

    RCLCPP_INFO(
      get_logger(), "Synthetic Stage2 LiDAR: %zu world points, %.1f Hz",
      world_.size(), rate_hz_);
  }

 private:
  void add(double x, double y, double z, float intensity = 80.0f) {
    world_.push_back({x, y, z, intensity});
  }

  void build_world() {
    // Floor with non-uniform intensity.
    for (double x = -10.0; x <= 30.0; x += 0.5) {
      for (double y = -10.0; y <= 10.0; y += 0.5) {
        add(x, y, 0.0, static_cast<float>(30.0 + std::fmod(std::abs(x * 7 + y * 11), 100.0)));
      }
    }

    // Side walls.
    for (double x = -10.0; x <= 30.0; x += 0.35) {
      for (double z = 0.4; z <= 4.0; z += 0.4) {
        add(x, -10.0, z, 120.0f);
        add(x, 10.0, z, 100.0f);
      }
    }

    // End walls.
    for (double y = -10.0; y <= 10.0; y += 0.35) {
      for (double z = 0.4; z <= 4.0; z += 0.4) {
        add(-10.0, y, z, 90.0f);
        add(30.0, y, z, 140.0f);
      }
    }

    // Asymmetric poles/objects to prevent planar ambiguity.
    const std::vector<std::pair<double,double>> poles = {
      {2.0, 3.0}, {5.0, -4.5}, {8.0, 6.0}, {12.0, -2.0},
      {15.0, 5.5}, {19.0, -6.0}, {24.0, 2.5}};
    for (size_t p = 0; p < poles.size(); ++p) {
      const auto [cx, cy] = poles[p];
      for (double z = 0.0; z <= 3.5; z += 0.15) {
        for (int a = 0; a < 12; ++a) {
          const double th = 2.0 * 3.14159265358979323846 * static_cast<double>(a) / 12.0;
          add(cx + 0.18 * std::cos(th), cy + 0.18 * std::sin(th), z,
              static_cast<float>(160 + p * 8));
        }
      }
    }

    // A raised box.
    for (double x = 10.0; x <= 12.0; x += 0.2) {
      for (double y = 2.0; y <= 4.0; y += 0.2) {
        add(x, y, 1.5, 210.0f);
      }
    }
  }

  void tick() {
    const auto stamp = now();
    const double t = static_cast<double>(frame_) / rate_hz_;

    const double tx = forward_speed_mps_ * t;
    const double ty = lateral_amplitude_m_ * std::sin(0.25 * t);
    const double tz = 1.2;
    const double yaw = yaw_rate_rad_s_ * t;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    std::vector<LivoxSyntheticPoint> points;
    points.reserve(world_.size());

    const uint64_t base_ns = static_cast<uint64_t>(stamp.nanoseconds());
    for (const auto &w : world_) {
      const double dx = w.x - tx;
      const double dy = w.y - ty;
      const double dz = w.z - tz;

      // world -> sensor, inverse of planar sensor pose.
      const double sx = c * dx + s * dy;
      const double sy = -s * dx + c * dy;
      const double sz = dz;
      const double r = std::sqrt(sx*sx + sy*sy + sz*sz);
      if (r < 1.0 || r > max_range_m_) continue;

      LivoxSyntheticPoint p{};
      p.x = static_cast<float>(sx);
      p.y = static_cast<float>(sy);
      p.z = static_cast<float>(sz);
      p.intensity = w.intensity;
      p.tag = 0;
      p.line = static_cast<uint8_t>(points.size() % 4);
      points.push_back(p);
    }

    const double scan_ns = 1.0e9 / rate_hz_;
    for (size_t i = 0; i < points.size(); ++i) {
      const double alpha = points.size() > 1
        ? static_cast<double>(i) / static_cast<double>(points.size() - 1) : 0.0;
      points[i].timestamp = static_cast<double>(base_ns) + alpha * scan_ns;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = sizeof(LivoxSyntheticPoint);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(7);

    auto field = [&](size_t idx, const std::string &name, uint32_t offset, uint8_t datatype) {
      cloud.fields[idx].name = name;
      cloud.fields[idx].offset = offset;
      cloud.fields[idx].count = 1;
      cloud.fields[idx].datatype = datatype;
    };
    field(0, "x", 0, sensor_msgs::msg::PointField::FLOAT32);
    field(1, "y", 4, sensor_msgs::msg::PointField::FLOAT32);
    field(2, "z", 8, sensor_msgs::msg::PointField::FLOAT32);
    field(3, "intensity", 12, sensor_msgs::msg::PointField::FLOAT32);
    field(4, "tag", 16, sensor_msgs::msg::PointField::UINT8);
    field(5, "line", 17, sensor_msgs::msg::PointField::UINT8);
    field(6, "timestamp", 18, sensor_msgs::msg::PointField::FLOAT64);

    cloud.data.resize(points.size() * sizeof(LivoxSyntheticPoint));
    if (!points.empty()) {
      std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
    }
    cloud_pub_->publish(cloud);

    nav_msgs::msg::Odometry gt;
    gt.header.stamp = stamp;
    gt.header.frame_id = world_frame_;
    gt.child_frame_id = frame_id_;
    gt.pose.pose.position.x = tx;
    gt.pose.pose.position.y = ty;
    gt.pose.pose.position.z = tz;
    gt.pose.pose.orientation.z = std::sin(0.5 * yaw);
    gt.pose.pose.orientation.w = std::cos(0.5 * yaw);
    gt.twist.twist.linear.x = forward_speed_mps_;
    gt.twist.twist.angular.z = yaw_rate_rad_s_;
    gt_pub_->publish(gt);

    ++frame_;
  }

  std::string topic_, gt_topic_, frame_id_, world_frame_;
  double rate_hz_{10.0}, forward_speed_mps_{0.5}, yaw_rate_rad_s_{0.03};
  double lateral_amplitude_m_{0.4}, max_range_m_{45.0};
  uint64_t frame_{0};

  std::vector<WorldPoint> world_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyntheticLidarMotion>());
  rclcpp::shutdown();
  return 0;
}
