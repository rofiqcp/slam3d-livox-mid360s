#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

class CloudToKitti : public rclcpp::Node {
 public:
  CloudToKitti() : Node("stage2_cloud_to_kitti") {
    topic_ = declare_parameter<std::string>("topic", "/livox/lidar");
    output_dir_ = declare_parameter<std::string>("output_dir", "");
    max_scans_ = declare_parameter<int>("max_scans", 0);

    if (output_dir_.empty()) {
      if (const char *root = std::getenv("LIVOX_MID360_ROOT"); root && *root) {
        output_dir_ = std::string(root) + "/datasets/stage2/kitti_scans";
      } else {
        output_dir_ = "kitti_scans";
      }
    }

    scans_dir_ = fs::path(output_dir_) / "scans";
    fs::create_directories(scans_dir_);
    index_ = find_next_index();

    timestamps_.open(fs::path(output_dir_) / "timestamps.txt", std::ios::app);
    if (!timestamps_) throw std::runtime_error("Cannot open timestamps.txt");

    metadata_path_ = fs::path(output_dir_) / "metadata.yaml";

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_, rclcpp::SensorDataQoS(),
      std::bind(&CloudToKitti::on_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "KITTI exporter topic=%s output=%s start_index=%zu",
      topic_.c_str(), output_dir_.c_str(), index_);
  }

 private:
  struct FieldInfo {
    uint32_t offset{0};
    uint8_t datatype{0};
  };

  static std::optional<FieldInfo> field(
      const sensor_msgs::msg::PointCloud2 &msg, const std::string &name) {
    for (const auto &f : msg.fields) {
      if (f.name == name) return FieldInfo{f.offset, f.datatype};
    }
    return std::nullopt;
  }

  static double read_number(
      const uint8_t *ptr, const FieldInfo &f, bool &ok) {
    ok = true;
    switch (f.datatype) {
      case sensor_msgs::msg::PointField::FLOAT32: {
        float v;
        std::memcpy(&v, ptr + f.offset, sizeof(v));
        return v;
      }
      case sensor_msgs::msg::PointField::FLOAT64: {
        double v;
        std::memcpy(&v, ptr + f.offset, sizeof(v));
        return v;
      }
      case sensor_msgs::msg::PointField::UINT8: {
        return *(ptr + f.offset);
      }
      case sensor_msgs::msg::PointField::UINT16: {
        uint16_t v;
        std::memcpy(&v, ptr + f.offset, sizeof(v));
        return v;
      }
      default:
        ok = false;
        return 0.0;
    }
  }

  size_t find_next_index() const {
    size_t max_index = 0;
    bool found = false;
    for (const auto &entry : fs::directory_iterator(scans_dir_)) {
      if (!entry.is_regular_file() || entry.path().extension() != ".bin") continue;
      const auto stem = entry.path().stem().string();
      try {
        const size_t value = static_cast<size_t>(std::stoull(stem));
        max_index = std::max(max_index, value);
        found = true;
      } catch (...) {
      }
    }
    return found ? max_index + 1 : 0;
  }

  void write_metadata(
      const sensor_msgs::msg::PointCloud2 &msg,
      size_t valid_points,
      const std::string &last_file) {
    std::ofstream out(metadata_path_);
    if (!out) return;
    out << "format: kitti_xyzi_float32\n";
    out << "source_topic: " << topic_ << "\n";
    out << "frame_id: " << msg.header.frame_id << "\n";
    out << "exported_scans: " << exported_ << "\n";
    out << "next_index: " << index_ << "\n";
    out << "last_file: " << last_file << "\n";
    out << "last_valid_points: " << valid_points << "\n";
    out << "scan_directory: scans\n";
    out << "point_layout: [x, y, z, intensity]\n";
    out << "point_scalar_type: float32\n";
    out << "bytes_per_point: 16\n";
  }

  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (max_scans_ > 0 && static_cast<int>(exported_) >= max_scans_) return;

    const auto fx = field(*msg, "x");
    const auto fy = field(*msg, "y");
    const auto fz = field(*msg, "z");
    const auto fi = field(*msg, "intensity");
    if (!fx || !fy || !fz) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "PointCloud2 lacks x/y/z fields");
      return;
    }

    std::ostringstream name;
    name << std::setw(6) << std::setfill('0') << index_ << ".bin";
    const fs::path path = scans_dir_ / name.str();
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s", path.c_str());
      return;
    }

    size_t valid = 0;
    const size_t n = static_cast<size_t>(msg->width) * msg->height;
    for (size_t i = 0; i < n; ++i) {
      const uint8_t *ptr = msg->data.data() + i * msg->point_step;
      bool okx=false, oky=false, okz=false, oki=true;
      const double x = read_number(ptr, *fx, okx);
      const double y = read_number(ptr, *fy, oky);
      const double z = read_number(ptr, *fz, okz);
      double intensity = 1.0;
      if (fi) intensity = read_number(ptr, *fi, oki);

      if (!(okx && oky && okz && oki) ||
          !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const float point[4] = {
        static_cast<float>(x), static_cast<float>(y),
        static_cast<float>(z), static_cast<float>(intensity)};
      out.write(reinterpret_cast<const char *>(point), sizeof(point));
      ++valid;
    }

    out.close();
    const rclcpp::Time stamp(msg->header.stamp);
    timestamps_ << std::fixed << std::setprecision(9) << stamp.seconds() << "\n";
    timestamps_.flush();

    ++index_;
    ++exported_;
    write_metadata(*msg, valid, name.str());

    if ((exported_ % 10) == 0 || exported_ == 1) {
      RCLCPP_INFO(
        get_logger(), "exported scan=%zu file=%s points=%zu",
        exported_, name.str().c_str(), valid);
    }
  }

  std::string topic_, output_dir_;
  int max_scans_{0};
  size_t index_{0}, exported_{0};
  fs::path metadata_path_;
  fs::path scans_dir_;
  std::ofstream timestamps_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudToKitti>());
  rclcpp::shutdown();
  return 0;
}
