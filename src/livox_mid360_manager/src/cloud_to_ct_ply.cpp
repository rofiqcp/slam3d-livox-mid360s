#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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

class CloudToCtPly : public rclcpp::Node {
 public:
  CloudToCtPly() : Node("stage2_cloud_to_ct_ply") {
    topic_ = declare_parameter<std::string>("topic", "/livox/lidar");
    output_root_ = declare_parameter<std::string>("output_root", "");
    const int sequence_id = declare_parameter<int>("sequence_id", 0);
    std::ostringstream sequence_name;
    sequence_name << std::setw(2) << std::setfill('0') << sequence_id;
    sequence_ = sequence_name.str();
    max_scans_ = declare_parameter<int>("max_scans", 0);

    if (output_root_.empty()) {
      if (const char *root = std::getenv("LIVOX_MID360_ROOT"); root && *root) {
        output_root_ = std::string(root) + "/datasets/stage2/ct_ply";
      } else {
        output_root_ = "ct_ply";
      }
    }

    frames_dir_ = fs::path(output_root_) / sequence_ / "frames";
    fs::create_directories(frames_dir_);
    index_ = next_index();

    timestamps_.open(
      fs::path(output_root_) / sequence_ / "scan_timestamps.txt",
      std::ios::app);
    if (!timestamps_) throw std::runtime_error("Cannot open scan_timestamps.txt");

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_, rclcpp::SensorDataQoS(),
      std::bind(&CloudToCtPly::on_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "CT-ICP PLY exporter topic=%s output=%s sequence=%s start=%zu",
      topic_.c_str(), output_root_.c_str(), sequence_.c_str(), index_);
  }

 private:
  struct Field {
    uint32_t offset{0};
    uint8_t datatype{0};
  };

  static std::optional<Field> field(
      const sensor_msgs::msg::PointCloud2 &msg, const std::string &name) {
    for (const auto &f : msg.fields) {
      if (f.name == name) return Field{f.offset, f.datatype};
    }
    return std::nullopt;
  }

  static double read_num(const uint8_t *ptr, const Field &f, bool &ok) {
    ok = true;
    switch (f.datatype) {
      case sensor_msgs::msg::PointField::FLOAT32: {
        float v; std::memcpy(&v, ptr + f.offset, sizeof(v)); return v;
      }
      case sensor_msgs::msg::PointField::FLOAT64: {
        double v; std::memcpy(&v, ptr + f.offset, sizeof(v)); return v;
      }
      case sensor_msgs::msg::PointField::UINT32: {
        uint32_t v; std::memcpy(&v, ptr + f.offset, sizeof(v)); return v;
      }
      default:
        ok = false;
        return 0.0;
    }
  }

  static double to_seconds(double t) {
    const double a = std::abs(t);
    if (a > 1.0e15) return t * 1.0e-9;  // ns epoch / Livox SDK timestamps
    if (a > 1.0e12) return t * 1.0e-6;  // us epoch
    if (a > 1.0e10) return t * 1.0e-3;  // ms epoch
    return t;                            // already seconds
  }

  size_t next_index() const {
    size_t max_idx = 0;
    bool any = false;
    for (const auto &e : fs::directory_iterator(frames_dir_)) {
      if (!e.is_regular_file()) continue;
      const auto n = e.path().filename().string();
      if (n.rfind("frame_", 0) != 0 || e.path().extension() != ".ply") continue;
      try {
        const auto stem = e.path().stem().string();
        const size_t idx = std::stoul(stem.substr(6));
        max_idx = any ? std::max(max_idx, idx) : idx;
        any = true;
      } catch (...) {}
    }
    return any ? max_idx + 1 : 0;
  }

  void write_metadata(
      const sensor_msgs::msg::PointCloud2 &msg, size_t count,
      bool source_has_timestamp) {
    std::ofstream out(fs::path(output_root_) / "metadata.yaml");
    out << "format: ct_icp_ply_xyz_timestamp\n";
    out << "source_topic: " << topic_ << "\n";
    out << "frame_id: " << msg.header.frame_id << "\n";
    out << "sequence: " << sequence_ << "\n";
    out << "exported_scans: " << exported_ << "\n";
    out << "last_valid_points: " << count << "\n";
    out << "source_has_per_point_timestamp: "
        << (source_has_timestamp ? "true" : "false") << "\n";
    out << "timestamp_unit_written: seconds\n";
    out << "ply_properties: [x, y, z, timestamp]\n";
  }

  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (max_scans_ > 0 && static_cast<int>(exported_) >= max_scans_) return;

    const auto fx = field(*msg, "x");
    const auto fy = field(*msg, "y");
    const auto fz = field(*msg, "z");
    auto ft = field(*msg, "timestamp");
    if (!ft) ft = field(*msg, "time");
    if (!ft) ft = field(*msg, "t");

    if (!fx || !fy || !fz) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "PointCloud2 lacks x/y/z");
      return;
    }

    struct P {
      float x, y, z;
      double t;
    };
    std::vector<P> points;
    const size_t n = static_cast<size_t>(msg->width) * msg->height;
    points.reserve(n);

    const double scan_stamp = rclcpp::Time(msg->header.stamp).seconds();
    for (size_t i = 0; i < n; ++i) {
      const uint8_t *ptr = msg->data.data() + i * msg->point_step;
      bool okx=false, oky=false, okz=false, okt=true;
      const double x = read_num(ptr, *fx, okx);
      const double y = read_num(ptr, *fy, oky);
      const double z = read_num(ptr, *fz, okz);
      double t = scan_stamp;
      if (ft) t = to_seconds(read_num(ptr, *ft, okt));
      if (okt) {
        if (!time_origin_) time_origin_ = t;
        t -= *time_origin_;
      }

      if (!(okx && oky && okz && okt) ||
          !std::isfinite(x) || !std::isfinite(y) ||
          !std::isfinite(z) || !std::isfinite(t)) {
        continue;
      }
      points.push_back({
        static_cast<float>(x), static_cast<float>(y),
        static_cast<float>(z), t});
    }

    std::ostringstream name;
    name << "frame_" << std::setw(4) << std::setfill('0') << index_ << ".ply";
    const fs::path path = frames_dir_ / name.str();

    // ASCII PLY is intentionally used for maximum compatibility with the
    // reference CT-ICP/tinyply loader and transparent inspection.
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out) {
      RCLCPP_ERROR(get_logger(), "Cannot write %s", path.c_str());
      return;
    }
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "comment generated by livox_mid360_manager cloud_to_ct_ply\n";
    out << "element vertex " << points.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property double timestamp\n";
    out << "end_header\n";
    out << std::fixed << std::setprecision(9);
    for (const auto &p : points) {
      out << p.x << " " << p.y << " " << p.z << " " << p.t << "\n";
    }
    out.close();

    timestamps_ << std::fixed << std::setprecision(9) << scan_stamp << "\n";
    timestamps_.flush();

    ++index_;
    ++exported_;
    write_metadata(*msg, points.size(), ft.has_value());

    if (exported_ == 1 || exported_ % 10 == 0) {
      RCLCPP_INFO(
        get_logger(), "CT PLY exported=%zu file=%s points=%zu timestamp=%s",
        exported_, name.str().c_str(), points.size(),
        ft ? "per-point" : "scan-stamp fallback");
    }
  }

  std::string topic_, output_root_, sequence_;
  int max_scans_{0};
  size_t index_{0}, exported_{0};
  fs::path frames_dir_;
  std::optional<double> time_origin_;
  std::ofstream timestamps_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudToCtPly>());
  rclcpp::shutdown();
  return 0;
}
