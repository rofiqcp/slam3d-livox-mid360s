#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

class Stage2OdomEvaluator : public rclcpp::Node {
 public:
  Stage2OdomEvaluator() : Node("stage2_odom_evaluator") {
    backend_ = declare_parameter<std::string>("backend", "kiss_icp");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/kiss/odometry");
    path_topic_ = declare_parameter<std::string>("path_topic", "/stage2/kiss_path");
    ground_truth_topic_ = declare_parameter<std::string>("ground_truth_topic", "");
    expected_hz_ = declare_parameter<double>("expected_hz", 10.0);
    startup_grace_s_ = declare_parameter<double>("startup_grace", 5.0);
    stale_timeout_s_ = declare_parameter<double>("stale_timeout", 2.0);
    translation_jump_m_ = declare_parameter<double>("translation_jump_m", 1.0);
    rotation_jump_rad_ = declare_parameter<double>("rotation_jump_rad", 0.5);
    z_span_warn_m_ = declare_parameter<double>("z_span_warn_m", 0.5);
    max_path_poses_ = declare_parameter<int>("max_path_poses", 20000);

    std::string default_csv = "stage2_odometry.csv";
    if (const char *root = std::getenv("SLAM3D_DATA"); root && *root) {
      default_csv = std::string(root) + "/results/stage2/" + backend_ + "_odometry.csv";
    }
    csv_path_ = declare_parameter<std::string>("csv_path", default_csv);
    std::string default_summary = fs::path(csv_path_).replace_extension(".summary.yaml").string();
    summary_path_ = declare_parameter<std::string>("summary_path", default_summary);

    const auto parent = fs::path(csv_path_).parent_path();
    if (!parent.empty()) fs::create_directories(parent);
    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_) throw std::runtime_error("Cannot open CSV: " + csv_path_);
    csv_ << "stamp_sec,x,y,z,qx,qy,qz,qw,step_m,step_angle_rad,path_length_m,"
            "displacement_m,z_span_m,odom_hz,jump_count,gt_error_m\n";

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(50),
      std::bind(&Stage2OdomEvaluator::on_odom, this, std::placeholders::_1));

    if (!ground_truth_topic_.empty()) {
      gt_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        ground_truth_topic_, rclcpp::QoS(50),
        std::bind(&Stage2OdomEvaluator::on_gt, this, std::placeholders::_1));
    }

    start_steady_ = std::chrono::steady_clock::now();
    report_steady_ = start_steady_;
    timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&Stage2OdomEvaluator::report, this));

    RCLCPP_INFO(
      get_logger(), "Stage2 evaluator backend=%s odom=%s csv=%s",
      backend_.c_str(), odom_topic_.c_str(), csv_path_.c_str());
  }

  ~Stage2OdomEvaluator() override {
    if (csv_) csv_.flush();
    write_summary();
  }

 private:
  struct Pose {
    double x{0}, y{0}, z{0};
    double qx{0}, qy{0}, qz{0}, qw{1};
  };

  static Pose pose_from(const geometry_msgs::msg::Pose &p) {
    return {p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
  }

  static double translation_distance(const Pose &a, const Pose &b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  static std::array<double, 3> relative_translation(
      const Pose &pose, const Pose &origin) {
    // Translation of origin^-1 * pose. Rotate the translation delta by
    // the inverse of the origin quaternion.
    const double vx = pose.x - origin.x;
    const double vy = pose.y - origin.y;
    const double vz = pose.z - origin.z;

    const double ux = -origin.qx;
    const double uy = -origin.qy;
    const double uz = -origin.qz;
    const double qw = origin.qw;

    const double udotv = ux * vx + uy * vy + uz * vz;
    const double udotu = ux * ux + uy * uy + uz * uz;
    const double cx = uy * vz - uz * vy;
    const double cy = uz * vx - ux * vz;
    const double cz = ux * vy - uy * vx;

    return {
      2.0 * udotv * ux + (qw * qw - udotu) * vx + 2.0 * qw * cx,
      2.0 * udotv * uy + (qw * qw - udotu) * vy + 2.0 * qw * cy,
      2.0 * udotv * uz + (qw * qw - udotu) * vz + 2.0 * qw * cz};
  }

  static double rotation_distance(const Pose &a, const Pose &b) {
    double dot = a.qx*b.qx + a.qy*b.qy + a.qz*b.qz + a.qw*b.qw;
    dot = std::clamp(std::abs(dot), 0.0, 1.0);
    return 2.0 * std::acos(dot);
  }

  static diagnostic_msgs::msg::KeyValue kv(const std::string &k, const std::string &v) {
    diagnostic_msgs::msg::KeyValue x;
    x.key = k;
    x.value = v;
    return x;
  }

  static std::string f4(double v) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(4) << v;
    return os.str();
  }

  struct StampedPose {
    rclcpp::Time stamp;
    Pose pose;
  };

  double evaluate_gt_pair(
      const rclcpp::Time &odom_stamp, const Pose &odom_pose, const Pose &gt_pose) {
    const int64_t ns = odom_stamp.nanoseconds();
    if (ns <= last_paired_odom_stamp_ns_) return -1.0;
    last_paired_odom_stamp_ns_ = ns;

    if (!have_gt_eval_origin_) {
      odom_eval_origin_ = odom_pose;
      gt_eval_origin_ = gt_pose;
      have_gt_eval_origin_ = true;
      ++gt_error_count_;
      return 0.0;
    }

    const auto odom_rel = relative_translation(odom_pose, odom_eval_origin_);
    const auto gt_rel = relative_translation(gt_pose, gt_eval_origin_);
    const double dx = odom_rel[0] - gt_rel[0];
    const double dy = odom_rel[1] - gt_rel[1];
    const double dz = odom_rel[2] - gt_rel[2];
    const double error = std::sqrt(dx * dx + dy * dy + dz * dz);
    gt_error_sq_sum_ += error * error;
    ++gt_error_count_;
    return error;
  }

  double match_gt_for_odom(const rclcpp::Time &stamp, const Pose &odom_pose) {
    constexpr double kMaxPairDeltaS = 0.02;
    double best_dt = kMaxPairDeltaS + 1.0;
    const StampedPose *best = nullptr;
    for (auto it = gt_buffer_.rbegin(); it != gt_buffer_.rend(); ++it) {
      const double dt = std::abs((stamp - it->stamp).seconds());
      if (dt < best_dt) {
        best_dt = dt;
        best = &(*it);
      }
      if (it->stamp < stamp && dt > kMaxPairDeltaS) break;
    }
    if (best && best_dt <= kMaxPairDeltaS) {
      return evaluate_gt_pair(stamp, odom_pose, best->pose);
    }
    return -1.0;
  }

  void on_gt(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    const StampedPose gt{rclcpp::Time(msg->header.stamp), pose_from(msg->pose.pose)};
    gt_buffer_.push_back(gt);
    while (gt_buffer_.size() > 500) gt_buffer_.pop_front();

    constexpr double kMaxPairDeltaS = 0.02;
    double best_dt = kMaxPairDeltaS + 1.0;
    const StampedPose *best = nullptr;
    for (auto it = odom_buffer_.rbegin(); it != odom_buffer_.rend(); ++it) {
      const double dt = std::abs((gt.stamp - it->stamp).seconds());
      if (dt < best_dt) {
        best_dt = dt;
        best = &(*it);
      }
      if (it->stamp < gt.stamp && dt > kMaxPairDeltaS) break;
    }
    if (best && best_dt <= kMaxPairDeltaS) {
      evaluate_gt_pair(best->stamp, best->pose, gt.pose);
    }
  }

  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    const auto steady_now = std::chrono::steady_clock::now();
    const Pose current = pose_from(msg->pose.pose);
    const rclcpp::Time stamp(msg->header.stamp);

    double step = 0.0;
    double angle = 0.0;
    if (have_odom_) {
      step = translation_distance(current, previous_);
      angle = rotation_distance(current, previous_);
      path_length_m_ += step;
      max_step_m_ = std::max(max_step_m_, step);
      max_step_angle_rad_ = std::max(max_step_angle_rad_, angle);
      if (step > translation_jump_m_ || angle > rotation_jump_rad_) ++jump_count_;
      if (stamp <= last_stamp_) ++nonmonotonic_stamp_count_;

      const double dt = std::chrono::duration<double>(steady_now - last_arrival_).count();
      if (dt > 0.0) {
        arrival_dt_sum_ += dt;
        arrival_dt_sq_sum_ += dt * dt;
        ++arrival_dt_count_;
      }
    } else {
      origin_ = current;
      z_min_ = z_max_ = current.z;
      path_.header.frame_id = msg->header.frame_id;
      have_odom_ = true;
    }

    z_min_ = std::min(z_min_, current.z);
    z_max_ = std::max(z_max_, current.z);
    previous_ = current;
    last_stamp_ = stamp;
    last_arrival_ = steady_now;
    ++window_count_;
    ++total_count_;

    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg->header;
    ps.pose = msg->pose.pose;
    path_.header = msg->header;
    path_.poses.push_back(ps);
    if (max_path_poses_ > 0 && static_cast<int>(path_.poses.size()) > max_path_poses_) {
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + path_.poses.size() / 4);
    }
    if (path_pub_->get_subscription_count()>0 &&
        std::chrono::steady_clock::now()-last_path_wall_>=std::chrono::seconds(1)) {
      path_pub_->publish(path_);last_path_wall_=std::chrono::steady_clock::now();
    }

    const double displacement = translation_distance(current, origin_);
    const double z_span = z_max_ - z_min_;

    odom_buffer_.push_back(StampedPose{stamp, current});
    while (odom_buffer_.size() > 500) odom_buffer_.pop_front();
    const double gt_error = match_gt_for_odom(stamp, current);

    const double stamp_sec = stamp.seconds();
    csv_ << std::setprecision(12) << stamp_sec << ","
         << current.x << "," << current.y << "," << current.z << ","
         << current.qx << "," << current.qy << "," << current.qz << "," << current.qw << ","
         << step << "," << angle << "," << path_length_m_ << "," << displacement << ","
         << z_span << "," << last_rate_hz_ << "," << jump_count_ << "," << gt_error << "\n";
    if ((total_count_ % 50) == 0) csv_.flush();
  }

  void write_summary() {
    if (!have_odom_) return;
    const auto parent = fs::path(summary_path_).parent_path();
    if (!parent.empty()) fs::create_directories(parent);

    std::ofstream out(summary_path_);
    if (!out) return;

    const double z_span = z_max_ - z_min_;
    const double displacement = translation_distance(previous_, origin_);
    out << std::setprecision(10);
    out << "backend: " << backend_ << std::endl;
    out << "odom_topic: " << odom_topic_ << std::endl;
    out << "poses: " << total_count_ << std::endl;
    out << "path_length_m: " << path_length_m_ << std::endl;
    out << "displacement_m: " << displacement << std::endl;
    out << "z_span_m: " << z_span << std::endl;
    out << "max_step_m: " << max_step_m_ << std::endl;
    out << "max_step_angle_rad: " << max_step_angle_rad_ << std::endl;
    out << "jump_count: " << jump_count_ << std::endl;
    out << "nonmonotonic_stamp_count: " << nonmonotonic_stamp_count_ << std::endl;
    if (gt_error_count_ > 0) {
      out << "gt_pairs: " << gt_error_count_ << std::endl;
      out << "translation_rmse_vs_gt_m: "
          << std::sqrt(gt_error_sq_sum_ / static_cast<double>(gt_error_count_))
          << std::endl;
    }
  }

  void report() {
    const auto now_steady = std::chrono::steady_clock::now();
    const double dt = std::max(
      1e-6, std::chrono::duration<double>(now_steady - report_steady_).count());
    last_rate_hz_ = static_cast<double>(window_count_) / dt;
    window_count_ = 0;
    report_steady_ = now_steady;

    const double uptime =
      std::chrono::duration<double>(now_steady - start_steady_).count();
    const double age = have_odom_
      ? std::chrono::duration<double>(now_steady - last_arrival_).count() : -1.0;
    const double z_span = have_odom_ ? z_max_ - z_min_ : 0.0;

    double jitter = 0.0;
    if (arrival_dt_count_ > 1) {
      const double mean = arrival_dt_sum_ / static_cast<double>(arrival_dt_count_);
      const double var = std::max(
        0.0, arrival_dt_sq_sum_ / static_cast<double>(arrival_dt_count_) - mean * mean);
      jitter = std::sqrt(var);
    }

    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "Stage2/" + backend_ + "/Odometry";
    st.hardware_id = "livox_mid360s";
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "odometry healthy";

    if (!have_odom_ && uptime > startup_grace_s_) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      st.message = "no odometry";
    } else if (have_odom_ && age > stale_timeout_s_) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      st.message = "stale odometry";
    } else if (jump_count_ > 0 || z_span > z_span_warn_m_) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      st.message = "trajectory quality warning";
    }

    st.values.push_back(kv("odom_topic", odom_topic_));
    st.values.push_back(kv("rate_hz", f4(last_rate_hz_)));
    st.values.push_back(kv("arrival_jitter_s", f4(jitter)));
    st.values.push_back(kv("age_s", f4(age)));
    st.values.push_back(kv("total_poses", std::to_string(total_count_)));
    st.values.push_back(kv("path_length_m", f4(path_length_m_)));
    st.values.push_back(kv("z_span_m", f4(z_span)));
    st.values.push_back(kv("max_step_m", f4(max_step_m_)));
    st.values.push_back(kv("max_step_angle_rad", f4(max_step_angle_rad_)));
    st.values.push_back(kv("jump_count", std::to_string(jump_count_)));
    st.values.push_back(kv("nonmonotonic_stamp_count", std::to_string(nonmonotonic_stamp_count_)));
    if (gt_error_count_ > 0) {
      st.values.push_back(kv(
        "translation_rmse_vs_gt_m",
        f4(std::sqrt(gt_error_sq_sum_ / static_cast<double>(gt_error_count_)))));
    }

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    arr.status.push_back(st);
    diag_pub_->publish(arr);

    RCLCPP_INFO(
      get_logger(),
      "%s odom=%.2fHz poses=%lu path=%.2fm z_span=%.3fm jumps=%lu%s",
      backend_.c_str(), last_rate_hz_, static_cast<unsigned long>(total_count_),
      path_length_m_, z_span, static_cast<unsigned long>(jump_count_),
      gt_error_count_ > 0 ? " GT=active" : "");
  }

  std::string backend_, odom_topic_, path_topic_, ground_truth_topic_, csv_path_, summary_path_;
  double expected_hz_{10.0}, startup_grace_s_{5.0}, stale_timeout_s_{2.0};
  double translation_jump_m_{1.0}, rotation_jump_rad_{0.5}, z_span_warn_m_{0.5};
  std::chrono::steady_clock::time_point last_path_wall_{};
  int max_path_poses_{20000};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_, gt_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path path_;
  std::ofstream csv_;
  bool have_odom_{false}, have_gt_eval_origin_{false};
  Pose origin_, previous_, odom_eval_origin_, gt_eval_origin_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  std::deque<StampedPose> gt_buffer_, odom_buffer_;
  int64_t last_paired_odom_stamp_ns_{-1};
  std::chrono::steady_clock::time_point start_steady_, report_steady_, last_arrival_;

  uint64_t window_count_{0}, total_count_{0}, jump_count_{0}, nonmonotonic_stamp_count_{0};
  uint64_t arrival_dt_count_{0}, gt_error_count_{0};
  double path_length_m_{0.0}, z_min_{0.0}, z_max_{0.0};
  double max_step_m_{0.0}, max_step_angle_rad_{0.0}, last_rate_hz_{0.0};
  double arrival_dt_sum_{0.0}, arrival_dt_sq_sum_{0.0}, gt_error_sq_sum_{0.0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Stage2OdomEvaluator>());
  rclcpp::shutdown();
  return 0;
}
