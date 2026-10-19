#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tbb/global_control.h>
#include <pcl_conversions/pcl_conversions.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
constexpr double kPi = 3.14159265358979323846;
constexpr size_t kRings = 20;
constexpr size_t kSectors = 60;

static gtsam::Pose3 toPose(const geometry_msgs::msg::Pose &p) {
  const auto &q = p.orientation;
  const Eigen::Quaterniond e(q.w, q.x, q.y, q.z);
  if (e.norm() < 0.5 || !std::isfinite(e.norm())) {
    throw std::runtime_error("Invalid odometry quaternion");
  }
  return gtsam::Pose3(gtsam::Rot3(e.normalized().toRotationMatrix()),
                      gtsam::Point3(p.position.x, p.position.y, p.position.z));
}
static geometry_msgs::msg::Pose toRos(const gtsam::Pose3 &pose) {
  geometry_msgs::msg::Pose out;
  const auto p = pose.translation();
  const Eigen::Quaterniond q(pose.rotation().matrix());
  out.position.x = p.x(); out.position.y = p.y(); out.position.z = p.z();
  out.orientation.x = q.x(); out.orientation.y = q.y();
  out.orientation.z = q.z(); out.orientation.w = q.w();
  return out;
}
static Eigen::Matrix4f toMatrix(const gtsam::Pose3 &p) {
  return p.matrix().cast<float>();
}
static Cloud::Ptr downsample(const Cloud::Ptr &in, float leaf) {
  auto out = Cloud::Ptr(new Cloud());
  if (in->empty()) return out;
  pcl::VoxelGrid<Point> voxel;
  voxel.setInputCloud(in);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*out);
  return out;
}
struct Keyframe {
  double stamp{0.0};
  gtsam::Pose3 raw;
  Cloud::Ptr local;
  std::vector<float> descriptor;
};
static std::vector<float> describe(const Cloud &cloud, float max_r) {
  std::vector<float> cells(kRings*kSectors, -1000.0f);
  for (const auto &p : cloud.points) {
    const double r = std::hypot(p.x, p.y);
    if (!std::isfinite(r) || r < 0.1 || r >= max_r) continue;
    double a = std::atan2(p.y, p.x) + kPi;
    size_t ring = std::min(kRings-1, static_cast<size_t>(r/max_r*kRings));
    size_t sector = std::min(kSectors-1, static_cast<size_t>(a/(2*kPi)*kSectors));
    float &v = cells[ring*kSectors+sector];
    v = std::max(v, p.z);
  }
  for (auto &v : cells) if (v == -1000.0f) v = 0.0f;
  return cells;
}
static double descriptorDistance(const std::vector<float> &a,
                                 const std::vector<float> &b) {
  // Scan Context-style circular sector alignment; return 1 - best cosine.
  double best = 1.0;
  for (size_t shift = 0; shift < kSectors; shift += 2) {
    double dot=0, aa=0, bb=0;
    for (size_t r=0; r<kRings; ++r) {
      for (size_t s=0; s<kSectors; ++s) {
        const double x=a[r*kSectors+s];
        const double y=b[r*kSectors+(s+shift)%kSectors];
        dot += x*y; aa += x*x; bb += y*y;
      }
    }
    if (aa>1e-8 && bb>1e-8) {
      const double d = 1.0 - std::clamp(dot/std::sqrt(aa*bb), -1.0, 1.0);
      best=std::min(best,d);
    }
  }
  return best;
}

class PoseGraphNode : public rclcpp::Node {
 public:
  PoseGraphNode() : Node("pose_graph_node") {
    const int tbb_threads=declare_parameter<int>("tbb_max_parallelism",4);
    if(tbb_threads<1 || tbb_threads>32)
      throw std::runtime_error("P4 tbb_max_parallelism must be 1..32");
    tbb_control_=std::make_unique<tbb::global_control>(
      tbb::global_control::max_allowed_parallelism,static_cast<size_t>(tbb_threads));
    odom_topic_=declare_parameter<std::string>("odom_topic","/lio/odometry");
    cloud_topic_=declare_parameter<std::string>("cloud_topic","/lio/cloud_registered");
    input_frame_=declare_parameter<std::string>("input_frame","lio_odom");
    map_frame_=declare_parameter<std::string>("map_frame","map");
    child_frame_=declare_parameter<std::string>("child_frame","livox_frame");
    output_dir_=declare_parameter<std::string>("map_output_dir","");
    publish_tf_=declare_parameter<bool>("publish_tf",true);
    enable_loop_=declare_parameter<bool>("enable_loop_closure",true);
    distance_=declare_parameter<double>("keyframe_distance_m",1.0);
    angle_=declare_parameter<double>("keyframe_rotation_deg",12.0)*kPi/180.0;
    sync_tolerance_=declare_parameter<double>("sync_tolerance_s",0.03);
    min_loop_seconds_=declare_parameter<double>("min_loop_separation_s",15.0);
    min_loop_frames_=declare_parameter<int>("min_loop_keyframes",25);
    loop_accept_gap_=declare_parameter<int>("min_keyframes_between_accepted_loops",6);
    candidate_radius_=declare_parameter<double>("candidate_radius_m",8.0);
    descriptor_limit_=declare_parameter<double>("descriptor_max_distance",0.29);
    descriptor_radius_=declare_parameter<double>("scan_context_max_radius_m",50.0);
    icp_correspondence_=declare_parameter<double>("icp_max_correspondence_m",2.0);
    icp_iterations_=declare_parameter<int>("icp_max_iterations",45);
    icp_fitness_=declare_parameter<double>("icp_fitness_threshold",0.15);
    // Geometric verification after ICP: squared nearest-neighbor distance and
    // overlap ratio mitigate visually similar but nonoverlapping place matches.
    loop_min_overlap_=declare_parameter<double>("loop_min_overlap_ratio",0.3);
    loop_overlap_distance_=declare_parameter<double>("loop_overlap_distance_m",0.65);
    loop_overlap_sample_limit_=declare_parameter<int>("loop_overlap_max_samples",1000);
    max_loop_correction_=declare_parameter<double>("max_loop_correction_m",2.0);
    max_loop_rotation_=declare_parameter<double>("max_loop_rotation_deg",35.0)*kPi/180.0;
    scan_voxel_=declare_parameter<double>("scan_voxel_m",0.5);
    map_voxel_=declare_parameter<double>("map_voxel_m",0.3);
    map_period_=declare_parameter<double>("map_publish_period_s",5.0);
    max_keyframes_=declare_parameter<int>("max_keyframes",1500);
    max_map_points_=declare_parameter<int>("max_map_points",250000);

    if (distance_<=0 || scan_voxel_<=0 || map_voxel_<=0 ||
        map_period_<=0 || max_keyframes_<5 || max_map_points_<1000 ||
        loop_min_overlap_<0 || loop_min_overlap_>1 ||
        loop_overlap_distance_<=0 || loop_overlap_sample_limit_<20) {
      throw std::runtime_error("Invalid P4 configuration: positive bounds required");
    }
    gtsam::ISAM2Params params;
    params.relinearizeThreshold=0.01;
    params.relinearizeSkip=1;
    isam_=std::make_unique<gtsam::ISAM2>(params);
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    corrected_pub_=create_publisher<nav_msgs::msg::Odometry>("/p4/optimized_odometry",20);
    path_pub_=create_publisher<nav_msgs::msg::Path>("/p4/optimized_path",rclcpp::QoS(1).transient_local());
    map_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/p4/optimized_map",rclcpp::QoS(1).transient_local());
    diag_pub_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",10);
    reset_sub_=create_subscription<std_msgs::msg::Empty>("/lio/reset",10,
      [this](std_msgs::msg::Empty::ConstSharedPtr){resetGraph();});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        if (m->header.frame_id != input_frame_) {
          if (++bad_frames_ <= 3) RCLCPP_ERROR(get_logger(),"Wrong odom frame %s expected %s",
            m->header.frame_id.c_str(),input_frame_.c_str());
          return;
        }
        odoms_.push_back(m);
        if (odoms_.size()>150) odoms_.pop_front();
        synchronize();
        publishCorrected(*m);
      });
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
        if (m->header.frame_id != input_frame_) {
          if (++bad_frames_<=3) RCLCPP_ERROR(get_logger(),"Wrong cloud frame %s expected %s",
            m->header.frame_id.c_str(),input_frame_.c_str());
          return;
        }
        clouds_.push_back(m);
        if (clouds_.size()>20) { clouds_.pop_front(); ++dropped_clouds_; }
        synchronize();
      });
    service_=create_service<std_srvs::srv::Trigger>("/p4/save_map",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        try {
          if (keys_.empty()) throw std::runtime_error("No keyframes, cannot save map");
          publishMap();
          if (!output_dir_.empty()) {
            fs::create_directories(output_dir_);
            const auto tag=std::to_string(
              std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
            const auto pcd=fs::path(output_dir_)/("stage4_map_"+tag+".pcd");
            const auto csv=fs::path(output_dir_)/("stage4_poses_"+tag+".csv");
            if (pcl::io::savePCDFileBinary(pcd.string(),*cached_map_)<0)
              throw std::runtime_error("PCD write failed");
            std::ofstream out(csv);
            if (!out) throw std::runtime_error("CSV open failed");
            out<<"stamp,x,y,z,qx,qy,qz,qw\n"<<std::setprecision(12);
            for (size_t i=0;i<keys_.size();++i) {
              const auto p=toRos(optimized_.at(i));
              out<<keys_[i].stamp<<","<<p.position.x<<","<<p.position.y<<","<<p.position.z
                 <<","<<p.orientation.x<<","<<p.orientation.y<<","<<p.orientation.z
                 <<","<<p.orientation.w<<"\n";
            }
            response->message=pcd.string();
          } else {
            throw std::runtime_error("Set map_output_dir before saving");
          }
          response->success=true;
        } catch(const std::exception &e) {
          response->success=false;
          response->message=e.what();
        }
      });
    timer_=create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(map_period_)),[this](){
      if (!keys_.empty() && map_dirty_)
        publishMap();
      publishDiagnostics();
    });
    RCLCPP_INFO(get_logger(),"P4 independent pose graph started; odom=%s cloud=%s loops=%s",
      odom_topic_.c_str(), cloud_topic_.c_str(),enable_loop_?"on":"off");
  }

 private:
  static gtsam::Key key(size_t idx) { return gtsam::Symbol('x',idx); }
  void resetGraph() {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold=0.01;params.relinearizeSkip=1;
    isam_=std::make_unique<gtsam::ISAM2>(params);
    keys_.clear();optimized_.clear();odoms_.clear();clouds_.clear();cached_map_->clear();
    correction_=gtsam::Pose3{};warned_full_=false;map_dirty_=false;
    has_accepted_loop_=false;last_accepted_loop_keyframe_=0;
    loops_accepted_=0;loops_rejected_=0;last_optimized_loop_count_=0;
    map_input_points_=0;map_sampled_points_=0;
    publishMap();publishPath();publishDiagnostics();
    RCLCPP_WARN(get_logger(),"P4 reset after FAST-LIO input discontinuity");
  }
  void synchronize() {
    while (!clouds_.empty() && !odoms_.empty()) {
      auto cloud=clouds_.front();
      const double ts=rclcpp::Time(cloud->header.stamp).seconds();
      size_t best=0; double error=std::numeric_limits<double>::infinity();
      for (size_t i=0;i<odoms_.size();++i) {
        const double dt=std::abs(rclcpp::Time(odoms_[i]->header.stamp).seconds()-ts);
        if (dt<error) { error=dt; best=i; }
      }
      if (error<=sync_tolerance_) {
        auto odom=odoms_[best];
        clouds_.pop_front();
        odoms_.erase(odoms_.begin(),odoms_.begin()+best+1);
        try { processPair(*odom,*cloud,ts); }
        catch(const std::exception &e) {
          RCLCPP_ERROR(get_logger(),"P4 keyframe rejected: %s",e.what());
          ++bad_clouds_;
        }
      } else {
        const double newest=rclcpp::Time(odoms_.back()->header.stamp).seconds();
        if (newest>ts+sync_tolerance_) {
          clouds_.pop_front(); ++dropped_clouds_;
        } else break;  // odometry for this cloud may arrive later
      }
    }
  }
  void processPair(const nav_msgs::msg::Odometry &odom,
                   const sensor_msgs::msg::PointCloud2 &msg,double stamp) {
    if (msg.width*msg.height<30) { ++bad_clouds_; return; }
    const gtsam::Pose3 raw=toPose(odom.pose.pose);
    if (!keys_.empty()) {
      const auto delta=keys_.back().raw.between(raw);
      if (delta.translation().norm()<distance_ &&
          gtsam::Rot3::Logmap(delta.rotation()).norm()<angle_) return;
    }
    if (keys_.size()>=static_cast<size_t>(max_keyframes_)) {
      if (!warned_full_) { RCLCPP_ERROR(get_logger(),"Reached max_keyframes=%d, refusing new keyframes",max_keyframes_); warned_full_=true; }
      return;
    }
    Cloud::Ptr world(new Cloud);
    pcl::fromROSMsg(msg,*world);
    world->points.erase(std::remove_if(world->points.begin(),world->points.end(),
      [](const Point &p){return !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z);}),
      world->points.end());
    world->width=world->points.size(); world->height=1; world->is_dense=true;
    if (world->size()<30) {++bad_clouds_;return;}
    Cloud::Ptr local(new Cloud);
    pcl::transformPointCloud(*world,*local,toMatrix(raw.inverse()));
    local=downsample(local,static_cast<float>(scan_voxel_));
    if (local->size()<30) {++bad_clouds_;return;}

    Keyframe frame;
    frame.stamp=stamp; frame.raw=raw; frame.local=local;
    frame.descriptor=describe(*local,static_cast<float>(descriptor_radius_));
    const size_t index=keys_.size();
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    gtsam::Vector sigmas(6);
    if (index==0) {
      sigmas<<0.001,0.001,0.001,0.002,0.002,0.002;
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(key(0),raw,gtsam::noiseModel::Diagonal::Sigmas(sigmas)));
    } else {
      sigmas<<0.02,0.02,0.02,0.05,0.05,0.05;
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key(index-1),key(index),keys_.back().raw.between(raw),
        gtsam::noiseModel::Diagonal::Sigmas(sigmas)));
    }
    const auto proposal=detectLoop(frame,index);
    if (proposal.first>=0) {
      const size_t candidate=static_cast<size_t>(proposal.first);
      const gtsam::Pose3 guess=keys_[candidate].raw.between(raw);
      pcl::IterativeClosestPoint<Point,Point> icp;
      icp.setInputSource(local);
      icp.setInputTarget(keys_[candidate].local);
      icp.setMaxCorrespondenceDistance(icp_correspondence_);
      icp.setMaximumIterations(icp_iterations_);
      icp.setTransformationEpsilon(1e-7);
      Cloud aligned;
      icp.align(aligned,toMatrix(guess));
      ++icp_checked_;
      const auto overlap_begin=std::chrono::steady_clock::now();
      double overlap=0.0;
      if(icp.hasConverged() && std::isfinite(icp.getFitnessScore()) &&
         icp.getFitnessScore()<icp_fitness_ && !aligned.empty()){
        pcl::KdTreeFLANN<Point> kd;
        kd.setInputCloud(keys_[candidate].local);
        size_t visited=0,hits=0;
        const size_t stride=std::max<size_t>(1,
          (aligned.size()+static_cast<size_t>(loop_overlap_sample_limit_)-1)/
          static_cast<size_t>(loop_overlap_sample_limit_));
        std::vector<int> indices(1);
        std::vector<float> sqdist(1);
        const float threshold_sq=static_cast<float>(loop_overlap_distance_*loop_overlap_distance_);
        for(size_t i=0;i<aligned.size();i+=stride){
          const auto &pt=aligned[i];
          if(!std::isfinite(pt.x)||!std::isfinite(pt.y)||!std::isfinite(pt.z))continue;
          ++visited;
          if(kd.nearestKSearch(pt,1,indices,sqdist)>0&&sqdist[0]<=threshold_sq)
            ++hits;
        }
        if(visited>0)overlap=static_cast<double>(hits)/visited;
      }
      last_loop_overlap_=overlap;
      overlap_eval_ms_=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-overlap_begin).count();
      if (icp.hasConverged() && std::isfinite(icp.getFitnessScore()) &&
          icp.getFitnessScore()<icp_fitness_ && overlap>=loop_min_overlap_) {
        const gtsam::Matrix4 matrix=icp.getFinalTransformation().cast<double>();
        const gtsam::Pose3 measured(matrix);
        const gtsam::Pose3 error=guess.between(measured);
        if (error.translation().norm()<=max_loop_correction_ &&
            gtsam::Rot3::Logmap(error.rotation()).norm()<=max_loop_rotation_) {
          gtsam::Vector loop_noise(6);
          loop_noise<<0.035,0.035,0.035,0.12,0.12,0.12;
          auto robust=gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345),
            gtsam::noiseModel::Diagonal::Sigmas(loop_noise));
          graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            key(candidate),key(index),measured,robust));
          ++loops_accepted_;
          has_accepted_loop_=true;
          last_accepted_loop_keyframe_=index;
          RCLCPP_WARN(get_logger(),"LOOP VERIFIED %zu -> %zu score=%.3f ICP=%.4f",
            candidate,index,proposal.second,icp.getFitnessScore());
        } else ++loops_rejected_;
      } else {
        ++loops_rejected_;
        if(overlap<loop_min_overlap_) {
          ++loops_rejected_overlap_;
          RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
            "P4 ICP candidate rejected: overlap %.3f < required %.3f (fitness %.4f)",
            overlap,loop_min_overlap_,icp.getFitnessScore());
        }
      }
    }

    initial.insert(key(index),raw);
    isam_->update(graph,initial);
    if (proposal.first>=0 && loops_accepted_>last_optimized_loop_count_) {
      isam_->update();
      isam_->update();
      last_optimized_loop_count_=loops_accepted_;
    }
    auto values=isam_->calculateEstimate();
    keys_.push_back(std::move(frame));
    optimized_.clear(); optimized_.reserve(keys_.size());
    for (size_t i=0;i<keys_.size();++i) optimized_.push_back(values.at<gtsam::Pose3>(key(i)));
    correction_=optimized_.back().compose(raw.inverse());
    // Corrected trajectory and map are republished after graph updates.
    // Never re-publish the same odometry timestamp: subscribers expect monotonic stamps.
    map_dirty_=true;
    publishPath();
    if (index%10==0) {
      RCLCPP_INFO(get_logger(),"P4 keyframes=%zu loops=%zu rejected=%zu points_last=%zu",
        keys_.size(),loops_accepted_,loops_rejected_,keys_.back().local->size());
    }
  }
  std::pair<int,double> detectLoop(const Keyframe &frame,size_t index) {
    if (!enable_loop_ || index<static_cast<size_t>(min_loop_frames_)) return {-1,1.0};
    if (has_accepted_loop_ &&
        index-last_accepted_loop_keyframe_<static_cast<size_t>(std::max(1,loop_accept_gap_)))
      return {-1,1.0};
    int best=-1; double score=descriptor_limit_;
    for (size_t i=0;i<index;++i) {
      if (index-i<static_cast<size_t>(min_loop_frames_)) continue;
      if (frame.stamp-keys_[i].stamp<min_loop_seconds_) continue;
      if ((frame.raw.translation()-keys_[i].raw.translation()).norm()>candidate_radius_) continue;
      double dist=descriptorDistance(frame.descriptor,keys_[i].descriptor);
      if (dist<score) {score=dist;best=static_cast<int>(i);}
    }
    return {best,score};
  }
  void publishCorrected(const nav_msgs::msg::Odometry &msg) {
    if (!corrected_pub_) return;
    nav_msgs::msg::Odometry corrected=msg;
    try {
      const auto pose=correction_.compose(toPose(msg.pose.pose));
      corrected.header.frame_id=map_frame_;
      corrected.child_frame_id=child_frame_;
      corrected.pose.pose=toRos(pose);
      Eigen::Matrix<double,6,6> covariance, jacobian=Eigen::Matrix<double,6,6>::Zero();
      jacobian.block<3,3>(0,0)=correction_.rotation().matrix();
      jacobian.block<3,3>(3,3)=correction_.rotation().matrix();
      for(int i=0;i<6;++i)for(int j=0;j<6;++j) covariance(i,j)=msg.pose.covariance[i*6+j];
      covariance=(jacobian*covariance*jacobian.transpose()).eval();
      for(int i=0;i<6;++i)for(int j=0;j<6;++j) corrected.pose.covariance[i*6+j]=covariance(i,j);
      corrected_pub_->publish(corrected);
      if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header=msg.header;tf.header.frame_id=map_frame_;tf.child_frame_id=input_frame_;
        auto p=toRos(correction_);
        tf.transform.translation.x=p.position.x;
        tf.transform.translation.y=p.position.y;
        tf.transform.translation.z=p.position.z;
        tf.transform.rotation=p.orientation;
        tf_->sendTransform(tf);
      }
    } catch(const std::exception &e) {
      RCLCPP_ERROR(get_logger(),"Cannot publish corrected odom: %s",e.what());
    }
  }
  void publishPath() {
    nav_msgs::msg::Path path;
    path.header.frame_id=map_frame_;
    path.header.stamp=now();
    for (size_t i=0;i<optimized_.size();++i) {
      geometry_msgs::msg::PoseStamped item;
      item.header.frame_id=map_frame_;
      item.header.stamp=rclcpp::Time(static_cast<int64_t>(keys_[i].stamp*1e9));
      item.pose=toRos(optimized_[i]);path.poses.push_back(item);
    }
    path_pub_->publish(path);
  }
  void publishMap() {
    auto combined=Cloud::Ptr(new Cloud);
    size_t total=0;
    for (const auto &k:keys_) total+=k.local->size();
    const size_t stride=std::max<size_t>(1,(total+static_cast<size_t>(max_map_points_)-1)/
                                         static_cast<size_t>(max_map_points_));
    combined->reserve(std::min(total,static_cast<size_t>(max_map_points_)));
    const auto map_begin=std::chrono::steady_clock::now();
    size_t global_index=0;
    for (size_t i=0;i<keys_.size();++i) {
      const auto transform = toMatrix(optimized_[i]);
      for (const auto &pt:keys_[i].local->points) {
        if ((global_index++%stride)!=0 || combined->size()>=static_cast<size_t>(max_map_points_)) continue;
        Point out = pt;
        out.getVector4fMap() = transform * pt.getVector4fMap();
        combined->push_back(out);
      }
    }
    cached_map_=downsample(combined,static_cast<float>(map_voxel_));
    map_build_ms_=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-map_begin).count();
    map_input_points_=total;
    map_sampled_points_=combined->size();
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cached_map_,msg);
    msg.header.frame_id=map_frame_;msg.header.stamp=now();
    map_pub_->publish(msg);
    map_dirty_=false;
  }
  void publishDiagnostics() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name="Stage4/Pose Graph";
    status.hardware_id="MID360S LIO";
    const bool at_capacity=keys_.size()>=static_cast<size_t>(max_keyframes_);
    status.level=keys_.empty()?diagnostic_msgs::msg::DiagnosticStatus::WARN:
                 at_capacity?diagnostic_msgs::msg::DiagnosticStatus::ERROR:
                             diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message=keys_.empty()?"waiting for synced LIO odom + cloud":
                   at_capacity?"KEYFRAME CAPACITY REACHED: mapping stopped; export map and start new session":
                               "running";
    auto add=[&](const std::string &k,const std::string &v){
      diagnostic_msgs::msg::KeyValue value;value.key=k;value.value=v;
      status.values.push_back(value);};
    add("keyframes",std::to_string(keys_.size()));
    add("max_keyframes",std::to_string(max_keyframes_));
    add("map_at_capacity",at_capacity?"true":"false");
    add("loop_accepted",std::to_string(loops_accepted_));
    add("loop_rejected",std::to_string(loops_rejected_));
    add("icp_checked",std::to_string(icp_checked_));
    add("loop_rejected_overlap",std::to_string(loops_rejected_overlap_));
    add("loop_last_overlap_ratio",std::to_string(last_loop_overlap_));
    add("loop_overlap_eval_ms",std::to_string(overlap_eval_ms_));
    add("map_input_points",std::to_string(map_input_points_));
    add("map_sampled_points",std::to_string(map_sampled_points_));
    add("map_final_points",std::to_string(cached_map_?cached_map_->size():0));
    add("map_build_ms",std::to_string(map_build_ms_));
    add("cloud_dropped",std::to_string(dropped_clouds_));
    add("bad_clouds",std::to_string(bad_clouds_));
    add("input_frame",input_frame_);add("map_frame",map_frame_);
    array.status.push_back(status);diag_pub_->publish(array);
    RCLCPP_INFO(get_logger(),"P4 kf=%zu loops=%zu rejected=%zu dropped=%zu",
      keys_.size(),loops_accepted_,loops_rejected_,dropped_clouds_);
  }

  std::string odom_topic_,cloud_topic_,input_frame_,map_frame_,child_frame_,output_dir_;
  bool publish_tf_{true},enable_loop_{true},warned_full_{false},map_dirty_{false};
  double distance_,angle_,sync_tolerance_,min_loop_seconds_,candidate_radius_;
  double descriptor_limit_,descriptor_radius_,icp_correspondence_,icp_fitness_;
  double max_loop_correction_,max_loop_rotation_,scan_voxel_,map_voxel_,map_period_;
  double loop_min_overlap_,loop_overlap_distance_;
  double last_loop_overlap_{0.0},overlap_eval_ms_{0.0},map_build_ms_{0.0};
  size_t map_input_points_{0},map_sampled_points_{0};
  size_t loops_rejected_overlap_{0};
  int min_loop_frames_,loop_accept_gap_,icp_iterations_,max_keyframes_,max_map_points_;
  int loop_overlap_sample_limit_;
  bool has_accepted_loop_{false};
  size_t last_accepted_loop_keyframe_{0};
  size_t dropped_clouds_{0},bad_clouds_{0},bad_frames_{0},icp_checked_{0};
  size_t loops_accepted_{0},loops_rejected_{0},last_optimized_loop_count_{0};
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odoms_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds_;
  std::vector<Keyframe> keys_;
  std::vector<gtsam::Pose3> optimized_;
  gtsam::Pose3 correction_;
  Cloud::Ptr cached_map_{new Cloud};
  std::unique_ptr<tbb::global_control> tbb_control_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr corrected_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<PoseGraphNode>()); }
  catch(const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("pose_graph_node"),"P4 fatal: %s",e.what());
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
