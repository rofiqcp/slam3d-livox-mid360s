#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fs=std::filesystem;
using Point=pcl::PointXYZI;
using Cloud=pcl::PointCloud<Point>;
using Matrix=Eigen::Matrix4f;
constexpr float PI=3.14159265358979323846f;

static Matrix matrixOf(const geometry_msgs::msg::Pose &p) {
  Eigen::Quaternionf q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
  if (!std::isfinite(q.norm()) || q.norm()<0.8) throw std::runtime_error("invalid quaternion");
  Matrix m=Matrix::Identity();
  m.block<3,3>(0,0)=q.normalized().toRotationMatrix();
  m(0,3)=p.position.x;m(1,3)=p.position.y;m(2,3)=p.position.z;
  if (!m.allFinite()) throw std::runtime_error("invalid position");
  return m;
}
static geometry_msgs::msg::Pose poseOf(const Matrix &m) {
  geometry_msgs::msg::Pose p;
  Eigen::Quaternionf q(m.block<3,3>(0,0));
  q.normalize();
  p.position.x=m(0,3);p.position.y=m(1,3);p.position.z=m(2,3);
  p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();
  return p;
}
static Cloud::Ptr voxel(const Cloud::Ptr &cloud,float leaf) {
  auto o=Cloud::Ptr(new Cloud());
  if(cloud->empty()) return o;
  pcl::VoxelGrid<Point> filter;filter.setInputCloud(cloud);
  filter.setLeafSize(leaf,leaf,leaf);filter.filter(*o);return o;
}
static Matrix translateToMatrix(float x,float y,float z,float qx,float qy,float qz,float qw) {
  geometry_msgs::msg::Pose p;
  p.position.x=x;p.position.y=y;p.position.z=z;
  p.orientation.x=qx;p.orientation.y=qy;p.orientation.z=qz;p.orientation.w=qw;
  return matrixOf(p);
}
struct Result {
  bool accepted{false};
  Matrix transform{Matrix::Identity()};
  float fitness{std::numeric_limits<float>::infinity()};
  float overlap{0};
  size_t seed{0};
};

class Relocalizer : public rclcpp::Node {
 public:
  Relocalizer():Node("relocalizer_node") {
    pcd_path_=declare_parameter<std::string>("map_pcd","");
    csv_path_=declare_parameter<std::string>("poses_csv","");
    odom_topic_=declare_parameter<std::string>("odom_topic","/lio/odometry");
    cloud_topic_=declare_parameter<std::string>("cloud_topic","/lio/cloud_registered");
    map_frame_=declare_parameter<std::string>("map_frame","map");
    odom_frame_=declare_parameter<std::string>("odom_frame","lio_odom");
    sensor_frame_=declare_parameter<std::string>("sensor_frame","livox_frame");
    output_dir_=declare_parameter<std::string>("pose_output_dir","");
    use_tf_=declare_parameter<bool>("use_tf",true);
    auto_start_=declare_parameter<bool>("auto_global_on_start",true);
    auto_recovery_=declare_parameter<bool>("auto_recovery",true);
    min_points_=declare_parameter<int>("min_scan_points",100);
    sync_tol_=declare_parameter<double>("sync_tolerance_s",0.035);
    map_leaf_=declare_parameter<double>("map_voxel_m",0.4);
    scan_leaf_=declare_parameter<double>("scan_voxel_m",0.45);
    coarse_leaf_=declare_parameter<double>("coarse_voxel_m",0.85);
    local_radius_=declare_parameter<double>("local_map_radius_m",17.0);
    coarse_dist_=declare_parameter<double>("icp_coarse_correspondence_m",2.5);
    fine_dist_=declare_parameter<double>("icp_fine_correspondence_m",1.2);
    coarse_iters_=declare_parameter<int>("icp_coarse_iterations",18);
    fine_iters_=declare_parameter<int>("icp_fine_iterations",35);
    max_candidates_=declare_parameter<int>("max_candidates",16);
    icp_update_stride_=declare_parameter<int>("icp_update_stride",4);
    min_overlap_=declare_parameter<double>("min_overlap",0.52);
    max_fitness_=declare_parameter<double>("max_fitness",0.30);
    max_tracking_translation_=declare_parameter<double>("max_tracking_correction_m",1.5);
    max_tracking_yaw_=declare_parameter<double>("max_tracking_correction_deg",30)*PI/180;
    failure_limit_=declare_parameter<int>("tracking_failure_limit",3);
    recovery_cooldown_=declare_parameter<double>("recovery_cooldown_s",3);
    max_path_=declare_parameter<int>("max_path_poses",10000);

    if(pcd_path_.empty()) throw std::runtime_error(
      "P5 requires explicit map_pcd:=/path/to/stage4_map_*.pcd; never choose test map automatically");
    if(min_points_<25||sync_tol_<=0||map_leaf_<=0||scan_leaf_<=0||coarse_leaf_<=0
      ||local_radius_<=1||fine_dist_<=0||coarse_dist_<=0||coarse_iters_<2||fine_iters_<2
      ||min_overlap_<=0||min_overlap_>1||max_candidates_<1||max_candidates_>128
      ||failure_limit_<1||recovery_cooldown_<=0||max_path_<10||icp_update_stride_<1) {
      throw std::runtime_error("P5 parameters outside safe bounds");
    }
    loadMap();
    last_search_wall_=std::chrono::steady_clock::now()-std::chrono::hours(1);
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    status_pub_=create_publisher<std_msgs::msg::String>("/p5/status",rclcpp::QoS(1).transient_local());
    map_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/p5/reference_map",rclcpp::QoS(1).transient_local());
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("/p5/localized_odometry",20);
    path_pub_=create_publisher<nav_msgs::msg::Path>("/p5/localized_path",rclcpp::QoS(1).transient_local());
    aligned_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/p5/aligned_scan",rclcpp::SensorDataQoS());
    diag_pub_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",10);
    reset_sub_=create_subscription<std_msgs::msg::Empty>("/lio/reset",10,
      [this](std_msgs::msg::Empty::ConstSharedPtr){resetLocalization();});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
        if(m->header.frame_id!=odom_frame_||m->child_frame_id!=sensor_frame_) {
          ++bad_frames_;return;
        }
        if (rclcpp::Time(m->header.stamp).seconds() < last_stamp_-sync_tol_) resetLocalization();
        odoms_.push_back(m);if(odoms_.size()>120)odoms_.pop_front();
        synchronize();
      });
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){
        if(m->header.frame_id!=odom_frame_) {++bad_frames_;return;}
        if (rclcpp::Time(m->header.stamp).seconds() < last_stamp_-sync_tol_) resetLocalization();
        clouds_.push_back(m);if(clouds_.size()>8){clouds_.pop_front();++dropped_clouds_;}
        synchronize();
      });
    initial_sub_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose",rclcpp::QoS(10),[this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
        if(m->header.frame_id!=map_frame_) {
          RCLCPP_ERROR(get_logger(),"/initialpose must use frame '%s' (sensor pose, not base_link)",map_frame_.c_str());
          return;
        }
        try {
          manual_pose_=matrixOf(m->pose.pose);
          manual_pending_=true;state_="SEARCHING";failure_count_=0;
          publishStatus();
          RCLCPP_INFO(get_logger(),"Received manual /initialpose hint in map frame");
        } catch(const std::exception &e){RCLCPP_ERROR(get_logger(),"Invalid /initialpose: %s",e.what());}
      });
    relocalize_srv_=create_service<std_srvs::srv::Trigger>("/p5/relocalize",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        search_requested_=true;state_="SEARCHING";failure_count_=0;
        response->success=true;response->message="Global relocalization scheduled for next synchronized LiDAR scan";
        publishStatus();
      });
    reload_srv_=create_service<std_srvs::srv::Trigger>("/p5/reload_map",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        try {
          auto newpcd=get_parameter("map_pcd").as_string();
          auto newcsv=get_parameter("poses_csv").as_string();
          const auto oldpcd=pcd_path_,oldcsv=csv_path_;
          pcd_path_=newpcd;csv_path_=newcsv;
          try{loadMap();}
          catch(...){pcd_path_=oldpcd;csv_path_=oldcsv;loadMap();throw;}
          resetLocalization();
          publishReferenceMap();publishStatus();
          response->success=true;response->message="P5 map reloaded; localization reset";
        }catch(const std::exception &e){
          response->success=false;response->message=e.what();
        }
      });
    save_srv_=create_service<std_srvs::srv::Trigger>("/p5/save_pose",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if(state_!="TRACKING") {response->success=false;response->message="Not localized";return;}
        try{
          if(output_dir_.empty())throw std::runtime_error("pose_output_dir not configured");
          fs::create_directories(output_dir_);
          const auto stamp=std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::system_clock::now().time_since_epoch()).count();
          auto p=fs::path(output_dir_)/("p5_pose_"+std::to_string(stamp)+".yaml");
          std::ofstream f(p);
          if(!f)throw std::runtime_error("cannot write pose file");
          auto ros=poseOf(last_map_sensor_);
          f<<std::setprecision(12)<<"map_pcd: "<<pcd_path_<<"\n"
           <<"frame_id: "<<map_frame_<<"\n"
           <<"sensor_frame: "<<sensor_frame_<<"\n"
           <<"fitness: "<<last_fitness_<<"\n"<<"overlap: "<<last_overlap_<<"\n"
           <<"position_xyz: ["<<ros.position.x<<", "<<ros.position.y<<", "<<ros.position.z<<"]\n"
           <<"orientation_xyzw: ["<<ros.orientation.x<<", "<<ros.orientation.y<<", "
           <<ros.orientation.z<<", "<<ros.orientation.w<<"]\n";
          response->success=true;response->message=p.string();
        }catch(const std::exception &e){response->success=false;response->message=e.what();}
      });
    status_timer_=create_wall_timer(std::chrono::seconds(1),[this](){
      if(state_=="TRACKING" && (std::chrono::steady_clock::now()-last_scan_wall_)>
           std::chrono::seconds(3)) {
        state_="LOST";++stale_events_;
      }
      publishDiagnostics();publishStatus();
    });
    search_requested_=auto_start_;
    publishReferenceMap();
    publishStatus();
    RCLCPP_INFO(get_logger(),"P5 loaded %zu map points and %zu pose seeds from %s",
      map_->size(),seeds_.size(),pcd_path_.c_str());
    if(seeds_.size()<6) {
      RCLCPP_WARN(get_logger(),"Only %zu P4 keyframes in saved map; global relocalization "
        "may lack scan overlap. Prefer dense mapped route and verify via offline replay.",seeds_.size());
    }
  }

 private:
  void loadMap() {
    if(!fs::is_regular_file(pcd_path_))throw std::runtime_error("PCD map missing: "+pcd_path_);
    auto loaded=Cloud::Ptr(new Cloud);
    if(pcl::io::loadPCDFile<Point>(pcd_path_,*loaded)<0||loaded->size()<100)
      throw std::runtime_error("Invalid or too small P4 map PCD: "+pcd_path_);
    loaded->points.erase(std::remove_if(loaded->points.begin(),loaded->points.end(),
      [](const Point &p){return !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z);}),
      loaded->points.end());
    loaded->width=loaded->points.size();loaded->height=1;loaded->is_dense=true;
    if(loaded->size()<100)throw std::runtime_error("No valid map points");
    auto map=voxel(loaded,map_leaf_);
    if(map->size()<100)throw std::runtime_error("Downsampled map too small");
    std::string csv=csv_path_;
    if(csv.empty()) {
      const auto name=fs::path(pcd_path_).filename().string();
      if(name.rfind("stage4_map_",0)==0) csv=(fs::path(pcd_path_).parent_path()/
         ("stage4_poses_"+name.substr(std::string("stage4_map_").size(),
          name.size()-std::string("stage4_map_").size()-4)+".csv")).string();
    }
    std::vector<Matrix> seed;
    if(!csv.empty()&&fs::is_regular_file(csv)){
      std::ifstream f(csv);std::string line;
      std::getline(f,line);
      if(line!="stamp,x,y,z,qx,qy,qz,qw")
        throw std::runtime_error("Invalid pose CSV header: "+csv);
      while(std::getline(f,line)) {
        std::stringstream ss(line);std::string entry;std::vector<float> row;
        try{
          while(std::getline(ss,entry,','))row.push_back(std::stof(entry));
          if(row.size()==8)seed.push_back(translateToMatrix(row[1],row[2],row[3],row[4],row[5],row[6],row[7]));
        }catch(const std::exception&){}
      }
    }
    map_=map;seeds_=std::move(seed);csv_path_=csv;
  }
  void publishReferenceMap(){
    sensor_msgs::msg::PointCloud2 msg;pcl::toROSMsg(*map_,msg);
    msg.header.stamp=now();msg.header.frame_id=map_frame_;map_pub_->publish(msg);
  }
  Cloud::Ptr roi(const Matrix &hint) {
    auto local=Cloud::Ptr(new Cloud);
    const Eigen::Vector3f center=hint.block<3,1>(0,3);
    const float r2=local_radius_*local_radius_;
    local->reserve(map_->size()/4);
    for(const auto &p:map_->points) {
      const float dx=p.x-center.x(),dy=p.y-center.y(),dz=p.z-center.z();
      if(dx*dx+dy*dy+dz*dz<=r2)local->push_back(p);
    }
    if(local->size()<min_points_)return map_;
    return local;
  }
  Result registerScan(const Cloud::Ptr &full,const Cloud::Ptr &coarse,
                      const Matrix &guess,size_t seed_idx,bool fast_tracking=false) {
    Result result;result.seed=seed_idx;
    auto target=roi(guess);
    Matrix coarse_result=guess;
    if(!fast_tracking) {
      pcl::IterativeClosestPoint<Point,Point> icp;
      icp.setInputSource(coarse);
      icp.setInputTarget(voxel(target,coarse_leaf_));
      icp.setMaximumIterations(coarse_iters_);
      icp.setMaxCorrespondenceDistance(coarse_dist_);
      icp.setTransformationEpsilon(1e-6);
      Cloud aligned;
      icp.align(aligned,guess);
      if(!icp.hasConverged())return result;
      coarse_result=icp.getFinalTransformation();
      target=roi(coarse_result);
    }
    pcl::IterativeClosestPoint<Point,Point> fine;
    fine.setInputSource(full);fine.setInputTarget(target);
    fine.setMaxCorrespondenceDistance(fine_dist_);
    fine.setMaximumIterations(fine_iters_);
    fine.setTransformationEpsilon(1e-7);
    Cloud output;
    fine.align(output,coarse_result);
    if(!fine.hasConverged())return result;
    const float score=fine.getFitnessScore();
    if(!std::isfinite(score))return result;
    pcl::KdTreeFLANN<Point> tree;tree.setInputCloud(target);
    size_t inliers=0;
    std::vector<int> indices(1);std::vector<float> sqdist(1);
    const float r2=fine_dist_*fine_dist_;
    for(const auto &point:output.points) {
      if(tree.nearestKSearch(point,1,indices,sqdist)>0&&sqdist[0]<r2)++inliers;
    }
    result.transform=fine.getFinalTransformation();result.fitness=score;
    result.overlap=output.empty()?0.f:static_cast<float>(inliers)/output.size();
    result.accepted=score<=max_fitness_ && result.overlap>=min_overlap_ && result.transform.allFinite();
    return result;
  }
  void resetLocalization() {
    state_="UNINITIALIZED";last_accepted_=false;path_.poses.clear();
    last_stamp_=-1;odoms_.clear();clouds_.clear();failure_count_=0;
    last_search_wall_=std::chrono::steady_clock::time_point{};
    last_scan_wall_=std::chrono::steady_clock::time_point{};
    correction_=Matrix::Identity();last_map_sensor_=Matrix::Identity();
    search_requested_=auto_start_;manual_pending_=false;received_scans_=0;
    publishStatus();
  }
  void synchronize() {
    while(!clouds_.empty()&&!odoms_.empty()) {
      auto cloud=clouds_.front();
      const double stamp=rclcpp::Time(cloud->header.stamp).seconds();
      size_t best=0;double dt=std::numeric_limits<double>::infinity();
      for(size_t i=0;i<odoms_.size();++i) {
        double diff=std::abs(rclcpp::Time(odoms_[i]->header.stamp).seconds()-stamp);
        if(diff<dt){dt=diff;best=i;}
      }
      if(dt<=sync_tol_) {
        auto odom=odoms_[best];clouds_.pop_front();
        odoms_.erase(odoms_.begin(),odoms_.begin()+best+1);
        try{process(*odom,*cloud);}
        catch(const std::exception &e){++errors_;RCLCPP_ERROR(get_logger(),"P5 process: %s",e.what());}
      }else {
        const double newest=rclcpp::Time(odoms_.back()->header.stamp).seconds();
        if(newest>stamp+sync_tol_){clouds_.pop_front();++dropped_clouds_;}
        else break;
      }
    }
  }
  void process(const nav_msgs::msg::Odometry &odom,const sensor_msgs::msg::PointCloud2 &msg){
    const double stamp=rclcpp::Time(msg.header.stamp).seconds();
    if(stamp<=last_stamp_){++nonmonotonic_;return;}
    last_stamp_=stamp;
    Cloud::Ptr scan_odom(new Cloud);pcl::fromROSMsg(msg,*scan_odom);
    scan_odom->points.erase(std::remove_if(scan_odom->points.begin(),scan_odom->points.end(),
      [](const Point &p){return !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z);}),
      scan_odom->points.end());
    scan_odom->width=scan_odom->points.size();scan_odom->height=1;
    if(scan_odom->size()<static_cast<size_t>(min_points_)){++bad_clouds_;return;}
    Matrix odom_sensor=matrixOf(odom.pose.pose);
    Cloud::Ptr local(new Cloud);pcl::transformPointCloud(*scan_odom,*local,odom_sensor.inverse());
    local=voxel(local,scan_leaf_);
    if(local->size()<static_cast<size_t>(min_points_)){++bad_clouds_;return;}
    const auto wall=std::chrono::steady_clock::now();
    ++received_scans_;
    if(state_=="TRACKING" && !search_requested_ && !manual_pending_ &&
       received_scans_%static_cast<size_t>(icp_update_stride_)!=0) {
      last_map_sensor_=correction_*odom_sensor;
      last_scan_wall_=std::chrono::steady_clock::now();
      ++prediction_scans_;
      publishPose(odom,msg,local);
      return;
    }
    auto coarse=voxel(local,coarse_leaf_);
    Result best;
    float best_rejected_fitness=std::numeric_limits<float>::infinity();
    float max_rejected_overlap=0.0f;
    bool global=state_!="TRACKING"||search_requested_||manual_pending_;
    if(global) {
      if(!manual_pending_ && !search_requested_ && (!auto_recovery_||state_!="LOST"))
        return;
      if(!manual_pending_ && !search_requested_ &&
         std::chrono::duration<double>(wall-last_search_wall_).count()<recovery_cooldown_)return;
      last_search_wall_=wall;
      if(manual_pending_) {
        best=registerScan(local,coarse,manual_pose_,0);
        manual_pending_=false;
      }else {
        if(seeds_.empty()) {
          ++search_failures_;state_="UNINITIALIZED";
          RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),6000,
            "Global relocalization needs stage4_poses CSV, or /initialpose in map frame");
          search_requested_=false;return;
        }
        // Scan up to max_candidates poses distributed along the saved P4 trajectory.
        const size_t max=std::min(seeds_.size(),static_cast<size_t>(max_candidates_));
        for(size_t i=0;i<max;++i) {
          const size_t index=max==1?0:(i*(seeds_.size()-1)/(max-1));
          Result trial=registerScan(local,coarse,seeds_[index],index);
          if(std::isfinite(trial.fitness)) {
            best_rejected_fitness=std::min(best_rejected_fitness,trial.fitness);
            max_rejected_overlap=std::max(max_rejected_overlap,trial.overlap);
          }
          if(trial.accepted && (!best.accepted ||
              trial.fitness<best.fitness))best=trial;
          if(i>=2 && best.accepted && best.fitness<0.015f && best.overlap>0.96f)break;
        }
      }
      search_requested_=false;
      if(!best.accepted) {
        state_="LOST";
        ++search_failures_;
        RCLCPP_WARN(get_logger(),
          "P5 global search rejected: map_points=%zu candidate_seeds=%zu "
          "best_fitness=%f max_overlap=%f (requires fitness<=%.3f overlap>=%.2f). "
          "Sparse P4 maps may require more survey keyframes.",
          map_->size(),seeds_.size(),best_rejected_fitness,max_rejected_overlap,
          max_fitness_,min_overlap_);
        publishStatus();return;
      }
      correction_=best.transform*odom_sensor.inverse();
      state_="TRACKING";failure_count_=0;++relocalizations_;
      RCLCPP_WARN(get_logger(),"P5 RELOCALIZED seed=%zu fitness=%.5f overlap=%.3f",
        best.seed,best.fitness,best.overlap);
    }else {
      const Matrix predicted=correction_*odom_sensor;
      best=registerScan(local,coarse,predicted,0,true);
      if(best.accepted) {
        const Matrix delta=predicted.inverse()*best.transform;
        const float shift=delta.block<3,1>(0,3).norm();
        const float angle=Eigen::AngleAxisf(delta.block<3,3>(0,0)).angle();
        if(shift>max_tracking_translation_||angle>max_tracking_yaw_)
          best.accepted=false;
      }
      if(!best.accepted) {
        ++failure_count_;++tracking_failures_;
        if(failure_count_>=failure_limit_) {
          state_="LOST";
          RCLCPP_ERROR(get_logger(),"P5 TRACKING LOST after %d consecutive failed alignments",failure_count_);
        }
        publishStatus();return;
      }
      failure_count_=0;correction_=best.transform*odom_sensor.inverse();
    }
    last_fitness_=best.fitness;last_overlap_=best.overlap;
    last_map_sensor_=best.transform;
    last_scan_wall_=std::chrono::steady_clock::now();last_accepted_=true;
    ++accepted_scans_;++icp_refinements_;
    publishPose(odom,msg,local);
  }
  void publishPose(const nav_msgs::msg::Odometry &odom,const sensor_msgs::msg::PointCloud2 &cloud,const Cloud::Ptr &local) {
    nav_msgs::msg::Odometry pose=odom;
    pose.header.frame_id=map_frame_;pose.child_frame_id=sensor_frame_;
    pose.pose.pose=poseOf(last_map_sensor_);
    // Original odometry covariance refers to a different frame/model. We cannot
    // claim the same uncertainty; mark conservative diagonal based on ICP quality.
    std::fill(pose.pose.covariance.begin(),pose.pose.covariance.end(),0.0);
    const double sigma=std::max(0.04,std::sqrt(last_fitness_));
    for(int i=0;i<3;++i)pose.pose.covariance[i*6+i]=sigma*sigma;
    for(int i=3;i<6;++i)pose.pose.covariance[i*6+i]=0.12;
    odom_pub_->publish(pose);
    geometry_msgs::msg::PoseStamped item;item.header=pose.header;item.pose=pose.pose.pose;
    path_.header.frame_id=map_frame_;path_.header.stamp=pose.header.stamp;
    path_.poses.push_back(item);
    if(path_.poses.size()>static_cast<size_t>(max_path_))path_.poses.erase(path_.poses.begin());
    if (path_pub_->get_subscription_count()>0 &&
        std::chrono::steady_clock::now()-last_path_wall_>=std::chrono::seconds(1)) {
      path_pub_->publish(path_); last_path_wall_=std::chrono::steady_clock::now();
    }
    if(use_tf_){
      const Matrix m=correction_;
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp=odom.header.stamp;tf.header.frame_id=map_frame_;
      tf.child_frame_id=odom_frame_;
      auto p=poseOf(m);
      tf.transform.translation.x=p.position.x;
      tf.transform.translation.y=p.position.y;
      tf.transform.translation.z=p.position.z;
      tf.transform.rotation=p.orientation;
      tf_->sendTransform(tf);
    }
    Cloud registered;pcl::transformPointCloud(*local,registered,last_map_sensor_);
    sensor_msgs::msg::PointCloud2 aligned;pcl::toROSMsg(registered,aligned);
    aligned.header.frame_id=map_frame_;aligned.header.stamp=cloud.header.stamp;
    aligned_pub_->publish(aligned);
  }
  void publishStatus(){
    if(!status_pub_)return;
    std_msgs::msg::String msg;
    std::ostringstream out;
    out<<state_<<" | accepted="<<accepted_scans_<<" relocalizations="<<relocalizations_
       <<" predictions="<<prediction_scans_<<" icp_refinements="<<icp_refinements_
       <<" tracking_failures="<<tracking_failures_<<" search_failures="<<search_failures_
       <<" map_points="<<map_->size()<<" fitness="<<last_fitness_
       <<" overlap="<<last_overlap_;
    msg.data=out.str();status_pub_->publish(msg);
  }
  void publishDiagnostics(){
    diagnostic_msgs::msg::DiagnosticArray array;array.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus s;s.name="Stage5/Map Localization";
    s.hardware_id="Livox MID360S saved P4 map";
    s.level=state_=="TRACKING"?0:state_=="SEARCHING"?1:2;
    s.message=state_;
    auto add=[&](std::string k,std::string v) {
      diagnostic_msgs::msg::KeyValue kv;kv.key=k;kv.value=v;s.values.push_back(kv);};
    add("map_pcd",pcd_path_);
    add("pose_csv",csv_path_);
    add("map_points",std::to_string(map_->size()));
    add("pose_candidates",std::to_string(seeds_.size()));
    add("accepted_scans",std::to_string(accepted_scans_));
    add("predictions",std::to_string(prediction_scans_));
    add("icp_refinements",std::to_string(icp_refinements_));
    add("relocalizations",std::to_string(relocalizations_));
    add("tracking_failures",std::to_string(tracking_failures_));
    add("search_failures",std::to_string(search_failures_));
    add("dropped_clouds",std::to_string(dropped_clouds_));
    add("nonmonotonic_stamps",std::to_string(nonmonotonic_));
    add("bad_frames",std::to_string(bad_frames_));
    add("fitness",std::to_string(last_fitness_));
    add("overlap",std::to_string(last_overlap_));
    array.status.push_back(s);diag_pub_->publish(array);
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,
      "P5 %s scanned=%zu recovered=%zu fitness=%.4f overlap=%.2f dropped=%zu",
      state_.c_str(),accepted_scans_,relocalizations_,last_fitness_,last_overlap_,dropped_clouds_);
  }

  std::string pcd_path_,csv_path_,odom_topic_,cloud_topic_,map_frame_,odom_frame_,sensor_frame_,output_dir_;
  bool use_tf_{true},auto_start_{true},auto_recovery_{true};
  bool search_requested_{false},manual_pending_{false},last_accepted_{false};
  std::string state_{"UNINITIALIZED"};
  int min_points_,coarse_iters_,fine_iters_,max_candidates_,icp_update_stride_,
      failure_limit_,failure_count_{0},max_path_;
  double sync_tol_,map_leaf_,scan_leaf_,coarse_leaf_,local_radius_,coarse_dist_,fine_dist_;
  double min_overlap_,max_fitness_,max_tracking_translation_,max_tracking_yaw_,recovery_cooldown_;
  std::chrono::steady_clock::time_point last_path_wall_{};
  double last_stamp_{-1},last_fitness_{-1},last_overlap_{0};
  size_t dropped_clouds_{0},bad_clouds_{0},bad_frames_{0},errors_{0},nonmonotonic_{0};
  size_t accepted_scans_{0},prediction_scans_{0},received_scans_{0},
         icp_refinements_{0},relocalizations_{0},tracking_failures_{0},search_failures_{0},stale_events_{0};
  Cloud::Ptr map_{new Cloud};
  std::vector<Matrix> seeds_;
  Matrix correction_{Matrix::Identity()},last_map_sensor_{Matrix::Identity()},
         manual_pose_{Matrix::Identity()};
  nav_msgs::msg::Path path_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odoms_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds_;
  std::chrono::steady_clock::time_point last_scan_wall_{std::chrono::steady_clock::now()},
                                  last_search_wall_{std::chrono::steady_clock::time_point::min()};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_,aligned_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_srv_,reload_srv_,save_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
int main(int argc,char **argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<Relocalizer>());}
  catch(const std::exception &e){
    RCLCPP_FATAL(rclcpp::get_logger("relocalizer_node"),"P5 startup failed: %s",e.what());
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
