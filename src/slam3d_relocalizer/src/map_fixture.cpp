#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using Point=pcl::PointXYZI;
using Cloud=pcl::PointCloud<Point>;
using Matrix=Eigen::Matrix4f;

static nav_msgs::msg::Odometry makeOdom(
    const Matrix &pose, const builtin_interfaces::msg::Time &stamp, const std::string &frame) {
  nav_msgs::msg::Odometry m;m.header.stamp=stamp;m.header.frame_id=frame;
  m.child_frame_id="livox_frame";
  Eigen::Quaternionf q(pose.block<3,3>(0,0));
  m.pose.pose.position.x=pose(0,3);
  m.pose.pose.position.y=pose(1,3);
  m.pose.pose.position.z=pose(2,3);
  m.pose.pose.orientation.x=q.x();m.pose.pose.orientation.y=q.y();
  m.pose.pose.orientation.z=q.z();m.pose.pose.orientation.w=q.w();
  return m;
}
class MapFixture : public rclcpp::Node {
 public:
  MapFixture():Node("stage5_map_fixture") {
    map_path_=declare_parameter<std::string>("map_pcd","");
    csv_path_=declare_parameter<std::string>("poses_csv","");
    rate_=declare_parameter<double>("rate_hz",5.0);
    frames_=declare_parameter<int>("frames",100);
    start_index_=declare_parameter<int>("start_index",0);
    inject_start_=declare_parameter<int>("inject_start",0);
    inject_frames_=declare_parameter<int>("inject_frames",0);
    blackout_start_=declare_parameter<int>("blackout_start",0);
    blackout_frames_=declare_parameter<int>("blackout_frames",0);
    inject_m_=declare_parameter<double>("inject_translation_m",5.0);
    radius_=declare_parameter<double>("scan_radius_m",13.0);
    if(map_path_.empty()||csv_path_.empty())throw std::runtime_error("P5 fixture needs saved P4 map PCD and poses CSV");
    if(pcl::io::loadPCDFile<Point>(map_path_,world_)<0)throw std::runtime_error("fixture PCD failed");
    std::ifstream file(csv_path_);std::string line;std::getline(file,line);
    if(line!="stamp,x,y,z,qx,qy,qz,qw")throw std::runtime_error("fixture CSV header wrong");
    while(std::getline(file,line)) {
      std::stringstream ss(line);std::string part;std::vector<float> vals;
      while(std::getline(ss,part,','))vals.push_back(std::stof(part));
      if(vals.size()!=8)continue;
      Eigen::Quaternionf q(vals[7],vals[4],vals[5],vals[6]);
      Matrix m=Matrix::Identity();
      m.block<3,3>(0,0)=q.normalized().toRotationMatrix();
      m(0,3)=vals[1];m(1,3)=vals[2];m(2,3)=vals[3];
      poses_.push_back(m);
    }
    if(poses_.empty())throw std::runtime_error("Fixture no saved poses");
    start_index_=std::clamp(start_index_,0,static_cast<int>(poses_.size()-1));
    origin_=poses_[start_index_];
    cloud_pub_=create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lio/cloud_registered",rclcpp::SensorDataQoS());
    sensor_cloud_pub_=create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lio/cloud_sensor",rclcpp::SensorDataQoS());
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>(
      "/lio/odometry",rclcpp::SensorDataQoS());
    gt_pub_=create_publisher<nav_msgs::msg::Odometry>(
      "/p5/fixture/ground_truth",rclcpp::QoS(30));
    timer_=create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0/rate_)),[this](){publish();});
    RCLCPP_INFO(get_logger(),"P5 map fixture map points=%zu pose candidates=%zu origin index=%d",
      world_.size(),poses_.size(),start_index_);
  }
 private:
  void publish(){
    const int current=frame_++;
    if(current>=frames_)return;
    const int pose_index=std::min(start_index_+current/4,static_cast<int>(poses_.size()-1));
    Matrix true_map_sensor=poses_[pose_index];
    Matrix odom_sensor=origin_.inverse()*true_map_sensor;
    const builtin_interfaces::msg::Time stamp=now();
    auto odom=makeOdom(odom_sensor,stamp,"lio_odom");
    auto gt=makeOdom(true_map_sensor,stamp,"map");
    if(current>=blackout_start_ && current<blackout_start_+blackout_frames_) {
      odom_pub_->publish(odom);
      gt_pub_->publish(gt);
      if(current==blackout_start_)RCLCPP_WARN(get_logger(),"P5 fixture cloud blackout START %d frames",blackout_frames_);
      if(current==blackout_start_+blackout_frames_-1)
        RCLCPP_WARN(get_logger(),"P5 fixture cloud blackout END; relocalization should recover");
      return;
    }
    auto cloud_odom=Cloud::Ptr(new Cloud);
    auto cloud_sensor=Cloud::Ptr(new Cloud);
    cloud_odom->reserve(world_.size());
    cloud_sensor->reserve(world_.size());
    const auto position=true_map_sensor.block<3,1>(0,3);
    const Matrix transform=odom_sensor*true_map_sensor.inverse();
    const Matrix sensor_from_odom=odom_sensor.inverse();
    const float radius2=radius_*radius_;
    bool inject=(current>=inject_start_&&current<inject_start_+inject_frames_);
    for(const auto &p:world_.points){
      const float dx=p.x-position.x(),dy=p.y-position.y(),dz=p.z-position.z();
      if(dx*dx+dy*dy+dz*dz>radius2)continue;
      Eigen::Vector4f value(p.x,p.y,p.z,1.f);
      value=transform*value;
      if(inject)value.x()+=inject_m_;
      Point out;out.x=value.x();out.y=value.y();out.z=value.z();out.intensity=p.intensity;
      cloud_odom->push_back(out);
      // Dynamic obstacle sensors MUST be expressed relative to their moving
      // origin so rolling Nav2 costmaps can raytrace from the true LiDAR pose.
      const Eigen::Vector4f sensor_value=sensor_from_odom*value;
      Point local;
      local.x=sensor_value.x();local.y=sensor_value.y();local.z=sensor_value.z();
      local.intensity=p.intensity;
      cloud_sensor->push_back(local);
    }
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::toROSMsg(*cloud_odom,cloud);
    cloud.header.stamp=stamp;
    cloud.header.frame_id="lio_odom";
    sensor_msgs::msg::PointCloud2 sensor_cloud;
    pcl::toROSMsg(*cloud_sensor,sensor_cloud);
    sensor_cloud.header.stamp=stamp;
    sensor_cloud.header.frame_id="livox_frame";
    odom_pub_->publish(odom);
    cloud_pub_->publish(cloud);
    sensor_cloud_pub_->publish(sensor_cloud);
    gt_pub_->publish(gt);
    if(current%20==0) RCLCPP_INFO(get_logger(),"P5 fixture frame=%d pose_index=%d cloud=%zu injected=%s",
      current,pose_index,cloud_odom->size(),inject?"yes":"no");
  }
  std::string map_path_,csv_path_;
  double rate_{5},radius_{13},inject_m_{5};
  int frames_{100},frame_{0},start_index_{0},inject_start_{0},inject_frames_{0},
      blackout_start_{0},blackout_frames_{0};
  Matrix origin_{Matrix::Identity()};
  Cloud world_;
  std::vector<Matrix> poses_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_,gt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_,sensor_cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<MapFixture>());}
  catch(const std::exception &e){
    RCLCPP_FATAL(rclcpp::get_logger("stage5_map_fixture"),"Fixture: %s",e.what());
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
