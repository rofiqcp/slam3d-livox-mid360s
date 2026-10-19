#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <limits>
#include <string>

class CloudFilter final : public rclcpp::Node {
public:
  CloudFilter():Node("cloud_filter") {
    const auto input=declare_parameter<std::string>("input_topic","/livox/lidar");
    const auto output=declare_parameter<std::string>("output_topic","/mid360/points_filtered");
    min_range_=declare_parameter<double>("min_range_m",0.25);
    max_range_=declare_parameter<double>("max_range_m",65.0);
    voxel_=declare_parameter<double>("voxel_m",0.15);
    max_points_=declare_parameter<int>("max_points",150000);
    if(!(voxel_>0.01 && voxel_<5.0 && min_range_>=0.0 && max_range_>min_range_
        && max_points_>=1000 && max_points_<=5000000)){
      throw std::invalid_argument("Invalid cloud_filter ranges or voxel/point limits");
    }
    pub_=create_publisher<sensor_msgs::msg::PointCloud2>(output,rclcpp::SensorDataQoS());
    sub_=create_subscription<sensor_msgs::msg::PointCloud2>(input,rclcpp::SensorDataQoS(),
      std::bind(&CloudFilter::onCloud,this,std::placeholders::_1));
    RCLCPP_INFO(get_logger(),"Read %s -> %s voxel=%.2f m (preview only; raw LIO timing unchanged)",
                input.c_str(),output.c_str(),voxel_);
  }
private:
  void onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
    pcl::PointCloud<pcl::PointXYZI> raw;
    try {pcl::fromROSMsg(*msg,raw);}
    catch(const std::exception& e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,
      "Invalid cloud fields: %s",e.what());return;}
    pcl::PointCloud<pcl::PointXYZI>::Ptr clean(new pcl::PointCloud<pcl::PointXYZI>());
    clean->reserve(std::min<size_t>(raw.size(),static_cast<size_t>(max_points_)));
    const float a=static_cast<float>(min_range_*min_range_);
    const float b=static_cast<float>(max_range_*max_range_);
    const size_t stride=std::max<size_t>(1,(raw.size()+static_cast<size_t>(max_points_)-1)/static_cast<size_t>(max_points_));
    for(size_t i=0;i<raw.size();i+=stride){
      auto& pt=raw.points[i];
      const float r=pt.x*pt.x+pt.y*pt.y+pt.z*pt.z;
      if(std::isfinite(r)&&r>=a&&r<=b)clean->push_back(pt);
    }
    if(clean->empty())return;
    pcl::VoxelGrid<pcl::PointXYZI> vf;
    vf.setInputCloud(clean);
    const float leaf=static_cast<float>(voxel_);
    vf.setLeafSize(leaf,leaf,leaf);
    pcl::PointCloud<pcl::PointXYZI> filtered;
    vf.filter(filtered);
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(filtered,out);
    out.header=msg->header;
    pub_->publish(out);
  }
  double min_range_,max_range_,voxel_;
  int max_points_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<CloudFilter>());}
  catch(const std::exception& e){std::cerr<<e.what()<<std::endl; rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
