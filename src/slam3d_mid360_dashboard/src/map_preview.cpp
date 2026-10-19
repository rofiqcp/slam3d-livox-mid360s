#include <cstdlib>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <stdexcept>

class MapPreview final : public rclcpp::Node {
public:
  MapPreview():Node("map_preview") {
    const auto input=declare_parameter<std::string>("input_topic","/p4/optimized_map");
    const auto output=declare_parameter<std::string>("output_topic","/mid360/map_preview");
    const auto service=declare_parameter<std::string>("save_service","/mid360/save_preview");
    dir_=declare_parameter<std::string>("output_dir","");
    if(dir_.empty()){
      const char* root=std::getenv("LIVOX_MID360_ROOT");
      dir_=std::string(root?root:"/tmp")+"/results/mid360_dashboard/maps";
    }
    voxel_=declare_parameter<double>("voxel_m",0.25);
    max_=declare_parameter<int>("max_points",90000);
    if(!(voxel_>0.02&&voxel_<5.0&&max_>=1000&&max_<=1000000))throw std::invalid_argument("map_preview invalid parameter");
    auto qos=rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    pub_=create_publisher<sensor_msgs::msg::PointCloud2>(output,qos);
    sub_=create_subscription<sensor_msgs::msg::PointCloud2>(input,rclcpp::QoS(1).reliable().transient_local(),
       std::bind(&MapPreview::onMap,this,std::placeholders::_1));
    srv_=create_service<std_srvs::srv::Trigger>(service,
       std::bind(&MapPreview::save,this,std::placeholders::_1,std::placeholders::_2));
    RCLCPP_INFO(get_logger(),"Corrected P4 map %s -> %s; PCD preview export %s",
         input.c_str(),output.c_str(),service.c_str());
  }
private:
  void onMap(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
    if(msg->header.frame_id!="map"){
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,
          "Ignore P4 cloud with frame '%s' (required 'map')",msg->header.frame_id.c_str());
      return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>());
    try {pcl::fromROSMsg(*msg,*input);}
    catch(const std::exception& e){RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),5000,
       "Cloud conversion: %s",e.what());return;}
    if(input->empty())return;
    pcl::VoxelGrid<pcl::PointXYZI> grid;
    grid.setInputCloud(input);
    const auto size=static_cast<float>(voxel_);
    grid.setLeafSize(size,size,size);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    grid.filter(*filtered);
    pcl::PointCloud<pcl::PointXYZI>::Ptr bounded(new pcl::PointCloud<pcl::PointXYZI>());
    const auto stride=std::max<size_t>(1,(filtered->size()+max_-1)/static_cast<size_t>(max_));
    bounded->reserve(std::min<size_t>(filtered->size(),max_));
    for(size_t i=0;i<filtered->size();i+=stride){
      auto pt=filtered->points[i];
      if(std::isfinite(pt.x)&&std::isfinite(pt.y)&&std::isfinite(pt.z))bounded->push_back(pt);
    }
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*bounded,out);
    out.header=msg->header;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_=bounded;
    }
    pub_->publish(out);
  }
  void save(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    pcl::PointCloud<pcl::PointXYZI>::Ptr snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot=latest_;
    }
    if(!snapshot||snapshot->empty()){
      response->success=false;
      response->message="No P4 corrected map received; cannot save.";
      return;
    }
    try{
      std::filesystem::create_directories(dir_);
      auto epoch=std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      const auto path=std::filesystem::path(dir_)/("mid360_preview_"+std::to_string(epoch)+".pcd");
      const auto ret=pcl::io::savePCDFileBinary(path.string(),*snapshot);
      if(ret!=0)throw std::runtime_error("PCL PCD writer returned "+std::to_string(ret));
      response->success=true;
      response->message=path.string()+" ("+std::to_string(snapshot->size())+" points)";
      RCLCPP_INFO(get_logger(),"Saved %s",response->message.c_str());
    }catch(const std::exception& e){
      response->success=false;
      response->message=e.what();
    }
  }
  std::string dir_;
  double voxel_;
  int max_;
  std::mutex mutex_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr latest_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<MapPreview>());}
  catch(const std::exception& e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
