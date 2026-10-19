#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class PcdOccupancyMap : public rclcpp::Node {
 public:
  PcdOccupancyMap():Node("pcd_occupancy_map") {
    path_=declare_parameter<std::string>("map_pcd","");
    frame_=declare_parameter<std::string>("map_frame","map");
    res_=declare_parameter<double>("resolution_m",0.15);
    padding_=declare_parameter<double>("padding_m",2.0);
    floor_=declare_parameter<double>("floor_z_m",0.0);
    tolerance_=declare_parameter<double>("floor_tolerance_m",0.2);
    min_z_=declare_parameter<double>("obstacle_min_z_m",0.35);
    max_z_=declare_parameter<double>("obstacle_max_z_m",2.0);
    min_obstacles_=declare_parameter<int>("min_obstacle_points_per_cell",2);
    max_cells_=declare_parameter<int>("max_map_cells",1500000);
    period_=declare_parameter<double>("publish_period_s",5.0);
    if(path_.empty())throw std::runtime_error("Map PCD must be selected explicitly");
    if(res_<=0||padding_<0||tolerance_<0||min_z_<=floor_+tolerance_||
        max_z_<=min_z_||min_obstacles_<1||max_cells_<100||period_<=0)
      throw std::runtime_error("Invalid occupancy parameters");
    pcl::PointCloud<pcl::PointXYZI> cloud;
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(path_,cloud)<0 ||cloud.size()<100)
      throw std::runtime_error("Invalid PCD map: "+path_);
    double minx=1e30,maxx=-1e30,miny=1e30,maxy=-1e30;
    size_t finite=0;
    for(const auto &p:cloud.points) {
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
      minx=std::min<double>(minx,p.x);maxx=std::max<double>(maxx,p.x);
      miny=std::min<double>(miny,p.y);maxy=std::max<double>(maxy,p.y);++finite;
    }
    if(finite<100)throw std::runtime_error("PCD has too few finite points");
    minx-=padding_;maxx+=padding_;miny-=padding_;maxy+=padding_;
    const size_t width=static_cast<size_t>(std::ceil((maxx-minx)/res_))+1;
    const size_t height=static_cast<size_t>(std::ceil((maxy-miny)/res_))+1;
    if(width*height>static_cast<size_t>(max_cells_)||width>100000||height>100000)
      throw std::runtime_error("Map bounds exceed max_map_cells: check point outliers or resolution");
    auto &info=grid_.info;
    info.resolution=res_;info.width=width;info.height=height;
    info.origin.position.x=minx;info.origin.position.y=miny;info.origin.position.z=0;
    info.origin.orientation.w=1.0;
    grid_.header.frame_id=frame_;
    grid_.data.assign(width*height,-1);
    std::vector<uint16_t> obstacles(width*height,0);
    for(const auto &p:cloud.points){
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
      const auto x=static_cast<int>((p.x-minx)/res_);
      const auto y=static_cast<int>((p.y-miny)/res_);
      if(x<0||y<0||x>=static_cast<int>(width)||y>=static_cast<int>(height))continue;
      const size_t idx=static_cast<size_t>(y)*width+x;
      if(std::abs(p.z-floor_)<=tolerance_ && grid_.data[idx]!=100)
        grid_.data[idx]=0;
      if(p.z>=min_z_ && p.z<=max_z_ && obstacles[idx]<UINT16_MAX)
        ++obstacles[idx];
    }
    for(size_t i=0;i<obstacles.size();++i)
      if(obstacles[i]>=static_cast<uint16_t>(min_obstacles_))grid_.data[i]=100;
    pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("/map",
      rclcpp::QoS(1).reliable().transient_local());
    publish();
    timer_=create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(period_)),[this](){publish();});
    RCLCPP_WARN(get_logger(),
      "P6 3D->2D occupancy %zux%zu resolution %.2f m. Unknown cells stay UNKNOWN; verify floor_z before navigation!",
      width,height,res_);
  }
 private:
  void publish(){
    grid_.header.stamp=now();
    grid_.info.map_load_time=grid_.header.stamp;
    pub_->publish(grid_);
  }
  std::string path_,frame_;
  double res_,padding_,floor_,tolerance_,min_z_,max_z_,period_;
  int min_obstacles_,max_cells_;
  nav_msgs::msg::OccupancyGrid grid_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv){
 rclcpp::init(argc,argv);
 try{rclcpp::spin(std::make_shared<PcdOccupancyMap>());}
 catch(const std::exception &e){
  RCLCPP_FATAL(rclcpp::get_logger("pcd_occupancy_map"),"%s",e.what());
  rclcpp::shutdown();return 1;
 }
 rclcpp::shutdown();return 0;
}
