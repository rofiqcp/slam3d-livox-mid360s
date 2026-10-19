#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <utility>

constexpr double PI=3.14159265358979323846;
struct World { float x,y,z,intensity; };
struct XYZIntensity { float x,y,z,intensity; };

class LoopFixture : public rclcpp::Node {
 public:
  LoopFixture() : Node("p4_loop_fixture") {
    hz_=declare_parameter<double>("rate_hz",20.0);
    frames_=declare_parameter<int>("frames",180);
    drift_=declare_parameter<double>("total_translation_drift_m",0.65);
    cloud_pub_=create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lio/cloud_registered",rclcpp::SensorDataQoS());
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>(
      "/lio/odometry",rclcpp::SensorDataQoS());
    gt_pub_=create_publisher<nav_msgs::msg::Odometry>(
      "/p4/fixture/ground_truth",rclcpp::QoS(50));
    for (float x=-13;x<=13;x+=0.65f)
      for (float y=-13;y<=13;y+=0.65f)
        add(x,y,0,35.f);
    for (float x=-13;x<=13;x+=0.55f)
      for (float z=0.3f;z<=4.0f;z+=0.35f) {
        add(x,-13,z,105);
        add(x,13,z,120);
      }
    for (float y=-13;y<=13;y+=0.55f)
      for (float z=0.3f;z<=4.0f;z+=0.35f) {
        add(-13,y,z,140);
        add(13,y,z,150);
      }
    const std::vector<std::pair<float,float>> posts={
      {-7,2},{-4,8},{1,9},{6,8},{9,0},{6,-7},{0,-9},{-8,-6}};
    for(size_t k=0;k<posts.size();++k)
      for(float z=0;z<4;z+=0.18f)
        for(int a=0;a<9;++a) {
          double th=2*PI*a/9;
          add(posts[k].first+0.17*std::cos(th),
              posts[k].second+0.17*std::sin(th),z,190+k*5);
        }
    timer_=create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0/hz_)),
      std::bind(&LoopFixture::tick,this));
    RCLCPP_INFO(get_logger(),"P4 loop fixture: %zu world pts %d scans %.1f Hz drift %.2f m",
      world_.size(),frames_,hz_,drift_);
  }
 private:
  void add(float x,float y,float z,float i){world_.push_back({x,y,z,i});}
  void tick() {
    if (frame_>=frames_) return;
    const double fraction=static_cast<double>(frame_)/static_cast<double>(frames_-1);
    const double theta=2*PI*fraction;
    const double tx=4.5*std::cos(theta),ty=4.5*std::sin(theta),tz=1.2;
    const double true_yaw=theta+PI/2;
    const double raw_x=tx+drift_*fraction, raw_y=ty-0.25*fraction;
    const double raw_yaw=true_yaw+0.07*fraction;
    const auto stamp=now();
    nav_msgs::msg::Odometry odom,gt;
    odom.header.stamp=stamp;
    odom.header.frame_id="lio_odom";
    odom.child_frame_id="livox_frame";
    odom.pose.pose.position.x=raw_x;
    odom.pose.pose.position.y=raw_y;
    odom.pose.pose.position.z=tz;
    odom.pose.pose.orientation.z=std::sin(raw_yaw/2);
    odom.pose.pose.orientation.w=std::cos(raw_yaw/2);
    gt=odom;
    gt.header.frame_id="fixture_world";
    gt.pose.pose.position.x=tx;
    gt.pose.pose.position.y=ty;
    gt.pose.pose.orientation.z=std::sin(true_yaw/2);
    gt.pose.pose.orientation.w=std::cos(true_yaw/2);

    const double c=std::cos(raw_yaw-true_yaw),s=std::sin(raw_yaw-true_yaw);
    std::vector<XYZIntensity> points;
    points.reserve(world_.size());
    for (const auto &p:world_) {
      const double dx=p.x-tx,dy=p.y-ty,dz=p.z-tz;
      if (dx*dx+dy*dy+dz*dz>18*18) continue;
      XYZIntensity result;
      result.x=raw_x+c*dx-s*dy;
      result.y=raw_y+s*dx+c*dy;
      result.z=p.z;
      result.intensity=p.intensity;
      points.push_back(result);
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp=stamp;
    cloud.header.frame_id="lio_odom";
    cloud.height=1;cloud.width=points.size();
    cloud.is_bigendian=false;cloud.is_dense=true;
    cloud.point_step=sizeof(XYZIntensity);
    cloud.row_step=cloud.point_step*cloud.width;
    for(int i=0;i<4;++i) {
      sensor_msgs::msg::PointField field;
      field.name=std::vector<std::string>{"x","y","z","intensity"}[i];
      field.offset=i*4;
      field.datatype=sensor_msgs::msg::PointField::FLOAT32;
      field.count=1;
      cloud.fields.push_back(field);
    }
    cloud.data.resize(cloud.row_step);
    std::memcpy(cloud.data.data(),points.data(),cloud.data.size());
    odom_pub_->publish(odom);
    cloud_pub_->publish(cloud);
    gt_pub_->publish(gt);
    ++frame_;
    if(frame_==frames_) RCLCPP_INFO(get_logger(),"Loop fixture complete, drift at closure %.3f m",drift_);
  }
  double hz_{20},drift_{0.65};
  int frame_{0},frames_{180};
  std::vector<World> world_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_,gt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<LoopFixture>());
  rclcpp::shutdown();
  return 0;
}
