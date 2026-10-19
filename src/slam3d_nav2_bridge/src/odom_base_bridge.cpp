#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using Matrix=Eigen::Matrix4d;
static geometry_msgs::msg::Pose rosPose(const Matrix &t) {
  geometry_msgs::msg::Pose p;
  const auto xyz=t.block<3,1>(0,3);
  const Eigen::Quaterniond q(t.block<3,3>(0,0));
  p.position.x=xyz.x();p.position.y=xyz.y();p.position.z=xyz.z();
  p.orientation.x=q.x();p.orientation.y=q.y();
  p.orientation.z=q.z();p.orientation.w=q.w();
  return p;
}
static Matrix poseMatrix(const geometry_msgs::msg::Pose &p) {
  const Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
  if (!std::isfinite(q.norm())||q.norm()<0.9||!std::isfinite(p.position.x)
       ||!std::isfinite(p.position.y)||!std::isfinite(p.position.z))
    throw std::runtime_error("invalid P3 odometry pose");
  Matrix result=Matrix::Identity();
  result.block<3,3>(0,0)=q.normalized().toRotationMatrix();
  result(0,3)=p.position.x;result(1,3)=p.position.y;result(2,3)=p.position.z;
  return result;
}
class OdomBaseBridge : public rclcpp::Node {
 public:
  OdomBaseBridge():Node("odom_base_bridge") {
    const auto source=declare_parameter<std::string>("source_odom","/lio/odometry");
    const auto output=declare_parameter<std::string>("base_odom","/odom");
    odom_frame_=declare_parameter<std::string>("odom_frame","lio_odom");
    base_frame_=declare_parameter<std::string>("base_frame","base_link");
    sensor_frame_=declare_parameter<std::string>("sensor_frame","livox_frame");
    publish_tf_=declare_parameter<bool>("publish_tf",true);
    publish_static_=declare_parameter<bool>("publish_sensor_static_tf",true);
    const auto xyz=declare_parameter<std::vector<double>>("sensor_xyz_m",{0.,0.,0.});
    const auto rpy=declare_parameter<std::vector<double>>("sensor_rpy_rad",{0.,0.,0.});
    if(xyz.size()!=3||rpy.size()!=3)throw std::runtime_error("extrinsic xyz/rpy must have 3 elements");
    for(double v:xyz)if(!std::isfinite(v))throw std::runtime_error("invalid sensor_xyz_m");
    for(double v:rpy)if(!std::isfinite(v))throw std::runtime_error("invalid sensor_rpy_rad");
    if(base_frame_==sensor_frame_||odom_frame_==base_frame_)
      throw std::runtime_error("Frame names must be unique");
    const Eigen::Matrix3d R=(Eigen::AngleAxisd(rpy[2],Eigen::Vector3d::UnitZ())*
                             Eigen::AngleAxisd(rpy[1],Eigen::Vector3d::UnitY())*
                             Eigen::AngleAxisd(rpy[0],Eigen::Vector3d::UnitX())).toRotationMatrix();
    base_to_sensor_=Matrix::Identity();
    base_to_sensor_.block<3,3>(0,0)=R;
    base_to_sensor_.block<3,1>(0,3)=Eigen::Vector3d(xyz[0],xyz[1],xyz[2]);
    publisher_=create_publisher<nav_msgs::msg::Odometry>(output,rclcpp::SensorDataQoS());
    broadcaster_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_broadcaster_=std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    if(publish_static_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp=now();
      tf.header.frame_id=base_frame_;tf.child_frame_id=sensor_frame_;
      const auto p=rosPose(base_to_sensor_);
      tf.transform.translation.x=p.position.x;tf.transform.translation.y=p.position.y;
      tf.transform.translation.z=p.position.z;tf.transform.rotation=p.orientation;
      static_broadcaster_->sendTransform(tf);
    }
    sub_=create_subscription<nav_msgs::msg::Odometry>(source,rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        if(m->header.frame_id!=odom_frame_||m->child_frame_id!=sensor_frame_) {
          RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),4000,
            "Unexpected P3 odometry frames: %s -> %s",
            m->header.frame_id.c_str(),m->child_frame_id.c_str());return;
        }
        try {
          const auto stamp=rclcpp::Time(m->header.stamp);
          if(stamp.nanoseconds()==last_stamp_)return;
          if(stamp.nanoseconds()<last_stamp_)last_stamp_=0;
          const Matrix base=poseMatrix(m->pose.pose)*base_to_sensor_.inverse();
          nav_msgs::msg::Odometry result=*m;
          result.header.frame_id=odom_frame_;result.child_frame_id=base_frame_;
          result.pose.pose=rosPose(base);
          const Eigen::Vector3d lever=base.block<3,1>(0,3)-poseMatrix(m->pose.pose).block<3,1>(0,3);
          Eigen::Matrix3d skew;
          skew << 0,-lever.z(),lever.y(),lever.z(),0,-lever.x(),-lever.y(),lever.x(),0;
          Eigen::Matrix<double,6,6> J=Eigen::Matrix<double,6,6>::Identity(),covariance;
          J.block<3,3>(0,3)=-skew;
          for(int i=0;i<6;++i)for(int j=0;j<6;++j)covariance(i,j)=m->pose.covariance[i*6+j];
          covariance=(J*covariance*J.transpose()).eval();
          for(int i=0;i<6;++i)for(int j=0;j<6;++j)result.pose.covariance[i*6+j]=covariance(i,j);
          std::fill(result.twist.covariance.begin(),result.twist.covariance.end(),0.0);
          result.twist.twist=geometry_msgs::msg::Twist();
          if(last_stamp_>0) {
            const double dt=(stamp.nanoseconds()-last_stamp_)*1e-9;
            if(dt>0.001&&dt<0.5) {
              const Matrix delta=last_base_.inverse()*base;
              const auto v=delta.block<3,1>(0,3)/dt;
              result.twist.twist.linear.x=v.x();
              result.twist.twist.linear.y=v.y();
              result.twist.twist.linear.z=v.z();
              const Eigen::AngleAxisd aa(delta.block<3,3>(0,0));
              const Eigen::Vector3d omega=aa.axis()*aa.angle()/dt;
              result.twist.twist.angular.x=omega.x();
              result.twist.twist.angular.y=omega.y();
              result.twist.twist.angular.z=omega.z();
            }
          }
          // Do not claim sensor-only finite differencing is calibrated wheel odometry.
          for(int i=0;i<6;++i)result.twist.covariance[i*6+i]=0.1;
          publisher_->publish(result);
          if(publish_tf_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header=m->header;tf.header.frame_id=odom_frame_;
            tf.child_frame_id=base_frame_;
            tf.transform.translation.x=result.pose.pose.position.x;
            tf.transform.translation.y=result.pose.pose.position.y;
            tf.transform.translation.z=result.pose.pose.position.z;
            tf.transform.rotation=result.pose.pose.orientation;
            broadcaster_->sendTransform(tf);
          }
          last_base_=base;last_stamp_=stamp.nanoseconds();
        } catch(const std::exception &e) {
          RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),4000,"TF bridge rejected odom: %s",e.what());
        }
      });
    RCLCPP_WARN(get_logger(),"P6 requires FAST-LIO publish_odom_tf:=false. Publishing %s -> %s, %s -> %s",
      odom_frame_.c_str(),base_frame_.c_str(),base_frame_.c_str(),sensor_frame_.c_str());
  }
 private:
  std::string odom_frame_,base_frame_,sensor_frame_;
  bool publish_tf_{true},publish_static_{true};
  Matrix base_to_sensor_{Matrix::Identity()},last_base_{Matrix::Identity()};
  int64_t last_stamp_{0};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<OdomBaseBridge>());}
  catch(const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("odom_base_bridge"),"%s",e.what());
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
