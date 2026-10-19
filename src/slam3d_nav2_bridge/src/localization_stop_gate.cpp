#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

class LocalizationStopGate:public rclcpp::Node {
 public:
  LocalizationStopGate():Node("localization_stop_gate") {
    const auto status_topic=declare_parameter<std::string>(
      "localization_status_topic","/p5/status");
    const auto cmd_topic=declare_parameter<std::string>(
      "raw_command_topic","/cmd_vel");
    const auto safe_topic=declare_parameter<std::string>(
      "safe_command_topic","/cmd_vel_safe");
    const auto odom_topic=declare_parameter<std::string>("base_odom_topic","/odom");
    map_frame_=declare_parameter<std::string>("map_frame","map");
    base_frame_=declare_parameter<std::string>("base_frame","base_link");
    calibration_valid_=declare_parameter<bool>("calibration_valid",false);
    simulation_override_=declare_parameter<bool>("simulation_only_override",false);
    require_tf_=declare_parameter<bool>("require_tf",true);
    require_arm_=declare_parameter<bool>("require_operator_arm",true);
    status_timeout_=declare_parameter<double>("status_timeout_s",2.5);
    odom_timeout_=declare_parameter<double>("odom_timeout_s",0.6);
    cmd_timeout_=declare_parameter<double>("command_timeout_s",0.35);
    tf_age_=declare_parameter<double>("tf_max_age_s",0.5);
    max_linear_=declare_parameter<double>("max_linear_x_mps",0.6);
    max_yaw_=declare_parameter<double>("max_angular_z_radps",0.9);
    rate_=declare_parameter<double>("output_frequency_hz",20.);
    min_tracking_msgs_=declare_parameter<int>("consecutive_tracking_messages",2);
    if(status_timeout_<=0||odom_timeout_<=0||cmd_timeout_<=0||tf_age_<=0
       ||max_linear_<=0||max_yaw_<=0||rate_<5||rate_>100||min_tracking_msgs_<1)
      throw std::runtime_error("Invalid safety gate parameters");
    const auto qos=rclcpp::QoS(1).reliable().transient_local();
    status_pub_=create_publisher<std_msgs::msg::String>("/p6/safety_state",qos);
    safe_pub_=create_publisher<geometry_msgs::msg::Twist>(safe_topic,rclcpp::QoS(10));
    diagnostics_pub_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",10);
    tf_buffer_=std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    nav_pose_client_=rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this,"/navigate_to_pose");
    nav_multi_client_=rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
        this,"/navigate_through_poses");
    status_sub_=create_subscription<std_msgs::msg::String>(status_topic,qos,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        const auto s=m->data;
        const auto first=s.substr(0,s.find_first_of(" |"));
        if(first=="TRACKING") {
          // Count only periodic updates; never accept another stale latched state
          // as sufficient without fresh base odometry and TF.
          consecutive_tracking_=std::min(100,consecutive_tracking_+1);
        } else {
          consecutive_tracking_=0;
          if(arm_)cancelNavigation();
          arm_=false;
        }
        tracking_=(first=="TRACKING");
        localization_text_=first;
        last_status_=std::chrono::steady_clock::now();
        if(!tracking_)publishStop();
      });
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(odom_topic,
      rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        if(m->header.frame_id=="lio_odom"&&m->child_frame_id==base_frame_&&
          std::isfinite(m->pose.pose.position.x)) {
          last_odom_=std::chrono::steady_clock::now();
          odom_seen_=true;
        }
      });
    cmd_sub_=create_subscription<geometry_msgs::msg::Twist>(cmd_topic,10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        if(!std::isfinite(m->linear.x)||!std::isfinite(m->angular.z)
           ||!std::isfinite(m->linear.y)||!std::isfinite(m->linear.z)
           ||!std::isfinite(m->angular.x)||!std::isfinite(m->angular.y)) {
          bad_commands_++;last_cmd_seen_=false;publishStop();return;
        }
        if(std::abs(m->linear.y)>1e-6||std::abs(m->linear.z)>1e-6||
           std::abs(m->angular.x)>1e-6||std::abs(m->angular.y)>1e-6) {
          bad_commands_++;last_cmd_seen_=false;publishStop();return;
        }
        last_cmd_=*m;
        last_cmd_.linear.x=std::clamp(last_cmd_.linear.x,-max_linear_,max_linear_);
        last_cmd_.angular.z=std::clamp(last_cmd_.angular.z,-max_yaw_,max_yaw_);
        last_command_=std::chrono::steady_clock::now();
        last_cmd_seen_=true;
      });
    arm_service_=create_service<std_srvs::srv::SetBool>("/p6/arm",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        if(!req->data) {
          arm_=false;publishStop();res->success=true;res->message="Disarmed";return;
        }
        auto cause=inhibitReason(/*allow_unarmed=*/true);
        if(cause!="READY") {
          arm_=false;res->success=false;res->message="Arm denied: "+cause;publishStop();return;
        }
        arm_=true;res->success=true;res->message="Armed under software guard; not a physical safety certification";
      });
    estop_service_=create_service<std_srvs::srv::SetBool>("/p6/estop",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        if(req->data){estop_=true;cancelNavigation();arm_=false;publishStop();res->success=true;
           res->message="Software E-STOP latched; physical E-STOP remains mandatory";return;}
        // Clearing does not arm. An operator must subsequently explicitly arm.
        estop_=false;arm_=false;publishStop();res->success=true;
        res->message="Software E-STOP reset; still disarmed";
      });
    const auto period=std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0/rate_));
    timer_=create_wall_timer(period,[this](){tick();});
    last_status_=std::chrono::steady_clock::now()-std::chrono::hours(1);
    last_odom_=last_status_;
    last_command_=last_status_;
    publishStop();publishState("STARTUP_LOCKED");
    RCLCPP_WARN(get_logger(),"P6 command gate STARTS DISARMED. %s -> %s. No motor connected by this node.",
      cmd_topic.c_str(),safe_topic.c_str());
  }
 private:
  std::string inhibitReason(bool allow_unarmed=false) {
    const auto t=std::chrono::steady_clock::now();
    if(estop_)return "E_STOP";
    if(!calibration_valid_&&!simulation_override_)return "EXTRINSICS_UNCALIBRATED";
    if(!tracking_)return "LOCALIZATION_"+localization_text_;
    if(std::chrono::duration<double>(t-last_status_).count()>status_timeout_)
      return "STATUS_TIMEOUT";
    if(consecutive_tracking_<min_tracking_msgs_)return "TRACKING_NOT_STABLE";
    if(!odom_seen_||std::chrono::duration<double>(t-last_odom_).count()>odom_timeout_)
      return "ODOMETRY_TIMEOUT";
    if(require_tf_) {
      try {
        const auto tf=tf_buffer_->lookupTransform(map_frame_,base_frame_,tf2::TimePointZero);
        const double age=(now()-rclcpp::Time(tf.header.stamp)).seconds();
        if(!std::isfinite(age)||age>tf_age_||age< -0.1)
          return "TF_STALE";
      } catch(const std::exception &e) {
        return "TF_UNAVAILABLE";
      }
    }
    if(require_arm_&&!arm_&&!allow_unarmed)return "OPERATOR_DISARMED";
    return "READY";
  }
  void cancelNavigation() {
    // Nonblocking best effort. The velocity gate remains safe even when Nav2
    // is offline; this call is NOT a substitute for motor-controller watchdog.
    try {
      if(nav_pose_client_ && nav_pose_client_->action_server_is_ready())
        nav_pose_client_->async_cancel_all_goals();
      if(nav_multi_client_ && nav_multi_client_->action_server_is_ready())
        nav_multi_client_->async_cancel_all_goals();
      ++cancel_events_;
      RCLCPP_ERROR(get_logger(),"P6 requested cancellation of active Nav2 goals");
    }catch(const std::exception &e){
      RCLCPP_ERROR(get_logger(),"Nav2 cancel request failed (zero speed still enforced): %s",e.what());
    }
  }
  void publishStop() {
    safe_pub_->publish(geometry_msgs::msg::Twist());
    zero_published_++;
  }
  void publishState(const std::string &s) {
    if(s==last_state_)return;
    last_state_=s;
    std_msgs::msg::String m;m.data=s;status_pub_->publish(m);
    if(s=="READY")RCLCPP_WARN(get_logger(),"P6 GATE READY; commanded speeds bounded");
    else RCLCPP_WARN(get_logger(),"P6 GATE INHIBITED: %s",s.c_str());
  }
  void tick() {
    std::string reason=inhibitReason();
    if(reason!="READY") {
      if(reason!="OPERATOR_DISARMED") {
        if(reason=="STATUS_TIMEOUT" || reason=="ODOMETRY_TIMEOUT" ||
           reason=="TF_STALE" || reason=="TF_UNAVAILABLE" ||
           reason.rfind("LOCALIZATION_",0)==0) consecutive_tracking_=0;
        if(arm_)cancelNavigation();
        arm_=false;
      }
      publishStop();publishState(reason);
    } else if(!last_cmd_seen_||
       std::chrono::duration<double>(std::chrono::steady_clock::now()-last_command_).count()>cmd_timeout_) {
      arm_=false;
      reason="COMMAND_TIMEOUT";
      publishStop();publishState(reason);
    } else {
      safe_pub_->publish(last_cmd_);
      allowed_published_++;publishState("READY");
    }
    if(++ticks_%static_cast<size_t>(rate_)==0)publishDiagnostics(reason);
  }
  void publishDiagnostics(const std::string &reason) {
    diagnostic_msgs::msg::DiagnosticArray msg;msg.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name="Stage6/Navigation Command Guard";s.hardware_id="software_gate";
    s.level=(reason=="READY")?0:2;s.message=reason;
    auto add=[&](std::string k,std::string v) {
      diagnostic_msgs::msg::KeyValue val;val.key=k;val.value=v;s.values.push_back(val);
    };
    add("armed",arm_?"true":"false");
    add("calibration_valid",calibration_valid_?"true":"false");
    add("simulation_override",simulation_override_?"true":"false");
    add("localization",localization_text_);
    add("consecutive_tracking",std::to_string(consecutive_tracking_));
    add("zero_outputs",std::to_string(zero_published_));
    add("nonzero_eligible_outputs",std::to_string(allowed_published_));
    add("invalid_commands",std::to_string(bad_commands_));
    add("nav2_cancel_requests",std::to_string(cancel_events_));
    msg.status.push_back(s);diagnostics_pub_->publish(msg);
  }
  std::string map_frame_,base_frame_,localization_text_{"UNKNOWN"},last_state_;
  bool calibration_valid_{false},simulation_override_{false},require_tf_{true},
       require_arm_{true},arm_{false},estop_{false},tracking_{false},
       odom_seen_{false},last_cmd_seen_{false};
  int min_tracking_msgs_{2},consecutive_tracking_{0};
  double status_timeout_,odom_timeout_,cmd_timeout_,tf_age_,max_linear_,max_yaw_,rate_;
  size_t zero_published_{0},allowed_published_{0},bad_commands_{0},ticks_{0},
         cancel_events_{0};
  geometry_msgs::msg::Twist last_cmd_;
  std::chrono::steady_clock::time_point last_status_,last_odom_,last_command_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_,estop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_pose_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_multi_client_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<LocalizationStopGate>());}
  catch(const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("localization_stop_gate"),"%s",e.what());
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
