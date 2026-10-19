#include <cstring>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
#include <cmath>
#include <string>
#include <algorithm>
#include <vector>
#include <utility>

namespace asio=boost::asio;
namespace beast=boost::beast;
namespace http=beast::http;
using tcp=asio::ip::tcp;
using Json=nlohmann::json;
using Clock=std::chrono::steady_clock;

class WebDashboard final : public rclcpp::Node {
public:
  WebDashboard():Node("web_dashboard"),ioc_(1),acceptor_(ioc_){
    const auto addr=declare_parameter<std::string>("bind_address","127.0.0.1");
    const int port=declare_parameter<int>("port",5001);
    max_points_=declare_parameter<int>("max_web_points",3500);
    stale_s_=declare_parameter<double>("stale_after_s",3.0);
    if(addr!="127.0.0.1"&&addr!="::1")throw std::invalid_argument("HTTP dashboard is localhost-only");
    if(port<1024||port>65535||max_points_<100||max_points_>10000||stale_s_<=0)
      throw std::invalid_argument("Invalid port/web-point count/stale threshold");
    std::ifstream f(ament_index_cpp::get_package_share_directory("slam3d_mid360_dashboard")+"/web/index.html");
    if(!f)throw std::runtime_error("Missing installed web/index.html");
    std::stringstream buffer;buffer<<f.rdbuf();html_=buffer.str();
    // Never call a map preview a physically calibrated map without evidence.
    const char *workspace=std::getenv("LIVOX_MID360_ROOT");
    const std::string extrinsic_file=std::string(workspace?workspace:"/tmp")+"/config/extrinsics.yaml";
    {
      std::ifstream calib(extrinsic_file);
      std::string line;
      extrinsic_status_="missing";
      while(std::getline(calib,line)) {
        const auto pos=line.find("status:");
        if(pos!=std::string::npos) {
          extrinsic_status_=line.substr(pos+7);
          extrinsic_status_.erase(0,extrinsic_status_.find_first_not_of(" \t\r"));
          break;
        }
      }
    }
    map_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("preview_topic","/mid360/map_preview"),
      rclcpp::QoS(1).reliable().transient_local(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){onMap(msg);});
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("cloud_topic","/livox/lidar"),rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
        bool sample=false;
        {
          std::lock_guard<std::mutex> l(mu_);
          last_cloud_=Clock::now(); cloud_count_++;cloud_points_=static_cast<uint64_t>(msg->width)*msg->height;
          cloud_frame_=msg->header.frame_id;
          sample=(last_live_preview_==Clock::time_point{} ||
                  age(last_live_preview_)>0.09);
          if(sample)last_live_preview_=Clock::now();
        }
        if(sample) {
          auto sampled=sampleCloud(msg,max_points_);
          std::lock_guard<std::mutex> l(mu_);
          live_points_=std::move(sampled); ++live_sequence_;
        }
      });
    lio_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lio/cloud_registered",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        if(msg->header.frame_id!="lio_odom")return;
        {
          std::lock_guard<std::mutex> l(mu_);
          if(last_lio_preview_!=Clock::time_point{}&&age(last_lio_preview_)<0.09)return;
          last_lio_preview_=Clock::now();
        }
        auto sampled=sampleCloud(msg,max_points_);
        std::lock_guard<std::mutex> l(mu_);
        lio_points_=std::move(sampled); ++lio_sequence_;
      });
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("imu_topic","/livox/imu"),rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m){
        const auto &a=m->linear_acceleration;const auto &g=m->angular_velocity;
        const double an=std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z)/9.80665;
        const double gn=std::sqrt(g.x*g.x+g.y*g.y+g.z*g.z);
        std::lock_guard<std::mutex> l(mu_);last_imu_=Clock::now();imu_count_++;
        if(std::isfinite(an)&&std::isfinite(gn)) {
          imu_accel_g_=imu_count_>1?(0.98*imu_accel_g_+0.02*an):an;
          imu_gyro_rads_=imu_count_>1?(0.98*imu_gyro_rads_+0.02*gn):gn;
        }
      });
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("odom_topic","/p4/optimized_odometry"),rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){
        std::lock_guard<std::mutex> l(mu_);
        last_odom_=Clock::now();
        position_={m->pose.pose.position.x,m->pose.pose.position.y,m->pose.pose.position.z};
      });
    graph_sub_=create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics",rclcpp::QoS(10),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m){
        for(const auto& status:m->status){
          if(status.name!="Stage4/Pose Graph")continue;
          auto get=[&](const char *key)->size_t{
            for(const auto &v:status.values)
              if(v.key==key){try{return static_cast<size_t>(std::stoull(v.value));}
                catch(const std::exception&){return 0;}}
            return 0;
          };
          std::lock_guard<std::mutex> l(mu_);
          p4_keyframes_=get("keyframes");
          p4_loops_=get("loop_accepted");
          p4_rejected_=get("loop_rejected");
          last_p4_diag_=Clock::now();
        }
      });
    save_backend_=create_client<std_srvs::srv::Trigger>("/p4/save_map");
    save_preview_=create_client<std_srvs::srv::Trigger>("/mid360/save_preview");
    const auto ip=asio::ip::make_address(addr);
    acceptor_.open(ip.is_v4()?tcp::v4():tcp::v6());
    acceptor_.set_option(asio::socket_base::reuse_address(true));
    acceptor_.bind(tcp::endpoint(ip,static_cast<unsigned short>(port)));
    acceptor_.listen(asio::socket_base::max_listen_connections);
    accept();
    server_thread_=std::thread([this](){ioc_.run();});
    RCLCPP_INFO(get_logger(),"Dashboard ready at http://%s:%d (loopback only)",addr.c_str(),port);
  }
  ~WebDashboard()override {
    ioc_.stop();
    if(server_thread_.joinable())server_thread_.join();
  }
private:
  static double age(Clock::time_point t){
    if(t==Clock::time_point{})return -1;
    return std::chrono::duration<double>(Clock::now()-t).count();
  }
  static Json sampleCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,int limit){
    const auto off=[&](const char*name)->int {
      for(const auto&f:msg->fields)if(f.name==name&&f.datatype==sensor_msgs::msg::PointField::FLOAT32)
        return static_cast<int>(f.offset);
      return -1;
    };
    const int x=off("x"),y=off("y"),z=off("z"),intens=off("intensity");
    Json pts=Json::array();
    if(x<0||y<0||z<0||msg->point_step==0||msg->width==0)return pts;
    const size_t total=static_cast<size_t>(msg->width)*msg->height;
    const size_t stride=std::max<size_t>(1,(total+limit-1)/limit);
    for(size_t i=0;i<total;i+=stride){
      const size_t row=i/msg->width,col=i%msg->width;
      const size_t at=row*msg->row_step+col*msg->point_step;
      if(at+msg->point_step>msg->data.size())break;
      const auto get=[&](int k)->float{
        float out=0;
        if(k>=0&&static_cast<size_t>(k+4)<=msg->point_step)
          std::memcpy(&out,msg->data.data()+at+k,4);
        return out;
      };
      const auto px=get(x),py=get(y),pz=get(z),v=get(intens);
      if(std::isfinite(px)&&std::isfinite(py)&&std::isfinite(pz))
        pts.push_back(Json::array({px,py,pz,std::isfinite(v)?v:0.f}));
    }
    return pts;
  }
  void onMap(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
    if(msg->header.frame_id!="map")return;
    auto sampled=sampleCloud(msg,max_points_);
    std::lock_guard<std::mutex> l(mu_);
    points_=std::move(sampled); ++map_sequence_;
    map_total_points_=static_cast<size_t>(msg->width)*msg->height;
    last_map_=Clock::now();
  }
  Json getStatus(){
    std::lock_guard<std::mutex> l(mu_);
    const double ca=age(last_cloud_),ia=age(last_imu_),ma=age(last_map_),oa=age(last_odom_);
    Json j={
      {"online",true},{"sensor_online",ca>=0&&ca<stale_s_},
      {"imu_online",ia>=0&&ia<stale_s_},{"map_available",ma>=0&&ma<30.0},
      {"cloud_age_s",ca},{"imu_age_s",ia},{"map_age_s",ma},{"odom_age_s",oa},
      {"cloud_messages",cloud_count_},{"imu_messages",imu_count_},
      {"cloud_points_last",cloud_points_},{"map_points",map_total_points_},
      {"web_points",points_.size()},{"cloud_frame",cloud_frame_},
      {"map_frame","map"},{"position",position_},
      {"save_result",save_result_},{"mapping_note","P4 map must be validated, do not equate visualization with precision"},
      {"extrinsics_status",extrinsic_status_},{"imu_accel_norm_g",imu_accel_g_},
      {"imu_gyro_norm_rad_s",imu_gyro_rads_},
      {"lio_preview_points",lio_points_.size()},{"live_preview_points",live_points_.size()},
      {"keyframes",p4_keyframes_},{"loop_closures",p4_loops_},
      {"loop_rejected",p4_rejected_},
      {"mapping_quality",p4_keyframes_==0?"no_keyframes":
        p4_loops_==0?"unclosed_preview":
        extrinsic_status_=="verified"||extrinsic_status_=="calibrated"?
          "candidate_loop_closed_calibrated":"candidate_loop_closed_uncalibrated"},
      {"p4_diag_age_s",age(last_p4_diag_)}
    };
    return j;
  }
  void saveVia(const std::string& target){
    auto client=target=="backend"?save_backend_:save_preview_;
    {
      std::lock_guard<std::mutex> l(mu_);save_result_="requesting "+target+"...";
    }
    auto req=std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(req,[this,target](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f){
      std::lock_guard<std::mutex> l(mu_);
      try{
        const auto ans=f.get();
        save_result_=(ans->success?"SAVED: ":"ERROR: ")+ans->message;
      }catch(const std::exception& e){save_result_=std::string("ERROR: ")+e.what();}
    });
  }
  struct Session {
    explicit Session(tcp::socket socket):stream(std::move(socket)) { parser.body_limit(1024); }
    beast::tcp_stream stream;
    beast::flat_buffer buffer;
    http::request_parser<http::string_body> parser;
    http::response<http::string_body> response;
  };
  void accept(){
    acceptor_.async_accept([this](beast::error_code ec,tcp::socket sock){
      if(!ec && active_sessions_<24){
        ++active_sessions_;
        auto session=std::make_shared<Session>(std::move(sock));
        session->stream.expires_after(std::chrono::seconds(3));
        http::async_read(session->stream,session->buffer,session->parser,
          [this,session](beast::error_code read_ec,std::size_t){
            if(read_ec){--active_sessions_;return;}
            try { session->response=respond(session->parser.release()); }
            catch(const std::exception& e){
              RCLCPP_WARN(get_logger(),"HTTP request failed: %s",e.what());
              --active_sessions_;return;
            }
            session->stream.expires_after(std::chrono::seconds(3));
            http::async_write(session->stream,session->response,
              [this,session](beast::error_code,std::size_t){
                beast::error_code close_ec;
                session->stream.socket().shutdown(tcp::socket::shutdown_both,close_ec);
                --active_sessions_;
              });
          });
      }
      if(acceptor_.is_open())accept();
    });
  }
  http::response<http::string_body> respond(const http::request<http::string_body>& request){
    http::response<http::string_body> response{http::status::ok,request.version()};
    response.set(http::field::server,"mid360s-ros2-cpp");
    response.set(http::field::cache_control,"no-store");
    response.keep_alive(false);
    auto route=std::string(request.target());
    if(request.method()==http::verb::get&&route=="/favicon.ico"){
      response.result(http::status::no_content);
      response.body()="";
    } else if((request.method()==http::verb::get||request.method()==http::verb::head)
              &&(route=="/"||route=="/index.html")){
      response.set(http::field::content_type,"text/html; charset=utf-8");
      response.body()=request.method()==http::verb::head?"":html_;
    } else if(request.method()==http::verb::get&&route=="/api/status"){
      response.set(http::field::content_type,"application/json");response.body()=getStatus().dump();
    } else if(request.method()==http::verb::get&&
             (route=="/api/points"||route.rfind("/api/points?source=",0)==0)){
      const auto source=route=="/api/points"?"p4":route.substr(std::string("/api/points?source=").size());
      Json j;
      {
        std::lock_guard<std::mutex> l(mu_);
        if(source=="live")j={{"points",live_points_},{"count",cloud_points_},{"frame","livox_frame"},{"source","live"},{"sequence",live_sequence_}};
        else if(source=="lio")j={{"points",lio_points_},{"count",lio_points_.size()},{"frame","lio_odom"},{"source","lio"},{"sequence",lio_sequence_}};
        else if(source=="p4")j={{"points",points_},{"count",map_total_points_},{"frame","map"},{"source","p4"},{"sequence",map_sequence_}};
        else j={{"error","Invalid source"}};
      }
      if(source!="live"&&source!="lio"&&source!="p4")response.result(http::status::bad_request);
      response.set(http::field::content_type,"application/json");response.body()=j.dump();
    } else if(request.method()==http::verb::get&&route=="/api/health"){
      response.set(http::field::content_type,"application/json");
      response.body()=Json({{"ok",true},{"service","mid360-dashboard"}}).dump();
    } else if(request.method()==http::verb::post&&(route=="/api/save"||route=="/api/save_preview")){
      const bool backend=(route=="/api/save");
      auto client=backend?save_backend_:save_preview_;
      response.set(http::field::content_type,"application/json");
      if(!client->service_is_ready()){
        response.result(http::status::service_unavailable);
        response.body()=Json({{"accepted",false},{"message","Mapping service unavailable"}}).dump();
      }else{
        saveVia(backend?"backend":"preview");
        response.result(http::status::accepted);
        response.body()=Json({{"accepted",true},{"message","Saving asynchronously; check /api/status"}}).dump();
      }
    }else{
      response.result(http::status::not_found);
      response.set(http::field::content_type,"application/json");
      response.body()=Json({{"error","Not found"}}).dump();
    }
    response.prepare_payload();
    return response;
  }
  std::mutex mu_;
  Json points_=Json::array(),live_points_=Json::array(),lio_points_=Json::array();
  std::string extrinsic_status_="uncalibrated";
  double imu_accel_g_{0.0},imu_gyro_rads_{0.0};
  size_t p4_keyframes_{0},p4_loops_{0},p4_rejected_{0};
  Clock::time_point last_p4_diag_{};
  Clock::time_point last_live_preview_{},last_lio_preview_{};
  size_t active_sessions_{0};
  uint64_t live_sequence_{0},lio_sequence_{0},map_sequence_{0};
  Json position_=Json::array({0.0,0.0,0.0});
  size_t map_total_points_=0;
  uint64_t cloud_count_=0,imu_count_=0,cloud_points_=0;
  std::string cloud_frame_="";std::string save_result_="none";
  Clock::time_point last_cloud_{},last_imu_{},last_map_{},last_odom_{};
  int max_points_;double stale_s_;
  asio::io_context ioc_;
  tcp::acceptor acceptor_;
  std::thread server_thread_;
  std::string html_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_,cloud_sub_,lio_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr graph_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_backend_,save_preview_;
};
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<WebDashboard>());}
  catch(const std::exception& e){std::cerr<<"Dashboard fatal: "<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
