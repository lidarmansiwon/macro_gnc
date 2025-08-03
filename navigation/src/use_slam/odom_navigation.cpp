#include <iostream>
#include <memory>
// C++ Standard lib 
#include <chrono>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mk3_msgs/msg/navigation_type.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "navigation/odom_navigation.hpp"
#include "navigation/tool/quaternion_utils.hpp"
#include "navigation/tool/velocity_calculator.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

OdomNavigation::OdomNavigation(const rclcpp::NodeOptions & node_options)
: Node("odom_navigation", node_options), 
  type_odom(false),
  enable_imu_(false),
  vel_calc_(0.9)
{

  // Declare parameters
  this->declare_parameter<std::string>("imu_topic", "/imu_topic");
  this->declare_parameter<std::string>("odom_topic", "/odom_topic");
  this->declare_parameter<std::string>("pose_topic", "/pose_topic");
  this->declare_parameter<std::string>("navigation_topic", "/navigation_data");

  this->declare_parameter<bool>("type_odom", false);
  this->declare_parameter<bool>("enable_imu", false);
  this->declare_parameter<double>("velocity_alpha", 0.9);

  // Get parameters
  std::string imu_topic  = this->get_parameter("imu_topic").as_string();
  std::string odom_topic = this->get_parameter("odom_topic").as_string();
  std::string pose_topic = this->get_parameter("pose_topic").as_string();
  std::string navigation_topic = this->get_parameter("navigation_topic").as_string();

  type_odom    = this->get_parameter("type_odom").as_bool();
  enable_imu_  = this->get_parameter("enable_imu").as_bool();
  double velocity_alpha = this->get_parameter("velocity_alpha").as_double();
  vel_calc_ = SixDofVelCalculator(velocity_alpha);

  // 생성자에서 시작 표시.
  RCLCPP_INFO(this->get_logger(), "Run Odom Navigation");

  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(), std::bind(&OdomNavigation::imu_callback, this, _1));

  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, std::bind(&OdomNavigation::odometry_callback, this, _1));

  pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic, 10, std::bind(&OdomNavigation::pose_callback, this, _1));

  navigation_publisher_ = this->create_publisher<mk3_msgs::msg::NavigationType>(
    navigation_topic, 10);
  
  // 50ms --> 20Hz 
  timer_ = this->create_wall_timer(50ms, std::bind(&OdomNavigation::process, this));
}

OdomNavigation::~OdomNavigation()
{

}

void OdomNavigation::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg){odom_data_ = msg;}

void OdomNavigation::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){pose_data_ = msg;}

void OdomNavigation::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){imu_data_ = msg;}


double OdomNavigation::get_sysSec_from_ros_time(const rclcpp::Time& ros_time)
{
  // ROS Time → system_clock 시간으로 변환
  std::chrono::nanoseconds ns_since_epoch(static_cast<int64_t>(ros_time.nanoseconds()));
  auto time_point = std::chrono::time_point<std::chrono::system_clock>(ns_since_epoch);

  // system_clock 기준으로 struct tm 얻기 (localtime)
  std::time_t t_c = std::chrono::system_clock::to_time_t(time_point);
  std::tm local_tm = *std::localtime(&t_c);

  // 마이크로초 추출
  auto duration_today = time_point - std::chrono::system_clock::from_time_t(t_c);
  auto microsec = std::chrono::duration_cast<std::chrono::microseconds>(duration_today).count();

  // 초 단위 시스템 시간 계산
  double sysSec = local_tm.tm_hour * 3600.0 +
                  local_tm.tm_min * 60.0 +
                  local_tm.tm_sec +
                  (microsec / 1e6);

  return sysSec;
}

void OdomNavigation::process()
{

  if (!odom_data_ && !imu_data_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "'Odometry' or 'PoseStamped' data not received yet.");
    return;
  }

  geometry_msgs::msg::Quaternion quat;
  geometry_msgs::msg::Point pos;

  if (!type_odom && pose_data_)
  {
    RCLCPP_INFO_ONCE(this->get_logger(),"DATA TYPE: PoseStamped");
    RCLCPP_INFO_ONCE(this->get_logger(),"NODE START");
    quat = pose_data_->pose.orientation;
    pos = pose_data_->pose.position;
  }
  else if (type_odom && odom_data_)
  {
    RCLCPP_INFO_ONCE(this->get_logger(),"DATA TYPE: Odometry");
    RCLCPP_INFO_ONCE(this->get_logger(),"NODE START");
    quat = odom_data_->pose.pose.orientation;
    pos = odom_data_->pose.pose.position;
  }
  else
  {
    return;
  }
  
std::array<double, 3> rpy = euler_from_quaternion(
  {quat.x, quat.y, quat.z, quat.w}
);
double roll = rpy[0];
double pitch = rpy[1];
double yaw = rpy[2];

double x = pos.x;
double y = pos.y;
double z = pos.z;

double phi = roll;
double theta = pitch;
double psi = yaw;

rclcpp::Time current_time = this->now();
double cal_time = current_time.seconds();
double systime = get_sysSec_from_ros_time(current_time); 

if (!start_time_set_) {
  start_time_sec_ = systime;
  start_time_set_ = true;
}

double time_since_start = systime - start_time_sec_;    // 상대 시간


// 속도 및 각속도 계산
auto result = vel_calc_.update(x, y, z, phi, theta, psi, cal_time);

// 메시지 구성
NavigationType nav_msg;
nav_msg.header.stamp = current_time;
nav_msg.systime = systime;          // 절대 시간
nav_msg.time = time_since_start;    // 상대 시간 (프로그램 시작 후 경과)

nav_msg.x = round(1000 * x) / 1000;
nav_msg.y = round(1000 * y) / 1000;
nav_msg.z = round(1000 * z) / 1000;

nav_msg.psi = round(1000 * (psi * 180.0 / M_PI)) / 1000;
nav_msg.theta = round(1000 * (theta * 180.0 / M_PI)) / 1000;
nav_msg.phi = round(1000 * (phi * 180.0 / M_PI)) / 1000;

// 속도 (LPF 적용된 값)
nav_msg.u = round(1000 * (result.LPFVel_x)) / 1000;
nav_msg.v = round(1000 * (result.LPFVel_y)) / 1000;
nav_msg.w = round(1000 * (result.LPFVel_z)) / 1000;

// 총 속도 크기 (optional)
nav_msg.t_vel = round(1000 * (std::sqrt(result.LPFVel_x * result.LPFVel_x +
                          result.LPFVel_y * result.LPFVel_y +
                          result.LPFVel_z * result.LPFVel_z))) / 1000;

// 각속도 (LPF 적용된 값)
nav_msg.p = round(1000 * (result.LPFAngVel_p)) / 1000;
nav_msg.q = round(1000 * (result.LPFAngVel_q)) / 1000;
nav_msg.r = round(1000 * (result.LPFAngVel_r)) / 1000;

navigation_publisher_->publish(nav_msg);

}