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
  vel_calc_ = VelocityCalculator(velocity_alpha);

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
  double psi = yaw;

  rclcpp::Time current_time = this->now();
  double time_sec = current_time.seconds();

  auto result = vel_calc_.update(x, y, psi, time_sec);
  double LPFVel_x = result.LPFVel_x;
  double LPFVel_y = result.LPFVel_y;
  double angular_velocity = result.angular_velocity;

  double imu_angular_x = 0.0, imu_angular_y = 0.0, imu_angular_z = 0.0;
  if (enable_imu_ && imu_data_)
  {
    RCLCPP_INFO_ONCE(this->get_logger(),"IMU Enable!");
    imu_angular_x = imu_data_->angular_velocity.x;
    imu_angular_y = imu_data_->angular_velocity.y;
    imu_angular_z = imu_data_->angular_velocity.z;
  }

  // 헤더 파일에서 using으로 정의함.
  NavigationType nav_msg;
  nav_msg.x = x;
  nav_msg.y = y;
  nav_msg.psi = psi * 180.0 / M_PI;
  nav_msg.u = LPFVel_x;
  nav_msg.v = LPFVel_y;
  nav_msg.r = angular_velocity;
  nav_msg.w = imu_angular_x;

  navigation_publisher_->publish(nav_msg);
}