#include <iostream>
#include <memory>
// C++ Standard lib 
#include <chrono>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include <GeographicLib/UTMUPS.hpp>

#include "std_msgs/msg/string.hpp"
#include "mk3_msgs/msg/navigation_type.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "navigation/gps_navigation.hpp"
#include "navigation/tool/quaternion_utils.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

GPSNavigation::GPSNavigation(const rclcpp::NodeOptions & node_options)
: Node("gps_navigation", node_options), 
  prev_pose_initialized_(false),
  reference_gps_latitude_(0.0),
  reference_gps_longitude_(0.0),
  reference_utm_x(0.0),
  reference_utm_y(0.0),
  reference_zone(0.0),
  reference_northp(false),
  LPFVel_x_(0.0),
  LPFVel_y_(0.0)
{
  this->declare_parameter<std::string>("imu_topic", "/imu_topic");
  this->declare_parameter<std::string>("gps_topic", "/gps_topic");
  this->declare_parameter<std::string>("navigation_topic", "/navigation_data");

  this->declare_parameter<double>("reference_gps_latitude", 0.0);
  this->declare_parameter<double>("reference_gps_longitude", 0.0);
  this->declare_parameter<double>("LPFVel_x", 0.0);
  this->declare_parameter<double>("LPFVel_y", 0.0);

  std::string imu_topic = this->get_parameter("imu_topic").as_string();
  std::string gps_topic = this->get_parameter("gps_topic").as_string();
  std::string navigation_topic = this->get_parameter("navigation_topic").as_string();

  reference_gps_latitude_  = this->get_parameter("reference_gps_latitude").as_double();
  reference_gps_longitude_ = this->get_parameter("reference_gps_longitude").as_double();
  LPFVel_x_    = this->get_parameter("LPFVel_x").as_double();
  LPFVel_y_    = this->get_parameter("LPFVel_y").as_double();

  // 생성자에서 시작 표시.
  RCLCPP_INFO(this->get_logger(), "Run GPS Navigation");

  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(), std::bind(&GPSNavigation::imu_callback, this, _1));

  gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_topic, rclcpp::SensorDataQoS(), std::bind(&GPSNavigation::gps_callback, this, _1));

  navigation_publisher_ = this->create_publisher<mk3_msgs::msg::NavigationType>(
    navigation_topic, 10);

  timer_ = this->create_wall_timer(10ms, std::bind(&GPSNavigation::process, this));
}

GPSNavigation::~GPSNavigation()
{

}

void GPSNavigation::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){imu_data_ = msg;}
void GPSNavigation::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){gps_data_ = msg;}

void GPSNavigation::process()
{
  if (!gps_data_ || !imu_data_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "GPS or IMU data not received yet.");
    return;
  }

  double latitude = gps_data_->latitude;
  double longitude = gps_data_->longitude;

  double utm_x, utm_y;
  int zone;
  bool northp;

  try {
    // 입력: latitude, longitude, 출력: zone(UTM 존), northp(북반구, 남반구 여부), utm_x, utm_y
    GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_x, utm_y);
    RCLCPP_INFO(this->get_logger(), "Converted GPS (%.8f, %.8f) → UTM (%.2f, %.2f), Zone: %d, Northp: %d\n", latitude, longitude, utm_x, utm_y, zone, northp);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "UTM conversion failed: %s", e.what());
    return;
  }

  if (!origin_set_) {

    try {
        GeographicLib::UTMUPS::Forward(reference_gps_latitude_, reference_gps_longitude_, reference_zone, reference_northp, reference_utm_x, reference_utm_y);
        RCLCPP_INFO(this->get_logger(), "Converted Reference GPS (%.8f, %.8f) → Reference UTM (%.2f, %.2f), Zone: %d, Northp: %d\n", 
        reference_gps_latitude_, reference_gps_longitude_, reference_utm_x, reference_utm_y, reference_zone, reference_northp);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Reference UTM conversion failed: %s", e.what());
        return;
    }
    origin_set_ = true;
  }

  double relative_x = utm_x - reference_utm_x;
  double relative_y = utm_y - reference_utm_y;


  RCLCPP_INFO(this->get_logger(), "Relative position: x=%.2f, y=%.2f\n\n", relative_x, relative_y);

  geometry_msgs::msg::Quaternion quat;
  rclcpp::Time current_time;

  quat = imu_data_->orientation;

  std::array<double, 3> rpy = euler_from_quaternion(
    {quat.x, quat.y, quat.z, quat.w}
  );

  double roll = rpy[0];
  double pitch = rpy[1];
  double yaw = rpy[2];

  double x = relative_x;
  double y = relative_y;
  double psi = yaw;

  if (!prev_pose_initialized_)
  {
    prev_x_ = x;
    prev_y_ = y;
    prev_psi_ = psi;
    prev_time_ = current_time;
    prev_pose_initialized_ = true;
    return;
  }

  double dt = 0.05;
  if (dt <= 0) return;

  double dx = x - prev_x_;
  double dy = y - prev_y_;
  double dpsi = psi - prev_psi_;

  double vx_fixed = dx / dt;
  double vy_fixed = dy / dt;

  double cos_psi = std::cos(psi);
  double sin_psi = std::sin(psi);

  double vx = vx_fixed * cos_psi + vy_fixed * sin_psi;
  double vy = -vx_fixed * sin_psi + vy_fixed * cos_psi;

  double angular_velocity = dpsi / dt;

  double alpha = 0.9;
  LPFVel_x_ = alpha * LPFVel_x_ + (1 - alpha) * vx;
  LPFVel_y_ = alpha * LPFVel_y_ + (1 - alpha) * vy;

  NavigationType nav_msg;
  nav_msg.x = x;
  nav_msg.y = y;
  nav_msg.psi = psi * 180.0 / M_PI;
  nav_msg.u = LPFVel_x_;
  nav_msg.v = LPFVel_y_;
  nav_msg.r = angular_velocity;
  nav_msg.w = imu_data_->angular_velocity.z;

  navigation_publisher_->publish(nav_msg);

  prev_x_ = x;
  prev_y_ = y;
  prev_psi_ = psi;
  prev_time_ = current_time;
}
