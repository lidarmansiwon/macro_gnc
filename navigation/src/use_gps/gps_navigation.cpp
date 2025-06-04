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
#include "navigation/tool/velocity_calculator.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

GPSNavigation::GPSNavigation(const rclcpp::NodeOptions & node_options)
: Node("gps_navigation", node_options), 
  reference_gps_latitude_(0.0),
  reference_gps_longitude_(0.0),
  reference_utm_x(0.0),
  reference_utm_y(0.0),
  reference_zone(0.0),
  reference_northp(false),
  vel_calc_(0.9)
{ 
  this->declare_parameter<std::string>("imu_topic", "/imu_topic");
  this->declare_parameter<std::string>("gps_topic", "/gps_topic");
  this->declare_parameter<std::string>("navigation_topic", "/navigation_data");

  this->declare_parameter<double>("reference_gps_latitude", 0.0);
  this->declare_parameter<double>("reference_gps_longitude", 0.0);
  this->declare_parameter<double>("velocity_alpha", 0.9);

  std::string imu_topic = this->get_parameter("imu_topic").as_string();
  std::string gps_topic = this->get_parameter("gps_topic").as_string();
  std::string navigation_topic = this->get_parameter("navigation_topic").as_string();

  reference_gps_latitude_  = this->get_parameter("reference_gps_latitude").as_double();
  reference_gps_longitude_ = this->get_parameter("reference_gps_longitude").as_double();
  double velocity_alpha = this->get_parameter("velocity_alpha").as_double();
  vel_calc_ = VelocityCalculator(velocity_alpha);

  // 생성자에서 시작 표시.
  RCLCPP_INFO(this->get_logger(), "Run GPS Navigation");

  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(), std::bind(&GPSNavigation::imu_callback, this, _1));

  gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_topic, rclcpp::SensorDataQoS(), std::bind(&GPSNavigation::gps_callback, this, _1));

  navigation_publisher_ = this->create_publisher<NavigationType>(
    navigation_topic, 10);

  timer_ = this->create_wall_timer(50ms, std::bind(&GPSNavigation::process, this));
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

  rclcpp::Time current_time = this->now();
  double time_sec = current_time.seconds();

  auto result = vel_calc_.update(x, y, psi, time_sec);
  double LPFVel_x = result.LPFVel_x;
  double LPFVel_y = result.LPFVel_y;
  double angular_velocity = result.angular_velocity;

  NavigationType nav_msg;
  nav_msg.x = x;
  nav_msg.y = y;
  nav_msg.psi = psi * 180.0 / M_PI;
  nav_msg.u = LPFVel_x;
  nav_msg.v = LPFVel_y;
  nav_msg.r = angular_velocity;
  nav_msg.w = imu_data_->angular_velocity.z;

  navigation_publisher_->publish(nav_msg);
}
