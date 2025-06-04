#ifndef GPS_NAVIGATION__GPS_NAVIGATION_HPP_
#define GPS_NAVIGATION__GPS_NAVIGATION_HPP_

#include <iostream>
#include <memory>
// C++ Standard lib 
#include <chrono>
#include <functional>
#include <string>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"


// Custom msg & utils
#include "mk3_msgs/msg/navigation_type.hpp"
#include "tool/low_pass_filter.hpp"
#include "tool/quaternion_utils.hpp"
#include "type.hpp"
#include "navigation/tool/velocity_calculator.hpp"



class GPSNavigation : public rclcpp::Node
{
public:
  using NavigationType = mk3_msgs::msg::NavigationType;
  
  explicit GPSNavigation(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~GPSNavigation();


  
private:
  void process(); // 주기적 처리 함수

  // Callback 함수 정의
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  // 객체 생성
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
  rclcpp::Publisher<NavigationType>::SharedPtr navigation_publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // 수신한 메시지 저장
  sensor_msgs::msg::Imu::SharedPtr imu_data_{nullptr};
  sensor_msgs::msg::NavSatFix::SharedPtr gps_data_{nullptr};

  double reference_gps_latitude_;
  double reference_gps_longitude_;
  bool origin_set_ = false;

  double reference_utm_x, reference_utm_y;
  int reference_zone;
  bool reference_northp;

  VelocityCalculator vel_calc_;

  NavigationData boat_data;
};
#endif  // GPS_NAVIGATION__GPS_NAVIGATION_HPP_