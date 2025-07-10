#ifndef ODOM_NAVIGATION__ODOM_NAVIGATION_HPP_
#define ODOM_NAVIGATION__ODOM_NAVIGATION_HPP_

#include <iostream>
#include <memory>
// C++ Standard lib 
#include <chrono>
#include <functional>
#include <string>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Custom msg & utils
#include "mk3_msgs/msg/navigation_type.hpp"
#include "tool/low_pass_filter.hpp"
#include "tool/quaternion_utils.hpp"
#include "type.hpp"
#include "navigation/tool/velocity_calculator.hpp"

class OdomNavigation : public rclcpp::Node
{
public:
  using NavigationType = mk3_msgs::msg::NavigationType;
  
  explicit OdomNavigation(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~OdomNavigation();


  
private:
  void process(); // 주기적 처리 함수
  double get_sysSec_from_ros_time(const rclcpp::Time& ros_time);

  // Callback 함수 정의
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // 객체 생성
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Publisher<mk3_msgs::msg::NavigationType>::SharedPtr navigation_publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;

  // 수신한 메시지 저장
  nav_msgs::msg::Odometry::SharedPtr odom_data_{nullptr};
  geometry_msgs::msg::PoseStamped::SharedPtr pose_data_{nullptr};
  sensor_msgs::msg::Imu::SharedPtr imu_data_{nullptr};

  // 상태 플래그
  bool type_odom{false};
  bool enable_imu_{false};

  bool start_time_set_ = false;
  double start_time_sec_ = 0.0;

  SixDofVelCalculator vel_calc_;

  NavigationData boat_data;
};
#endif  // ODOM_NAVI_NODE__ODOM_NAVI_NODE_HPP_