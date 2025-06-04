#ifndef ODOM_NAVI_NODE__ODOM_NAVI_NODE_HPP_
#define ODOM_NAVI_NODE__ODOM_NAVI_NODE_HPP_

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

class OdomNavigation : public rclcpp::Node
{
public:
  using NavigationType = mk3_msgs::msg::NavigationType;
  
  explicit OdomNavigation(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~OdomNavigation();


  
private:
  void process(); // 주기적 처리 함수
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

  // 수신한 메시지 저장
  nav_msgs::msg::Odometry::SharedPtr odom_data_{nullptr};
  geometry_msgs::msg::PoseStamped::SharedPtr pose_data_{nullptr};
  sensor_msgs::msg::Imu::SharedPtr imu_data_{nullptr};

  // 상태 플래그
  bool type_odom{false};
  bool enable_imu_{false};
  bool prev_pose_initialized_{false};

  // 이전 위치 및 시간
  double prev_x_{0.0};
  double prev_y_{0.0};
  double prev_psi_{0.0};
  rclcpp::Time prev_time_;

  // 속도 필터 변수
  double LPFVel_x_{0.0};
  double LPFVel_y_{0.0};

  NavigationData boat_data;
};
#endif  // ODOM_NAVI_NODE__ODOM_NAVI_NODE_HPP_