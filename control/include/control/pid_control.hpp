#ifndef PID_CONTROL__PID_CONTROL_HPP_
#define PID_CONTROL__PID_CONTROL_HPP_

#include <iostream>
#include <memory>
// C++ Standard lib 
#include <chrono>
#include <functional>
#include <string>
#include "std_msgs/msg/float64.hpp"

// ROS2
#include "rclcpp/rclcpp.hpp"


// Custom msg & utils
#include "mk3_msgs/msg/navigation_type.hpp"
#include "mk3_msgs/msg/guidance_type.hpp"
#include "mk3_msgs/msg/control_type.hpp"
#include "tool/pid_calculator.hpp"

struct PWM
{
    double port;
    double stbd;
};

class PIDControl : public rclcpp::Node
{
public:
  using ControlType = mk3_msgs::msg::ControlType;
  using NavigationType = mk3_msgs::msg::NavigationType;
  using GuidanceType = mk3_msgs::msg::GuidanceType;

  
  explicit PIDControl(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~PIDControl();


private:
  void process(); // 주기적 처리 함수
  PWM cal_pwm(double desire_psi_, double desire_u_, double psi_, double u_);

  // Callback 함수 정의
  void navigation_callback(const NavigationType::SharedPtr msg);
  void guidance_callback(const GuidanceType::SharedPtr msg);
  void rc_callback(const std_msgs::msg::Float64::SharedPtr msg);

  // 객체 생성
  rclcpp::Subscription<GuidanceType>::SharedPtr guidance_subscription_;
  rclcpp::Subscription<NavigationType>::SharedPtr navigation_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rc_subscription_;
  rclcpp::Publisher<ControlType>::SharedPtr control_publisher_;

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> & parameters);

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // 수신한 메시지 저장
  NavigationType::SharedPtr navigation_data_{nullptr};
  GuidanceType::SharedPtr guidance_data_{nullptr};
  std_msgs::msg::Float64::SharedPtr rc_data_{nullptr};

  double rc_mode;

  double command_pwm_port;
  double command_pwm_stbd;

  double thruster_pwm_port;
  double thruster_pwm_stbd;

  double kp_, ki_, kd_, dt_, saturation_, standard_pwm_;
  double dead_max_, dead_min_;

  PIDCalculator pid_calculator;


};
#endif  // PID_CONTROL__PID_CONTROL_HPP_