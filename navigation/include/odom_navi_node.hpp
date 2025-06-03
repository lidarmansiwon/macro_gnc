#ifndef ODOM_NAVI_NODE__ODOM_NAVI_NODE_HPP_
#define ODOM_NAVI_NODE__ODOM_NAVI_NODE_HPP_

#include <iostream>
#include <memory>
// C++ Standard lib 
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mk3_msgs/msg/navigation_type.hpp"

class odom_navigation : public rclcpp::Node
{
public:
  using NavigationType = mk3_msgs::msg::NavigationType;
  
}