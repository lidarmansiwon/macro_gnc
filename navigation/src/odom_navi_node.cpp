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
#include "navigation/odom_navi.hpp"



using std::placeholders::_1;
using namespace std::chrono_literals;

class odom_navigation : public rclcpp::Node
{
  public:
    odom_navigation()
    : Node("odom_navigation")
    {
      navigation_publisher_ = this->create_publisher<mk3_msgs::msg::NavigationType>("navigation", 10); 
      timer_     = this->create_wall_timer(1000ms, std::bind(&odom_navigation::process, this));
      
      imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/ouster/imu", , std::bind(&odom_navigation::imu_callback, this, _1));

      use_odometry = false;
      if (use_odometry)
      {
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "odometry", 10, std::bind(&odom_navigation::odometry_callback, this, _1));
      }
      else
      {
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "current_pose", 10, std::bind(&odom_navigation::pose_callback, this, _1));
      }
    }  
  private:
    void process()   
    {
      RCLCPP_INFO(this->get_logger(), "");
    }

    void odometry_callback(const nav_msgs::msg::Odometry & msg) const 
    {
      RCLCPP_INFO(this->get_logger(), "I heard Odom:");
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped & msg) const 
    {
      RCLCPP_INFO(this->get_logger(), "I heard Pose:");
    }

    void imu_callback(const sensor_msgs::msg::Imu & msg) const 
    {
      RCLCPP_INFO(this->get_logger(), "I heard IMU:");
    }

    bool use_odometry;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Publisher<mk3_msgs::msg::NavigationType>::SharedPtr navigation_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<odom_navigation>());
  rclcpp::shutdown();
  return 0;
}

