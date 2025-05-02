// navigation_node.hpp
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mk3_msgs/msg/navigation_type.hpp"
#include "low_pass_filter.hpp"
#include "quaternion_utils.hpp"

class NavigationNode : public rclcpp::Node {
public:
    NavigationNode();

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void timerCallback();
    void computeOdometry();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<mk3_msgs::msg::NavigationType>::SharedPtr navi_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool enable_slam_;
    bool enable_imu_;
    bool start_check_;
    int data_check_;
    
    nav_msgs::msg::Odometry::SharedPtr odometry_data_;
    sensor_msgs::msg::Imu::SharedPtr imu_data_;
    nav_msgs::msg::Odometry boat_odometry_;

    std::optional<std::vector<double>> prev_pose_;
    rclcpp::Time prev_time_;

    LowPassFilter vel_filter_;
};
