#include "navigation/odom_navigation.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomNavigation>());
  rclcpp::shutdown();
  return 0;
}