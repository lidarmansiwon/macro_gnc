#include "navigation/gps_navigation.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSNavigation>());
  rclcpp::shutdown();
  return 0;
}