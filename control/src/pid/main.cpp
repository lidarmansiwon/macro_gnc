#include "control/pid_control.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDControl>());
  rclcpp::shutdown();
  return 0;
}