// src/navigation_serial_node.cpp

#include <memory>
#include <sstream>
#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "mk3_msgs/msg/navigation_type.hpp"

using boost::asio::serial_port_base;
namespace asio = boost::asio;
using mk3_msgs::msg::NavigationType;

class NavigationSerialNode : public rclcpp::Node
{
public:
  NavigationSerialNode()
  : Node("navigation_serial_node"),
    io_(),
    serial_(io_)
  {
    //–– 파라미터 선언 및 읽기
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 9600);
    auto port      = this->get_parameter("serial_port").as_string();
    auto baud_rate = this->get_parameter("baud_rate").as_int();

    //–– 시리얼 포트 열기
    try {
      serial_.open(port);
      serial_.set_option(serial_port_base::baud_rate(baud_rate));
      serial_.set_option(serial_port_base::character_size(8));
      serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
      serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
      serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
      RCLCPP_INFO(this->get_logger(), "Serial opened: %s @ %d baud", port.c_str(), baud_rate);
    } catch (std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Serial open failed: %s", e.what());
    }

    //–– 토픽 구독 설정
    subscription_ = this->create_subscription<NavigationType>(
      "/navigation_data", 10,
      std::bind(&NavigationSerialNode::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const NavigationType::SharedPtr msg)
  {
    //–– 메시지 CSV 포맷팅
    std::ostringstream oss;
    oss 
      << msg->header.stamp.sec    << ','
      << msg->header.stamp.nanosec<< ','
      << msg->systime             << ','
      << msg->time                << ','
      << msg->x                   << ','
      << msg->y                   << ','
      << msg->z                   << ','
      << msg->psi                 << ','
      << msg->theta               << ','
      << msg->phi                 << ','
      << msg->t_vel               << ','
      << msg->u                   << ','
      << msg->v                   << ','
      << msg->w                   << ','
      << msg->p                   << ','
      << msg->q                   << ','
      << msg->r                   << '\n';
    auto data = oss.str();

    //–– 시리얼로 전송
    if (serial_.is_open()) {
      asio::write(serial_, asio::buffer(data));
    }
  }

  rclcpp::Subscription<NavigationType>::SharedPtr subscription_;
  asio::io_service io_;
  asio::serial_port serial_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationSerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
