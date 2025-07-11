// src/serial_navigation_receive_node.cpp

#include <memory>
#include <sstream>
#include <vector>
#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "mk3_msgs/msg/navigation_type.hpp"

using boost::asio::serial_port_base;
namespace asio = boost::asio;
using mk3_msgs::msg::NavigationType;

class SerialNavigationReceiveNode : public rclcpp::Node
{
public:
  SerialNavigationReceiveNode()
  : Node("serial_navigation_receive_node"),
    io_(),
    serial_(io_)
  {
    // 파라미터 선언 및 읽기
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 9600);
    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_   = this->get_parameter("baud_rate").as_int();

    // 시리얼 포트 오픈
    try {
      serial_.open(serial_port_);
      serial_.set_option(serial_port_base::baud_rate(baud_rate_));
      serial_.set_option(serial_port_base::character_size(8));
      serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
      serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
      serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
      RCLCPP_INFO(this->get_logger(), "Opened serial port %s @ %d baud", serial_port_.c_str(), baud_rate_);
    } catch (std::exception &e) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open serial port %s: %s", serial_port_.c_str(), e.what());
      rclcpp::shutdown();
      return;
    }

    // 퍼블리셔 생성
    pub_ = this->create_publisher<NavigationType>("/navigation_data", 10);

    // 별도 스레드에서 읽기 루프 실행
    reader_thread_ = std::thread([this]() { readLoop(); });
  }

  ~SerialNavigationReceiveNode()
  {
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
  }

private:
  void readLoop()
  {
    asio::streambuf buf;
    while (rclcpp::ok() && serial_.is_open()) {
      try {
        // '\n'까지 읽어들임
        asio::read_until(serial_, buf, '\n');
      } catch (std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
        break;
      }

      std::istream is(&buf);
      std::string line;
      std::getline(is, line);
      if (line.empty()) {
        continue;
      }

      // **터미널에 원본 CSV 라인 로그**
      RCLCPP_INFO(this->get_logger(), "Received raw: %s", line.c_str());

      parseAndPublish(line);
    }
  }

  void parseAndPublish(const std::string &line)
  {
    std::vector<std::string> tok;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) {
      tok.push_back(item);
    }
    // 필드 수 검사 (sec, nanosec, systime, time, x,y,z, psi,theta,phi, t_vel, u,v,w, p,q,r) = 17개
    if (tok.size() < 17) {
      RCLCPP_WARN(this->get_logger(), "Unexpected token count: %zu", tok.size());
      return;
    }

    auto msg = NavigationType();
    // header
    msg.header.stamp.sec     = std::stoll(tok[0]);
    msg.header.stamp.nanosec = std::stoll(tok[1]);
    // body
    msg.systime = std::stod(tok[2]);
    msg.time    = std::stod(tok[3]);
    msg.x       = std::stod(tok[4]);
    msg.y       = std::stod(tok[5]);
    msg.z       = std::stod(tok[6]);
    msg.psi     = std::stod(tok[7]);
    msg.theta   = std::stod(tok[8]);
    msg.phi     = std::stod(tok[9]);
    msg.t_vel   = std::stod(tok[10]);
    msg.u       = std::stod(tok[11]);
    msg.v       = std::stod(tok[12]);
    msg.w       = std::stod(tok[13]);
    msg.p       = std::stod(tok[14]);
    msg.q       = std::stod(tok[15]);
    msg.r       = std::stod(tok[16]);

    // **파싱된 주요 필드 로그** (예: x, y, z, psi)
    RCLCPP_INFO(this->get_logger(),
      "Parsed → systime: %.3f, time: %.6f, x: %.4f, y: %.4f, z: %.4f, psi: %.4f",
      msg.systime, msg.time, msg.x, msg.y, msg.z, msg.psi);

    // 토픽 퍼블리시
    pub_->publish(msg);
  }

  // 멤버 변수
  std::string serial_port_;
  int baud_rate_;
  asio::io_service io_;
  asio::serial_port serial_;
  std::thread reader_thread_;
  rclcpp::Publisher<NavigationType>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialNavigationReceiveNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
