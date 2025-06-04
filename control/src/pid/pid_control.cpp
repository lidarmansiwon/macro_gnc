#include <iostream>
#include <memory>
// C++ Standard lib 
#include <chrono>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "control/pid_control.hpp"
#include "tool/pid_calculator.hpp"

// Custom msg & utils
#include "mk3_msgs/msg/navigation_type.hpp"
#include "mk3_msgs/msg/guidance_type.hpp"
#include "mk3_msgs/msg/control_type.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

PIDControl::PIDControl(const rclcpp::NodeOptions & node_options)
: Node("PID_control", node_options),
  command_pwm_port(0.0),
  command_pwm_stbd(0.0),
  thruster_pwm_port(0.0),
  thruster_pwm_stbd(0.0),
  rc_mode(2.0),
  pid_calculator()
{
    this->declare_parameter<double>("kp", 0.0);
    this->declare_parameter<double>("ki", 0.0);
    this->declare_parameter<double>("kd", 0.0);
    this->declare_parameter<double>("dt", 0.1);
    this->declare_parameter<double>("saturation", 0.0);
    this->declare_parameter<double>("standard_pwm", 0.1);

    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();
    dt_ = this->get_parameter("dt").as_double();
    saturation_ = this->get_parameter("saturation").as_double();
    standard_pwm_ = this->get_parameter("standard_pwm").as_double();

    // DeadZone 
    this->declare_parameter<double>("dead_max", 1530.0);
    this->declare_parameter<double>("dead_min", 1470.0);
    dead_max_ = this->get_parameter("dead_max").as_double();
    dead_min_ = this->get_parameter("dead_min").as_double();

    this->declare_parameter<std::string>("navigation_topic", "/navigation_data");
    this->declare_parameter<std::string>("guidance_topic", "/guidance_data");
    this->declare_parameter<std::string>("rc_topic", "/rc_data");
    this->declare_parameter<std::string>("control_topic", "/control_data");

    std::string navigation_topic = this->get_parameter("navigation_topic").as_string();
    std::string guidance_topic = this->get_parameter("guidance_topic").as_string();
    std::string rc_topic = this->get_parameter("rc_topic").as_string();
    std::string control_topic  = this->get_parameter("control_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Run PID Control");

    guidance_subscription_   = this->create_subscription<GuidanceType>(guidance_topic, 10, std::bind(&PIDControl::guidance_callback, this, _1));
    navigation_subscription_ = this->create_subscription<NavigationType>(navigation_topic, 10, std::bind(&PIDControl::navigation_callback, this, _1));
    rc_subscription_         = this->create_subscription<std_msgs::msg::Float64>(rc_topic, 10, std::bind(&PIDControl::rc_callback, this, _1));
    control_publisher_       = this->create_publisher<ControlType>(control_topic, 10);

    timer_ = this->create_wall_timer(50ms, std::bind(&PIDControl::process, this));

    pid_calculator.update(kp_, ki_, kd_);

}

PIDControl::~PIDControl()
{

}

void PIDControl::guidance_callback(const GuidanceType::SharedPtr msg){guidance_data_ = msg;}
void PIDControl::navigation_callback(const NavigationType::SharedPtr msg){navigation_data_ = msg;}
void PIDControl::rc_callback(const std_msgs::msg::Float64::SharedPtr msg){rc_data_ = msg;}

void PIDControl::process()
{
    if (!guidance_data_ || !navigation_data_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "GUIDANCE or NAVIGATION data not received yet");
        return;
    }

    if (!rc_data_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "rc data not received yet");
    } else {
        RCLCPP_INFO_ONCE(this->get_logger(),"Received RC data");
        rc_mode = rc_data_->data;
    }

    if (rc_mode == 1.0) // RC MODE
    {
        RCLCPP_INFO_ONCE(this->get_logger(),"\nMODE: RC\n");
        thruster_pwm_port = 1500.0;
        thruster_pwm_stbd = 1500.0;
    }
    else if(rc_mode == 2.0) // AUTO MODE
    {
        RCLCPP_INFO_ONCE(this->get_logger(),"\nMODE: AUTO\n");
        PWM pwm_value;
        double desire_psi = guidance_data_->desired_psi;
        double desire_u = guidance_data_->desired_u;
        
        double psi = navigation_data_->psi;
        double u = navigation_data_->u;
        
        pwm_value = cal_pwm(desire_psi, desire_u, psi, u);

        

        ControlType control_msg;
        control_msg.delta_psi = desire_psi;
        control_msg.delta_u = desire_u;
        control_msg.pwm_standard = standard_pwm_;
        control_msg.command_pwm_port = pwm_value.port;
        control_msg.command_pwm_stbd = pwm_value.stbd;

        control_publisher_->publish(control_msg);
    }
}

PWM PIDControl::cal_pwm(double desire_psi_, double desire_u_, double psi_, double u_)
{
    double error_psi = desire_psi_ - psi_;

    double delta_psi = pid_calculator.compute(error_psi, saturation_, dt_);

    command_pwm_port = standard_pwm_ - delta_psi;
    command_pwm_stbd = standard_pwm_ + delta_psi;

    if (command_pwm_port < dead_max_)
    {
        command_pwm_port = dead_min_ - delta_psi;
    }

    if (command_pwm_stbd < dead_max_)
    {
        command_pwm_stbd = dead_min_ + delta_psi;
    }
    
    PWM pwm;
    pwm.port = command_pwm_port;
    pwm.stbd = command_pwm_stbd;

    return pwm;
}