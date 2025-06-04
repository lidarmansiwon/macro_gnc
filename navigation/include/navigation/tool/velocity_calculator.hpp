#ifndef VELOCITY_CALCULATOR_HPP
#define VELOCITY_CALCULATOR_HPP

#include <cmath>
#include <chrono>

class VelocityCalculator
{
public:
  VelocityCalculator(double alpha = 0.9)
    : alpha_(alpha), prev_initialized_(false), LPFVel_x_(0.0), LPFVel_y_(0.0) {}

  struct VelocityResult {
    double vx;
    double vy;
    double angular_velocity;
    double LPFVel_x;
    double LPFVel_y;
  };

  VelocityResult update(double x, double y, double psi, double current_time)
  {
    if (!prev_initialized_) {
      prev_x_ = x;
      prev_y_ = y;
      prev_psi_ = psi;
      prev_time_ = current_time;
      prev_initialized_ = true;
      return {0.0, 0.0, 0.0, 0.0, 0.0};
    }

    // double dt = current_time - prev_time_;
    // if (dt <= 0.0) {
    //   return {0.0, 0.0, 0.0, LPFVel_x_, LPFVel_y_};
    // }
    double dt = 0.05;

    double dx = x - prev_x_;
    double dy = y - prev_y_;
    double dpsi = psi - prev_psi_;

    double vx_fixed = dx / dt;
    double vy_fixed = dy / dt;

    double cos_psi = std::cos(psi);
    double sin_psi = std::sin(psi);

    double vx = vx_fixed * cos_psi + vy_fixed * sin_psi;
    double vy = -vx_fixed * sin_psi + vy_fixed * cos_psi;

    double angular_velocity = dpsi / dt;

    // Low-pass filter
    LPFVel_x_ = alpha_ * LPFVel_x_ + (1 - alpha_) * vx;
    LPFVel_y_ = alpha_ * LPFVel_y_ + (1 - alpha_) * vy;

    // Save current state
    prev_x_ = x;
    prev_y_ = y;
    prev_psi_ = psi;
    prev_time_ = current_time;

    return {vx, vy, angular_velocity, LPFVel_x_, LPFVel_y_};
  }

private:
  double alpha_;
  bool prev_initialized_;
  double prev_x_, prev_y_, prev_psi_, prev_time_;
  double LPFVel_x_, LPFVel_y_;
};

#endif  // VELOCITY_CALCULATOR_HPP
