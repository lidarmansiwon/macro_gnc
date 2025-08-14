#ifndef VELOCITY_CALCULATOR_HPP
#define VELOCITY_CALCULATOR_HPP

#include <cmath>
#include <chrono>

class SixDofVelCalculator
{
public:
  SixDofVelCalculator(double alpha = 0.9)
    : alpha_(alpha), prev_initialized_(false), 
      LPFVel_x_(0.0), LPFVel_y_(0.0), LPFVel_z_(0.0),  
      LPFAngVel_p_(0.0), LPFAngVel_q_(0.0), LPFAngVel_r_(0.0) {}

  struct VelocityResult {
    double vx, vy, vz;
    double p, q, r;
    double LPFVel_x, LPFVel_y, LPFVel_z;
    double LPFAngVel_p, LPFAngVel_q, LPFAngVel_r;
  };

  VelocityResult update(double x, double y, double z,
                        double phi, double theta, double psi,
                        double current_time)
  {
    if (!prev_initialized_) {
      prev_x_ = x;
      prev_y_ = y;
      prev_z_ = z;
      prev_phi_ = phi;
      prev_theta_ = theta;
      prev_psi_ = psi;
      prev_time_ = current_time;
      prev_initialized_ = true;
      return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    double dt = current_time - prev_time_;
    if (dt <= 0.0) {
      return {0.0, 0.0, 0.0, LPFVel_x_, LPFVel_y_};
    }
    // double dt = 0.05;

    double dx = x - prev_x_;
    double dy = y - prev_y_;
    double dz = z - prev_z_;



    double dpsi = (180.0 / M_PI)*psi - (180.0 / M_PI)*prev_psi_;
    double dtheta = (180.0 / M_PI)*theta - (180.0 / M_PI)*prev_theta_;
    double dphi = (180.0 / M_PI)*phi - (180.0 / M_PI)*prev_phi_;

    if (dpsi > 180.0) {
      dpsi = dpsi - 360.0; 
    } else if (dpsi < -180) {
      dpsi = dpsi + 360.0;
    }

    double vx_fixed = dx / dt;
    double vy_fixed = dy / dt;
    double vz_fixed = dz / dt;

    // 회전행렬의 ZYX 순 Euler 변환을 적용한 속도 변환
    double c_phi = std::cos(phi);
    double s_phi = std::sin(phi);
    double c_theta = std::cos(theta);
    double s_theta = std::sin(theta);
    double c_psi = std::cos(psi);
    double s_psi = std::sin(psi);

    // R_n^b = (R_b^n)^T 적용
    double vx = c_theta * c_psi * vx_fixed +
              c_theta * s_psi * vy_fixed -
              s_theta * vz_fixed;

    double vy = (s_phi * s_theta * c_psi - c_phi * s_psi) * vx_fixed +
              (s_phi * s_theta * s_psi + c_phi * c_psi) * vy_fixed +
              s_phi * c_theta * vz_fixed;

    double vz = (c_phi * s_theta * c_psi + s_phi * s_psi) * vx_fixed +
              (c_phi * s_theta * s_psi - s_phi * c_psi) * vy_fixed +
              c_phi * c_theta * vz_fixed;

    // 각속도 계산 (Euler 각의 시간 변화량)
    double p = dphi / dt;
    double q = dtheta / dt;
    double r = dpsi / dt;

    // LPF 적용
    LPFVel_x_ = alpha_ * LPFVel_x_ + (1 - alpha_) * vx;
    LPFVel_y_ = alpha_ * LPFVel_y_ + (1 - alpha_) * vy;
    LPFVel_z_ = alpha_ * LPFVel_z_ + (1 - alpha_) * vz;

    LPFAngVel_p_ = alpha_ * LPFAngVel_p_ + (1 - alpha_) * p;
    LPFAngVel_q_ = alpha_ * LPFAngVel_q_ + (1 - alpha_) * q;
    LPFAngVel_r_ = alpha_ * LPFAngVel_r_ + (1 - alpha_) * r;

    // 상태 갱신
    prev_x_ = x;
    prev_y_ = y;
    prev_z_ = z;
    prev_phi_ = phi;
    prev_theta_ = theta;
    prev_psi_ = psi;
    prev_time_ = current_time;

    return {vx, vy, vz, p, q, r, LPFVel_x_, LPFVel_y_, LPFVel_z_,
            LPFAngVel_p_, LPFAngVel_q_, LPFAngVel_r_};
  }

private:
  double alpha_;
  bool prev_initialized_;
  double prev_x_, prev_y_, prev_z_;
  double prev_phi_, prev_theta_, prev_psi_;
  double prev_time_;

  double LPFVel_x_, LPFVel_y_, LPFVel_z_;
  double LPFAngVel_p_, LPFAngVel_q_, LPFAngVel_r_;
};

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
