// pid_calculator.hpp
#pragma once
#include <cmath>

class PIDCalculator {
public:
    PIDCalculator() : kp(0.0), ki(0.0), kd(0.0), prev_error(0.0), integral(0.0), first_run(true) {}

    void update(double kp_, double ki_, double kd_) {
        kp = kp_;
        ki = ki_;
        kd = kd_;
    }

    double compute(double error, double saturation, double dt) {
        double error_dot = 0.0;

        if (!first_run) {
            error_dot = (error - prev_error) / dt;
        } else {
            first_run = false;
        }

        integral += error * dt;
        prev_error = error;

        double control_value = kp * error + ki * integral + kd * error_dot;

        if (saturation > 0.0 && std::abs(control_value) >= saturation) {
            control_value = (control_value > 0.0 ? 1.0 : -1.0) * saturation;
        }

        return control_value;
    }

private:
    double kp, ki, kd;
    double prev_error;
    double integral;
    bool first_run;
};