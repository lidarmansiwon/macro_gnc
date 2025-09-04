#include "pure_pursuit.hpp"
#include <cmath> //for hypot

PurePursuit::PurePursuit(double kappa)
    : kappa_(kappa), current_x_(0.0), current_y_(0.0), target_x_(0.0), target_y_(0.0) {}

void PurePursuit::setCurrentPosition(double x, double y) {
    current_x_ = x;
    current_y_ = y;
}

void PurePursuit::setTargetPosition(double x, double y) {
    target_x_ = x;
    target_y_ = y;
}

std::pair<double, double> PurePursuit::computeVelocity() {
    double dx = target_x_ - current_x_;
    double dy = target_y_ - current_y_;

    double distance = std::hypot(dx, dy);

    if (distance < 1e-6)
        return {0.0, 0.0};  // 목표점에 도달한 경우

    double vx = kappa_ * dx / distance;
    double vy = kappa_ * dy / distance;

    return {vx, vy};
}