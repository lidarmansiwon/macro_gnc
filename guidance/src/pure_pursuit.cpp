#include "guidance/pure_pursuit.hpp"
#include <cmath> // for hypot
#include <tuple> // for std::tie

PurePursuit::PurePursuit(double kappa, double goal_tolerance)
    : kappa_(kappa), goal_tolerance_(goal_tolerance), current_x_(0), current_y_(0), current_wp_index_(0) {}

void PurePursuit::setCurrentPosition(double x, double y) {
    current_x_ = x;
    current_y_ = y;
}

void PurePursuit::setWaypoints(const std::vector<std::pair<double, double>>& wps) {
    waypoints_ = wps;
    current_wp_index_ = 0;
}

/*
const: 이 함수 내에서 wps를 수정하지 않겠다는 약속

& (참조): wps를 복사하지 않고, 원본을 참조하겠다는 의미

결과적으로: "읽기 전용 참조"

만일 그냥 아래처럼 코드를 작성해서 --> 값으로 전달할 경우, 전체 벡터가 복사되어 메모리 낭비와 성능 저하가 발생

void setWaypoints(std::vector<std::pair<double, double>> wps);

*/

std::pair<double, double> PurePursuit::computeVelocity() {

    if (current_wp_index_ >= waypoints_.size()) {
        return {0.0, 0.0};  // 목표 끝남
    }

    auto [tx, ty] = waypoints_[current_wp_index_];
    double dx = tx - current_x_;
    double dy = ty - current_y_;

    double distance = std::hypot(dx, dy);

    if (distance < goal_tolerance_) {
        current_wp_index_++;
        if (current_wp_index_ >= waypoints_.size()) {
            return {0.0, 0.0};
        }
        std::tie(tx, ty) = waypoints_[current_wp_index_];
        dx = tx - current_x_;
        dy = ty - current_y_;
        distance = std::hypot(dx, dy);
    }

    double vx = kappa_ * dx / distance;
    double vy = kappa_ * dy / distance;

    return {vx, vy};
}

bool PurePursuit::isFinished() const {
    return current_wp_index_ >= waypoints_.size();
}

/*

std::tie --> std::pair<double, double> 와 같은 구조체에서 structured binding 을 할당

c++에서는 아래 코드와 동일함. 

auto next_wp = waypoints_[current_wp_index_];
tx = next_wp.first;
ty = next_wp.second;

Python 문법으로는 아래와 비슷하다. 

tx, ty = waypoints[current_wp_index]

next_wp = waypoints[current_wp_index]
tx = next_wp[0]
ty = next_wp[1]

*/