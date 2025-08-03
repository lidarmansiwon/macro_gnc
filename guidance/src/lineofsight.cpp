#include "guidance/lineofsight.hpp"
#include "cmath"
#include <stdexcept>

LineOfSight::LineOfSight()
    : current_index_(0), lookahead_distance_(5.0), current_yaw_(0.0) {}

void LineOfSight::setWaypoints(const std::vector<Eigen::Vector2d>& waypoints) {
    waypoints_ = waypoints;
    current_index_ = 0;
}

void LineOfSight::setLookaheadDistance(double R) {
    lookahead_distance_ = R;
}

void LineOfSight::updateCurrentPose(double x, double y, double yaw) {
    current_position_ = Eigen::Vector2d(x, y);
    current_yaw_ = yaw;
}

bool LineOfSight::isWaypointReached(double threshold) const {
    if (current_index_ >= waypoints_.size()) return true;
    double dist = (current_position_ - waypoints_[current_index_]).norm();
    return dist < threshold;
}

void LineOfSight::advanceWaypointIfNeeded() {
    if (isWaypointReached(1.0) && current_index_ + 1 < waypoints_.size()) {
        ++current_index_;
    }
}

Eigen::Vector2d LineOfSight::getCurrentTarget() const {
    if (current_index_ > = waypoints_.size()) {
        return waypoints_.back();
    }
    return waypoints_[current_index_];
}

Eigen::Vector2d LineOfSight::computeLookaheadPoint() {
    if (current_index_ + 1 >= waypoints_.size()) {
        return waypoints_.back();
    }
    const Eigen::Vector2d& p1 = waypoints_[current_index_];
    const Eigen::Vector2d& p2 = waypoints_[current_index_ + 1];

    return computeCircleLineIntersection(current_position_, lookahead_distance_, p1, p2);
}

// 원과 웨이포인트 구간 직선의 교차점을 계산
Eigen::Vector2d LineOfSight::computeCircleLineIntersection(
    const Eigen::Vector2d& center,
    double radius, 
    const Eigen::Vector2d& line_start, 
    const Eigen::Vector2d& line_end
) const {
    Eigen::Vector2d d = line_end - line_start;
    Eigen::Vector2d f = line_start - center;

    double a = d.dot(d);
    double b = 2 * f.dot(d);
    double c = f.dot(f) - radius * radius;

    double discriminant = b*b - 4*a*c;
    if (discriminant < 0) {
        return line_end; // fallback
    }

    discriminant = std::sqrt(discriminant);
    double t1 = (-b - discriminant) / (2*a);
    double t2 = (-b + discriminant) / (2*a);

    // [0,1] 세그먼트 내에서의 line_end에 더 가까운 지점 선택
    if (t2 >= 0 && t2 <= 1) {
        return line_start + t2 * d;
    } else if (t1 >= 0 && t1 <= 1) {
        return line_start + t1 * d;
    } else {
        return line_end;
    }
}

double LineOfSight::computeDesiredYaw() const {
    Eigen::Vector2d target = computeLookaheadPoint();
    Eigen::Vector2d delta = target - current_position_;
    return std::atan2(delta.y(), delta.x());
}
