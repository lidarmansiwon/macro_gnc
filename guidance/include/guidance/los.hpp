#pragma once

#include <vector>
#include <Eigen/Dense>

class LineOfSight
{
public:
    LineOfSight();

    void setWaypoints(const std::vector<Eigen::Vector2d>& waypoints);
    void setLookaheadDistance(double R);
    void updateCurrentPose(double x, double y, double yaw);

    Eigen::Vector2d computeLookaheadPoint();
    bool isWaypointReached(double threshold) const;
    void advanceWaypointIfNeeded();
    Eigen::Vector2d getCurrentTarget()const;

private:
    std::vector<Eigen::Vector2d> waypoints_;
    size_t current_index_;
    double lookahead_distance_;

    Eigen::Vector2d current_position_;
    double current_yaw_;

    Eigen::Vector2d computeCircleLineIntersection(
        const Eigen::Vector2d& circle_center,
        double radius,
        const Eigen::Vector2d& line_start,
        const Eigen::Vector2d& line_end
    ) const;
};