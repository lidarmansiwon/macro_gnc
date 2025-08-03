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
    
}