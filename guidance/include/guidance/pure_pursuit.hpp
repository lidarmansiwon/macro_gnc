#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <utility> // for std::pair
#include <vector>
#include <cstddef>

class PurePursuit {
public: 
    // 생성자 
    PurePursuit(double kappa, double goal_tolerance = 0.5);

    void setCurrentPosition(double x, double y);
    void setWaypoints(const std::vector<std::pair<double, double>>& wps);

    // 속도 계산 함수
    std::pair<double, double> computeVelocity(); // 반환 --> (vx, vy)

    bool isFinished() const;

private:
    double kappa_; // 추종 속도 계수
    double current_x_, current_y_;
    double goal_tolerance_;
    std::vector<std::pair<double, double>> waypoints_;
    size_t current_wp_index_;
};


#endif // PURE_PURSUIT_HPP