#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <utility> // for std::pair

class PurePursuit {
public: 
    // 생성자 
    PurePursuit(double kappa);

    void setCurrentPosition(double x, double y);
    void setTargetPosition(double x, double y);

    // 속도 계산 함수
    std::pair<double, double> computeVelocity(); // 반환 --> (vx, vy)

private:
    double kappa_; // 추종 속도 계수
    double current_x_, current_y_;
    double target_x_, target_y_;
};


#endif // PURE_PURSUIT_HPP