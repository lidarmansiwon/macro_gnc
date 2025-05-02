#ifndef LOW_PASS_FILTER_HPP
#define LOW_PASS_FILTER_HPP
#include <utility>

class LowPassFilter {
public:
    explicit LowPassFilter(double alpha = 0.9); // explicit -> 암시적 형변환 막기

    // 입력값 (x, y)에 대해 필터 적용 후 필터링된 (x, y) 반환
    std::pair<double, double> filter(double input_x, double input_y);

private:
    double alpha_;  // 필터 계수 (0~1), 높을수록 변화가 느림
    double prev_x_;
    double prev_y_;
    bool initialized_;
};

#endif // LOW_PASS_FILTER_HPP
