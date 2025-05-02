#include "navigation/tool/low_pass_filter.hpp"

LowPassFilter::LowPassFilter(double alpha)
: alpha_(alpha), prev_x_(0.0), prev_y_(0.0), initialized_(false) {}

std::pair<double, double> LowPassFilter::filter(double input_x, double input_y) {
    if (!initialized_) {
        prev_x_ = input_x;
        prev_y_ = input_y;
        initialized_ = true;
        return {input_x, input_y};
    }

    double filtered_x = alpha_ * prev_x_ + (1.0 - alpha_) * input_x;
    double filtered_y = alpha_ * prev_y_ + (1.0 - alpha_) * input_y;

    prev_x_ = filtered_x;
    prev_y_ = filtered_y;

    return {filtered_x, filtered_y};
}

// 사용 예시
// LowPassFilter lpf(0.9);
// auto filtered = lpf.filter(current_velocity_x, current_velocity_y);
// double u = filtered.first;
// double v = filtered.second;