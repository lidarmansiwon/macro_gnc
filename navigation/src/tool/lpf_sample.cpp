#include <iostream>
#include "navigation/tool/low_pass_filter.hpp"
#include <vector>

int main() {
    // 필터 초기화: alpha 값은 0.1~1.0 사이 (작을수록 반응이 느림)
    LowPassFilter lpf(0.2);

    // 테스트 입력값들 (예: noisy sensor data)
    std::vector<std::pair<double, double>> input_data = {
        {1.0, 2.0},
        {1.5, 2.5},
        {2.0, 3.0},
        {1.8, 2.8},
        {2.2, 3.2}
    };

    std::cout << "Filtered Output:\n";
    for (const auto& input : input_data) {
        auto filtered = lpf.filter(input.first, input.second);
        std::cout << "Input: (" << input.first << ", " << input.second << ") "
                  << "=> Filtered: (" << filtered.first << ", " << filtered.second << ")\n";
    }

    return 0;
}
