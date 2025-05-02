#ifndef QUATERNION_UTILS_HPP
#define QUATERNION_UTILS_HPP

#include <cmath>
#include <array>

// 쿼터니언을 오일러 각(roll, pitch, yaw)로 변환하는 함수
std::array<double, 3> euler_from_quaternion(const std::array<double, 4>& quaternion);

// 오일러 각(roll, pitch, yaw)을 쿼터니언으로 변환하는 함수
std::array<double, 4> quaternion_from_euler(double roll, double pitch, double yaw);
#endif // QUATERNION_UTILS_HPP
