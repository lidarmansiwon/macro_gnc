#include "navigation/tool/quaternion_utils.hpp"

// 쿼터니언을 오일러 각으로 변환
std::array<double, 3> euler_from_quaternion(const std::array<double, 4>& quaternion)
{
    double qx = quaternion[0];
    double qy = quaternion[1];
    double qz = quaternion[2];
    double qw = quaternion[3];

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (qw * qy - qz * qx);
    double pitch = (std::fabs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

std::array<double, 4> quaternion_from_euler(double roll, double pitch, double yaw) {
    // 오일러 각 (roll, pitch, yaw)을 쿼터니언으로 변환
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    double qw = cr * cp * cy + sr * sp * sy;
    double qx = sr * cp * cy - cr * sp * sy;
    double qy = cr * sp * cy + sr * cp * sy;
    double qz = cr * cp * sy - sr * sp * cy;

    return {qx, qy, qz, qw};
}
