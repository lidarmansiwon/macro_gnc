#include <iostream>
#include "navigation/tool/quaternion_utils.hpp"

int main()
{
    // 예시 쿼터니언 (x, y, z, w)
    std::array<double, 4> quaternion = {0.0, 0.0, 0.7071, 0.7071}; // 90도 회전한 쿼터니언

    std::cout << "inital quaternion value: " << quaternion[0] << ", " << 
    quaternion[1] << ", " << quaternion[2] << ", "<< quaternion [3] << std::endl;
    std::cout << std::endl;

    // 오일러 각 계산
    std::array<double, 3> euler_angles = euler_from_quaternion(quaternion);

    // 1. 계산된 오일러 각 출력 (roll, pitch, yaw)
    std::cout << "Output: euler_from_quaternion" << std::endl;
    std::cout << std::endl;
    std::cout << "Roll: " << euler_angles[0] << " rad" << std::endl;
    std::cout << "Pitch: " << euler_angles[1] << " rad" << std::endl;
    std::cout << "Yaw: " << euler_angles[2] << " rad" << std::endl;
    std::cout << std::endl;
    std::array<double, 4> quaternion_angles = quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2]);
    
    // 2. 계산된 쿼터니언 값 출력 (x, y, z, w)
    std::cout << "Trans_Again Output: quaternion_from_euler" << std::endl;
    std::cout << std::endl;
    std::cout << "quaternion: " << quaternion_angles[0] << ", " <<
    quaternion_angles[1] << ", "<< quaternion_angles[2] << ", "<< quaternion_angles [3] << std::endl;
    std::cout << std::endl;
    std::array<double, 3> euler_angles2 = euler_from_quaternion(quaternion_angles);
    std::cout << "Trans_Again2 Output: euler_from_quaternion" << std::endl;
    std::cout << std::endl;

    // 3. 계산된 오일러 각 출력 (roll, pitch, yaw)
    std::cout << "Output: euler_from_quaternion" << std::endl;
    std::cout << std::endl;
    std::cout << "Roll: " << euler_angles2[0] << " rad" << std::endl;
    std::cout << "Pitch: " << euler_angles2[1] << " rad" << std::endl;
    std::cout << "Yaw: " << euler_angles2[2] << " rad" << std::endl;
    std::cout << std::endl;

    return 0;
}
