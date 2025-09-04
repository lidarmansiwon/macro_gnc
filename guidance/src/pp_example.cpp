#include "guidance/pure_pursuit.hpp"
#include <iostream>
#include <vector>
#include <fstream>  
#include <iomanip> 

int main() {
    PurePursuit pp(1.0);  // kappa = 1.0

    std::vector<std::pair<double, double>> wps = {
        {3.0, 0.0},
        {3.0, 3.0},
        {0.0, 3.0},
        {0.0, 0.0}
    };

    std::ofstream log("trajectory.csv");  // 새로 쓰기 (덮어쓰기)
    if (!log.is_open()) {
        std::cerr << "Failed to open trajectory.csv for writing.\n";
        return -1;
    }

    pp.setWaypoints(wps);

    double x = 0.0, y = 0.0;
    double dt = 0.1;

    log << "x,y,vx,vy\n";  // optional: CSV 헤더

    while (!pp.isFinished()) {
        pp.setCurrentPosition(x, y);
        auto [vx, vy] = pp.computeVelocity();

        x += vx * dt;
        y += vy * dt;

        log << std::fixed << std::setprecision(6)
            << x << "," << y << "," << vx << "," << vy << "\n";

        std::cout << "x: " << x << ", y: " << y
                  << " | vx: " << vx << ", vy: " << vy << std::endl;
    }

    std::cout << "All waypoints reached!" << std::endl;
    log.close();
    return 0;
}
