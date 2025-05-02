struct NavigationData {
    double x;
    double y;
    double psi;  // yaw in degrees
    double u;    // surge
    double v;    // sway
    double r;    // yaw rate
    double w;    // roll rate (IMU)
};

enum class OdometryType {
    Odometry,
    CurrentPose
};
