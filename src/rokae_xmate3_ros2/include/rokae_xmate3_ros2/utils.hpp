#ifndef ROKAE_XMATE3_ROS2_UTILS_H
#define ROKAE_XMATE3_ROS2_UTILS_H

#include <array>
#include <cmath>
#include "rokae_xmate3_ros2/types.hpp"

namespace rokae::Utils {

constexpr double kPi = 3.14159265358979323846;

inline double degToRad(double degree) {
    return degree * kPi / 180.0;
}

inline double radToDeg(double radian) {
    return radian * 180.0 / kPi;
}

inline std::array<double, 6> degToRad(const std::array<double, 6>& degree) {
    std::array<double, 6> out{};
    for (size_t i = 0; i < out.size(); ++i) {
        out[i] = degToRad(degree[i]);
    }
    return out;
}

inline std::array<double, 6> radToDeg(const std::array<double, 6>& radian) {
    std::array<double, 6> out{};
    for (size_t i = 0; i < out.size(); ++i) {
        out[i] = radToDeg(radian[i]);
    }
    return out;
}

inline std::array<double, 16> postureToMatrix(const rokae::CartesianPosition& pose) {
    const double cx = std::cos(pose.rx), sx = std::sin(pose.rx);
    const double cy = std::cos(pose.ry), sy = std::sin(pose.ry);
    const double cz = std::cos(pose.rz), sz = std::sin(pose.rz);

    std::array<double, 16> T{};
    T[0] = cz * cy;
    T[1] = cz * sy * sx - sz * cx;
    T[2] = cz * sy * cx + sz * sx;
    T[3] = pose.x;
    T[4] = sz * cy;
    T[5] = sz * sy * sx + cz * cx;
    T[6] = sz * sy * cx - cz * sx;
    T[7] = pose.y;
    T[8] = -sy;
    T[9] = cy * sx;
    T[10] = cy * cx;
    T[11] = pose.z;
    T[12] = 0.0;
    T[13] = 0.0;
    T[14] = 0.0;
    T[15] = 1.0;
    return T;
}

inline rokae::CartesianPosition matrixToPosture(const std::array<double, 16>& T) {
    rokae::CartesianPosition pose;
    pose.x = T[3];
    pose.y = T[7];
    pose.z = T[11];
    pose.ry = std::atan2(-T[8], std::sqrt(T[0] * T[0] + T[4] * T[4]));
    if (std::abs(std::cos(pose.ry)) < 1e-9) {
        pose.rx = 0.0;
        pose.rz = std::atan2(-T[1], T[5]);
    } else {
        pose.rx = std::atan2(T[9], T[10]);
        pose.rz = std::atan2(T[4], T[0]);
    }
    return pose;
}

}  // namespace rokae::Utils

#endif  // ROKAE_XMATE3_ROS2_UTILS_H
