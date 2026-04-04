#ifndef ROKAE_UTILITY_H
#define ROKAE_UTILITY_H

#include <array>
#include <cmath>
#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rokae/data_types.h"

namespace rokae::Utils {

constexpr double kPi = 3.14159265358979323846;

inline double degToRad(double degree) {
    return degree * kPi / 180.0;
}

inline double radToDeg(double radian) {
    return radian * 180.0 / kPi;
}

template <size_t N>
inline std::array<double, N> degToRad(const std::array<double, N>& degree) {
    std::array<double, N> out{};
    for (size_t i = 0; i < out.size(); ++i) {
        out[i] = degToRad(degree[i]);
    }
    return out;
}

template <size_t N>
inline std::array<double, N> radToDeg(const std::array<double, N>& radian) {
    std::array<double, N> out{};
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

inline void postureToTransArray(const std::array<double, 6>& posture, std::array<double, 16>& out) {
    out = postureToMatrix(rokae::CartesianPosition(posture));
}

inline void postureToTransArray(const rokae::CartesianPosition& posture, std::array<double, 16>& out) {
    out = postureToMatrix(posture);
}

inline void postureToTransArray(const rokae::Frame& posture, std::array<double, 16>& out) {
    out = posture.pos;
}

inline void arrayToTransMatrix(const std::array<double, 16>& in,
                               Eigen::Matrix3d& rotation,
                               Eigen::Vector3d& translation) {
    rotation << in[0], in[1], in[2],
                in[4], in[5], in[6],
                in[8], in[9], in[10];
    translation << in[3], in[7], in[11];
}

inline void transMatrixToArray(const Eigen::Matrix3d& rotation,
                               const Eigen::Vector3d& translation,
                               std::array<double, 16>& out) {
    out = {
        rotation(0, 0), rotation(0, 1), rotation(0, 2), translation(0),
        rotation(1, 0), rotation(1, 1), rotation(1, 2), translation(1),
        rotation(2, 0), rotation(2, 1), rotation(2, 2), translation(2),
        0.0, 0.0, 0.0, 1.0
    };
}

inline void eulerToMatrix(const Eigen::Vector3d& euler, Eigen::Matrix3d& matrix) {
    matrix = (Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX()))
                 .toRotationMatrix();
}

inline void transArrayToPosture(const std::array<double, 16>& transMatrix,
                                std::array<double, 6>& xyz_abc) {
    const auto pose = matrixToPosture(transMatrix);
    xyz_abc = {pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz};
}

}  // namespace rokae::Utils

#endif  // ROKAE_UTILITY_H
