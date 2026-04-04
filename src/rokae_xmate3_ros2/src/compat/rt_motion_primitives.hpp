#ifndef ROKAE_COMPAT_RT_MOTION_PRIMITIVES_HPP
#define ROKAE_COMPAT_RT_MOTION_PRIMITIVES_HPP

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <system_error>
#include <vector>

#include "runtime/pose_utils.hpp"

namespace rokae::compat_rt {

struct CircularArcGeometry {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  double radius = 0.0;
  double sweep_angle = 0.0;
};

inline bool isSpeedFactorValid(const double speed) {
  return std::isfinite(speed) && speed > 0.0 && speed <= 1.0;
}

inline double translationMaxAbsDiff(const std::array<double, 6> &lhs,
                                    const std::array<double, 6> &rhs) {
  double max_value = 0.0;
  for (std::size_t i = 0; i < 3; ++i) {
    max_value = std::max(max_value, std::fabs(lhs[i] - rhs[i]));
  }
  return max_value;
}

inline double angularDistance(const std::array<double, 6> &lhs,
                              const std::array<double, 6> &rhs) {
  const std::vector<double> lhs_pose(lhs.begin(), lhs.end());
  const std::vector<double> rhs_pose(rhs.begin(), rhs.end());
  return rokae_xmate3_ros2::runtime::pose_utils::angularDistance(lhs_pose, rhs_pose);
}

inline bool cartesianPoseWithinTolerance(const std::array<double, 6> &actual,
                                         const std::array<double, 6> &expected,
                                         const double translation_tolerance,
                                         const double angular_tolerance) {
  return translationMaxAbsDiff(actual, expected) <= translation_tolerance + 1e-12 &&
         angularDistance(actual, expected) <= angular_tolerance + 1e-12;
}

inline std::array<double, 6> quaternionToPose(const Eigen::Quaterniond &q,
                                              const Eigen::Vector3d &translation) {
  std::array<double, 6> pose{translation.x(), translation.y(), translation.z(), 0.0, 0.0, 0.0};
  const Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
  for (int i = 0; i < 3; ++i) {
    double angle = rpy[i];
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    pose[3 + static_cast<std::size_t>(i)] = angle;
  }
  return pose;
}

inline std::optional<CircularArcGeometry> fitCircularArc(const std::array<double, 6> &start_pose,
                                                         const std::array<double, 6> &aux_pose,
                                                         const std::array<double, 6> &target_pose,
                                                         std::error_code &ec) {
  const Eigen::Vector3d p1(start_pose[0], start_pose[1], start_pose[2]);
  const Eigen::Vector3d p2(aux_pose[0], aux_pose[1], aux_pose[2]);
  const Eigen::Vector3d p3(target_pose[0], target_pose[1], target_pose[2]);
  const Eigen::Vector3d u = p2 - p1;
  const Eigen::Vector3d v = p3 - p1;
  const Eigen::Vector3d w = u.cross(v);
  const double w_sq = w.squaredNorm();
  if (w_sq < 1e-10) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return std::nullopt;
  }

  CircularArcGeometry geometry{};
  const Eigen::Vector3d center_offset =
      (u.squaredNorm() * v.cross(w) + v.squaredNorm() * w.cross(u)) / (2.0 * w_sq);
  geometry.center = p1 + center_offset;
  geometry.radius = (p1 - geometry.center).norm();
  if (!std::isfinite(geometry.radius) || geometry.radius <= 1e-6) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return std::nullopt;
  }

  geometry.axis = w.normalized();
  auto signed_angle = [&geometry](const Eigen::Vector3d &from, const Eigen::Vector3d &to) {
    const Eigen::Vector3d from_n = from.normalized();
    const Eigen::Vector3d to_n = to.normalized();
    const double sin_term = geometry.axis.dot(from_n.cross(to_n));
    const double cos_term = std::clamp(from_n.dot(to_n), -1.0, 1.0);
    return std::atan2(sin_term, cos_term);
  };

  geometry.sweep_angle = signed_angle(p1 - geometry.center, p3 - geometry.center);
  const double angle_aux = signed_angle(p1 - geometry.center, p2 - geometry.center);
  if ((geometry.sweep_angle > 0.0 && (angle_aux < 0.0 || angle_aux > geometry.sweep_angle)) ||
      (geometry.sweep_angle < 0.0 && (angle_aux > 0.0 || angle_aux < geometry.sweep_angle))) {
    constexpr double kTwoPi = 6.28318530717958647692;
    geometry.sweep_angle += geometry.sweep_angle >= 0.0 ? -kTwoPi : kTwoPi;
  }
  if (!std::isfinite(geometry.sweep_angle) || std::fabs(geometry.sweep_angle) < 1e-6) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return std::nullopt;
  }

  ec.clear();
  return geometry;
}

inline int estimateArcInterpolationSteps(const double speed,
                                         const double radius,
                                         const double sweep_angle) {
  const double arc_length = std::fabs(radius * sweep_angle);
  const double clamped_speed = std::clamp(speed, 0.05, 1.0);
  const double chord_goal = std::clamp(0.0025 + 0.0085 * clamped_speed, 0.0025, 0.0110);
  const int steps_by_arc_length = std::max(1, static_cast<int>(std::ceil(arc_length / chord_goal)));
  const int steps_by_angle = std::max(1, static_cast<int>(std::ceil(std::fabs(sweep_angle) * 32.0 / std::max(clamped_speed, 0.1))));
  return std::max(24, std::max(steps_by_arc_length, steps_by_angle));
}

}  // namespace rokae::compat_rt

#endif
