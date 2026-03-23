#ifndef ROKAE_XMATE3_ROS2_GAZEBO_APPROXIMATE_MODEL_HPP
#define ROKAE_XMATE3_ROS2_GAZEBO_APPROXIMATE_MODEL_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"

namespace rokae_xmate3_ros2::gazebo_model {

using Matrix4d = Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

struct LoadContext {
  double mass = 0.0;
  std::array<double, 3> com{{0.0, 0.0, 0.0}};
};

struct DynamicsBreakdown {
  std::array<double, 6> full{};
  std::array<double, 6> gravity{};
  std::array<double, 6> coriolis{};
  std::array<double, 6> inertia{};
  std::array<double, 6> external{};
};

inline double meanAbs(const std::array<double, 6> &values) {
  double sum = 0.0;
  for (double value : values) {
    sum += std::fabs(value);
  }
  return sum / static_cast<double>(values.size());
}

inline double comNorm(const LoadContext &load) {
  return std::sqrt(load.com[0] * load.com[0] + load.com[1] * load.com[1] + load.com[2] * load.com[2]);
}

template <std::size_t DoF>
inline std::vector<double> toJointVector(const std::array<double, DoF> &values) {
  return std::vector<double>(values.begin(), values.end());
}

template <std::size_t DoF>
inline Vector6d toVector6(const std::array<double, DoF> &values) {
  Vector6d out = Vector6d::Zero();
  for (std::size_t i = 0; i < DoF && i < 6; ++i) {
    out(static_cast<Eigen::Index>(i)) = values[i];
  }
  return out;
}

inline Vector6d toVector6(const std::array<double, 6> &values) {
  Vector6d out = Vector6d::Zero();
  for (std::size_t i = 0; i < 6; ++i) {
    out(static_cast<Eigen::Index>(i)) = values[i];
  }
  return out;
}

inline std::array<double, 6> toArray6(const Vector6d &values) {
  std::array<double, 6> out{};
  for (std::size_t i = 0; i < 6; ++i) {
    out[i] = values(static_cast<Eigen::Index>(i));
  }
  return out;
}

template <std::size_t DoF>
inline std::array<double, DoF> toJointArray(const Vector6d &values) {
  std::array<double, DoF> out{};
  for (std::size_t i = 0; i < DoF && i < 6; ++i) {
    out[i] = values(static_cast<Eigen::Index>(i));
  }
  return out;
}

template <std::size_t DoF>
inline Matrix6d jacobian(::gazebo::xMate3Kinematics &kinematics, const std::array<double, DoF> &joint_position) {
  return kinematics.computeJacobian(toJointVector(joint_position));
}

inline Matrix4d poseToMatrix(const std::array<double, 6> &pose) {
  const double cx = std::cos(pose[3]);
  const double sx = std::sin(pose[3]);
  const double cy = std::cos(pose[4]);
  const double sy = std::sin(pose[4]);
  const double cz = std::cos(pose[5]);
  const double sz = std::sin(pose[5]);

  Matrix4d matrix = Matrix4d::Identity();
  matrix(0, 0) = cz * cy;
  matrix(0, 1) = cz * sy * sx - sz * cx;
  matrix(0, 2) = cz * sy * cx + sz * sx;
  matrix(1, 0) = sz * cy;
  matrix(1, 1) = sz * sy * sx + cz * cx;
  matrix(1, 2) = sz * sy * cx - cz * sx;
  matrix(2, 0) = -sy;
  matrix(2, 1) = cy * sx;
  matrix(2, 2) = cy * cx;
  matrix(0, 3) = pose[0];
  matrix(1, 3) = pose[1];
  matrix(2, 3) = pose[2];
  return matrix;
}

inline std::array<double, 6> matrixToPose(const Matrix4d &matrix) {
  constexpr double kPi = 3.14159265358979323846;
  std::array<double, 6> pose{};
  pose[0] = matrix(0, 3);
  pose[1] = matrix(1, 3);
  pose[2] = matrix(2, 3);

  pose[4] = std::atan2(-matrix(2, 0), std::sqrt(matrix(0, 0) * matrix(0, 0) + matrix(1, 0) * matrix(1, 0)));
  if (std::fabs(pose[4] - kPi / 2.0) < 1e-3) {
    pose[5] = 0.0;
    pose[3] = std::atan2(matrix(1, 2), matrix(1, 1));
  } else if (std::fabs(pose[4] + kPi / 2.0) < 1e-3) {
    pose[5] = 0.0;
    pose[3] = std::atan2(-matrix(1, 2), matrix(1, 1));
  } else {
    pose[5] = std::atan2(matrix(1, 0), matrix(0, 0));
    pose[3] = std::atan2(matrix(2, 1), matrix(2, 2));
  }

  auto normalize = [kPi](double angle) {
    while (angle > kPi) {
      angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
      angle += 2.0 * kPi;
    }
    return angle;
  };
  pose[3] = normalize(pose[3]);
  pose[4] = normalize(pose[4]);
  pose[5] = normalize(pose[5]);
  return pose;
}

template <std::size_t DoF>
inline std::array<double, 6> cartesianPose(const std::array<double, DoF> &joint_position,
                                           ::gazebo::xMate3Kinematics &kinematics,
                                           const std::array<double, 6> &tool_pose = {}) {
  std::array<double, 6> pose{};
  const auto flange_pose = kinematics.forwardKinematicsRPY(toJointVector(joint_position));
  for (std::size_t i = 0; i < 6 && i < flange_pose.size(); ++i) {
    pose[i] = flange_pose[i];
  }
  const bool has_tool_offset = std::any_of(tool_pose.begin(), tool_pose.end(), [](double value) {
    return std::fabs(value) > 1e-12;
  });
  if (!has_tool_offset) {
    return pose;
  }
  return matrixToPose(poseToMatrix(pose) * poseToMatrix(tool_pose));
}

template <std::size_t DoF>
inline std::array<double, 6> cartesianVelocity(::gazebo::xMate3Kinematics &kinematics,
                                               const std::array<double, DoF> &joint_position,
                                               const std::array<double, DoF> &joint_velocity) {
  const auto jac = jacobian(kinematics, joint_position);
  return toArray6(jac * toVector6(joint_velocity));
}

template <std::size_t DoF>
inline std::array<double, 6> cartesianAcceleration(::gazebo::xMate3Kinematics &kinematics,
                                                   const std::array<double, DoF> &joint_position,
                                                   const std::array<double, DoF> &joint_velocity,
                                                   const std::array<double, DoF> &joint_acceleration) {
  const auto jac = jacobian(kinematics, joint_position);
  const Vector6d qd = toVector6(joint_velocity);
  const Vector6d qdd = toVector6(joint_acceleration);
  Vector6d cartesian_acc = jac * qdd;

  if (qd.norm() > 1e-9) {
    constexpr double kDt = 1e-3;
    auto future_joint_position = joint_position;
    for (std::size_t i = 0; i < DoF && i < 6; ++i) {
      future_joint_position[i] += joint_velocity[i] * kDt;
    }
    const auto future_jac = jacobian(kinematics, future_joint_position);
    cartesian_acc += ((future_jac - jac) / kDt) * qd;
  }

  return toArray6(cartesian_acc);
}

template <std::size_t DoF>
inline std::array<double, DoF> jointAccelerationFromCartesian(::gazebo::xMate3Kinematics &kinematics,
                                                              const std::array<double, DoF> &joint_position,
                                                              const std::array<double, 6> &cartesian_acceleration) {
  const auto jac = jacobian(kinematics, joint_position);
  const auto pseudo_inverse = jac.completeOrthogonalDecomposition().pseudoInverse();
  const auto joint_acc = pseudo_inverse * toVector6(cartesian_acceleration);
  return toJointArray<DoF>(joint_acc);
}

template <std::size_t DoF>
inline DynamicsBreakdown computeApproximateDynamics(::gazebo::xMate3Kinematics &kinematics,
                                                    const std::array<double, DoF> &joint_position,
                                                    const std::array<double, DoF> &joint_velocity,
                                                    const std::array<double, DoF> &joint_acceleration,
                                                    const std::array<double, 6> &external_force,
                                                    const LoadContext &load) {
  static constexpr std::array<double, 6> kGravityProxyBase = {0.4, 10.0, 7.0, 1.5, 0.8, 0.2};

  DynamicsBreakdown breakdown;
  const auto jac = jacobian(kinematics, joint_position);
  const auto external = jac.transpose() * toVector6(external_force);
  const double velocity_bias = meanAbs(toArray6(toVector6(joint_velocity)));
  const double com_norm = comNorm(load);

  for (std::size_t axis = 0; axis < 6; ++axis) {
    const double gravity_gain =
        kGravityProxyBase[axis] + load.mass * (0.15 + 0.05 * static_cast<double>(axis)) + com_norm * 0.5;
    breakdown.gravity[axis] = std::sin(joint_position[axis]) * gravity_gain;
    breakdown.coriolis[axis] =
        joint_velocity[axis] * (0.04 + 0.01 * velocity_bias + load.mass * 0.002);
    breakdown.inertia[axis] =
        joint_acceleration[axis] * (0.02 + 0.005 * static_cast<double>(axis) + load.mass * 0.004);
    breakdown.external[axis] = external(static_cast<Eigen::Index>(axis));
    breakdown.full[axis] = breakdown.gravity[axis] + breakdown.coriolis[axis] +
                           breakdown.inertia[axis] + breakdown.external[axis];
  }
  return breakdown;
}

inline std::array<double, 6> expectedTorqueProxy(::gazebo::xMate3Kinematics &kinematics,
                                                 const std::array<double, 6> &joint_position,
                                                 const std::array<double, 6> &joint_velocity,
                                                 const LoadContext &load) {
  const std::array<double, 6> zero_joint_acc{};
  const std::array<double, 6> zero_wrench{};
  return computeApproximateDynamics(
             kinematics, joint_position, joint_velocity, zero_joint_acc, zero_wrench, load)
      .full;
}

class ModelFacade {
 public:
  explicit ModelFacade(::gazebo::xMate3Kinematics &kinematics) : kinematics_(&kinematics) {}

  ModelFacade &setToolPose(const std::array<double, 6> &tool_pose) {
    tool_pose_ = tool_pose;
    return *this;
  }

  ModelFacade &setLoad(const LoadContext &load) {
    load_ = load;
    return *this;
  }

  [[nodiscard]] const LoadContext &load() const noexcept { return load_; }
  [[nodiscard]] const std::array<double, 6> &toolPose() const noexcept { return tool_pose_; }

  template <std::size_t DoF>
  [[nodiscard]] std::array<double, 6> cartPose(const std::array<double, DoF> &joint_position) const {
    return gazebo_model::cartesianPose(joint_position, *kinematics_, tool_pose_);
  }

  template <std::size_t DoF>
  [[nodiscard]] std::array<double, 6> cartVelocity(const std::array<double, DoF> &joint_position,
                                                   const std::array<double, DoF> &joint_velocity) const {
    return gazebo_model::cartesianVelocity(*kinematics_, joint_position, joint_velocity);
  }

  template <std::size_t DoF>
  [[nodiscard]] std::array<double, 6> cartAcceleration(const std::array<double, DoF> &joint_position,
                                                       const std::array<double, DoF> &joint_velocity,
                                                       const std::array<double, DoF> &joint_acceleration) const {
    return gazebo_model::cartesianAcceleration(*kinematics_, joint_position, joint_velocity, joint_acceleration);
  }

  template <std::size_t DoF>
  [[nodiscard]] std::array<double, DoF> jointAcceleration(
      const std::array<double, 6> &cartesian_acceleration,
      const std::array<double, DoF> &joint_position) const {
    return gazebo_model::jointAccelerationFromCartesian(*kinematics_, joint_position, cartesian_acceleration);
  }

  template <std::size_t DoF>
  [[nodiscard]] Matrix6d jacobian(const std::array<double, DoF> &joint_position) const {
    return gazebo_model::jacobian(*kinematics_, joint_position);
  }

  template <std::size_t DoF>
  [[nodiscard]] DynamicsBreakdown dynamics(const std::array<double, DoF> &joint_position,
                                           const std::array<double, DoF> &joint_velocity,
                                           const std::array<double, DoF> &joint_acceleration,
                                           const std::array<double, 6> &external_force = {}) const {
    return gazebo_model::computeApproximateDynamics(
        *kinematics_, joint_position, joint_velocity, joint_acceleration, external_force, load_);
  }

  [[nodiscard]] std::array<double, 6> expectedTorque(const std::array<double, 6> &joint_position,
                                                     const std::array<double, 6> &joint_velocity) const {
    return gazebo_model::expectedTorqueProxy(*kinematics_, joint_position, joint_velocity, load_);
  }

 private:
  ::gazebo::xMate3Kinematics *kinematics_;
  LoadContext load_{};
  std::array<double, 6> tool_pose_{};
};

inline ModelFacade configuredModelFacade(::gazebo::xMate3Kinematics &kinematics,
                                         const std::array<double, 6> &tool_pose,
                                         const LoadContext &load) {
  ModelFacade facade(kinematics);
  return facade.setToolPose(tool_pose).setLoad(load);
}

}  // namespace rokae_xmate3_ros2::gazebo_model

#endif  // ROKAE_XMATE3_ROS2_GAZEBO_APPROXIMATE_MODEL_HPP
