#ifndef ROKAE_XMATE3_ROS2_MODEL_FACADE_WRAPPER_HPP
#define ROKAE_XMATE3_ROS2_MODEL_FACADE_WRAPPER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rokae_xmate3_ros2/runtime/kinematics_provider_interface.hpp"

namespace rokae_xmate3_ros2::model_facade {

using Matrix4d = Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

struct ModelLoadContext {
  double mass = 0.0;
  std::array<double, 3> com{{0.0, 0.0, 0.0}};
};

struct ModelDynamicsBreakdown {
  std::array<double, 6> full{};
  std::array<double, 6> gravity{};
  std::array<double, 6> coriolis{};
  std::array<double, 6> inertia{};
  std::array<double, 6> external{};
};

struct ModelDiagnostics {
  ModelLoadContext load{};
  std::array<double, 6> tool_pose{};
  double effective_payload = 0.0;
  bool uses_approximate_jacobian = true;
  bool uses_simplified_inertia = true;
};

[[nodiscard]] inline std::vector<double> toJointVector(const std::array<double, 6> &values) {
  return std::vector<double>(values.begin(), values.end());
}

[[nodiscard]] inline Vector6d toVector6(const std::array<double, 6> &values) {
  Vector6d out = Vector6d::Zero();
  for (std::size_t i = 0; i < values.size(); ++i) {
    out(static_cast<Eigen::Index>(i)) = values[i];
  }
  return out;
}

[[nodiscard]] inline std::array<double, 6> toArray6(const Vector6d &values) {
  std::array<double, 6> out{};
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = values(static_cast<Eigen::Index>(i));
  }
  return out;
}

[[nodiscard]] inline double comNorm(const ModelLoadContext &load) noexcept {
  return std::sqrt(load.com[0] * load.com[0] + load.com[1] * load.com[1] + load.com[2] * load.com[2]);
}

[[nodiscard]] inline Matrix4d poseToMatrix(const std::array<double, 6> &pose) {
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

[[nodiscard]] inline std::array<double, 6> matrixToPose(const Matrix4d &matrix) {
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
  for (std::size_t i = 3; i < 6; ++i) {
    while (pose[i] > kPi) {
      pose[i] -= 2.0 * kPi;
    }
    while (pose[i] < -kPi) {
      pose[i] += 2.0 * kPi;
    }
  }
  return pose;
}

[[nodiscard]] inline bool hasToolOffset(const std::array<double, 6> &tool_pose) noexcept {
  return std::any_of(tool_pose.begin(), tool_pose.end(), [](double value) { return std::fabs(value) > 1e-12; });
}

/**
 * @brief Provider-owned public model facade for the xMateER3 compatibility lane.
 *
 * Public boundary behavior:
 * - does not include, re-export, or store the Gazebo model facade type;
 * - consumes the backend-neutral runtime kinematics Provider contract directly;
 * - remains simulation-grade and approximate for dynamics/wrench values;
 * - does not promise hardware-model parity or a future hardware backend contract.
 */
class ModelFacade {
 public:
  ModelFacade(rokae_xmate3_ros2::kinematics::Provider &provider,
              const std::array<double, 6> &tool_pose = {},
              const ModelLoadContext &load = {})
      : provider_(&provider), load_(load), tool_pose_(tool_pose) {}

  [[nodiscard]] std::array<double, 6> cartPose(const std::array<double, 6> &joint_position) const {
    auto native = provider().forwardKinematicsRPY(toJointVector(joint_position));
    std::array<double, 6> pose{};
    for (std::size_t i = 0; i < pose.size() && i < native.size(); ++i) {
      pose[i] = native[i];
    }
    if (!hasToolOffset(tool_pose_)) {
      return pose;
    }
    return matrixToPose(poseToMatrix(pose) * poseToMatrix(tool_pose_));
  }

  [[nodiscard]] std::array<double, 6> cartVelocity(const std::array<double, 6> &joint_position,
                                                   const std::array<double, 6> &joint_velocity) const {
    return toArray6(jacobian(joint_position) * toVector6(joint_velocity));
  }

  [[nodiscard]] std::array<double, 6> cartAcceleration(const std::array<double, 6> &joint_position,
                                                       const std::array<double, 6> &,
                                                       const std::array<double, 6> &joint_acceleration) const {
    return toArray6(jacobian(joint_position) * toVector6(joint_acceleration));
  }

  [[nodiscard]] std::array<double, 6> jointAcceleration(const std::array<double, 6> &cartesian_acceleration,
                                                        const std::array<double, 6> &joint_position) const {
    const auto solved = jacobian(joint_position).completeOrthogonalDecomposition().solve(toVector6(cartesian_acceleration));
    return toArray6(solved);
  }

  [[nodiscard]] Matrix6d jacobian(const std::array<double, 6> &joint_position) const {
    return provider().computeJacobian(toJointVector(joint_position));
  }

  [[nodiscard]] Matrix6d massMatrix(const std::array<double, 6> &joint_position) const {
    Matrix6d matrix = Matrix6d::Zero();
    const double com_norm = comNorm(load_);
    for (std::size_t axis = 0; axis < 6; ++axis) {
      matrix(static_cast<Eigen::Index>(axis), static_cast<Eigen::Index>(axis)) =
          0.02 + 0.005 * static_cast<double>(axis) + load_.mass * 0.004 +
          0.001 * std::fabs(std::sin(joint_position[axis])) + com_norm * 0.002;
    }
    return matrix;
  }

  [[nodiscard]] std::array<double, 6> coriolis(const std::array<double, 6> &joint_position,
                                               const std::array<double, 6> &joint_velocity) const {
    std::array<double, 6> out{};
    const double velocity_scale = 0.01 + 0.001 * load_.mass;
    for (std::size_t i = 0; i < out.size(); ++i) {
      out[i] = velocity_scale * joint_velocity[i] * std::fabs(joint_velocity[i]) +
               0.001 * std::sin(joint_position[i]) * std::fabs(joint_velocity[i]);
    }
    return out;
  }

  [[nodiscard]] std::array<double, 6> gravity(const std::array<double, 6> &joint_position) const {
    std::array<double, 6> out{};
    const double payload_scale = 1.0 + std::max(0.0, load_.mass) * 0.08 + comNorm(load_) * 0.04;
    for (std::size_t i = 0; i < out.size(); ++i) {
      out[i] = payload_scale * (0.18 + 0.03 * static_cast<double>(i)) * std::sin(joint_position[i]);
    }
    return out;
  }

  [[nodiscard]] std::array<double, 6> inverseDynamics(const std::array<double, 6> &joint_position,
                                                      const std::array<double, 6> &joint_velocity,
                                                      const std::array<double, 6> &joint_acceleration,
                                                      const std::array<double, 6> &external_force = {}) const {
    return dynamics(joint_position, joint_velocity, joint_acceleration, external_force).full;
  }

  [[nodiscard]] ModelDynamicsBreakdown dynamics(const std::array<double, 6> &joint_position,
                                                const std::array<double, 6> &joint_velocity,
                                                const std::array<double, 6> &joint_acceleration,
                                                const std::array<double, 6> &external_force = {}) const {
    ModelDynamicsBreakdown out;
    out.gravity = gravity(joint_position);
    out.coriolis = coriolis(joint_position, joint_velocity);
    out.external = external_force;
    const auto inertia_vec = massMatrix(joint_position) * toVector6(joint_acceleration);
    out.inertia = toArray6(inertia_vec);
    for (std::size_t i = 0; i < out.full.size(); ++i) {
      out.full[i] = out.gravity[i] + out.coriolis[i] + out.inertia[i] + out.external[i];
    }
    return out;
  }

  [[nodiscard]] std::array<double, 6> expectedTorque(const std::array<double, 6> &joint_position,
                                                     const std::array<double, 6> &joint_velocity) const {
    const std::array<double, 6> zero_acc{};
    const std::array<double, 6> zero_external{};
    return inverseDynamics(joint_position, joint_velocity, zero_acc, zero_external);
  }

  [[nodiscard]] ModelDiagnostics diagnostics() const {
    ModelDiagnostics diagnostics;
    diagnostics.load = load_;
    diagnostics.tool_pose = tool_pose_;
    diagnostics.effective_payload = load_.mass + comNorm(load_) * 0.1;
    diagnostics.uses_approximate_jacobian = true;
    diagnostics.uses_simplified_inertia = true;
    return diagnostics;
  }

 private:
  [[nodiscard]] rokae_xmate3_ros2::kinematics::Provider &provider() const {
    if (provider_ == nullptr) {
      throw std::logic_error("model facade provider is not configured");
    }
    return *provider_;
  }

  rokae_xmate3_ros2::kinematics::Provider *provider_ = nullptr;
  ModelLoadContext load_{};
  std::array<double, 6> tool_pose_{};
};

[[nodiscard]] inline ModelFacade makeModelFacade(rokae_xmate3_ros2::kinematics::Provider &kinematics,
                                                 const std::array<double, 6> &tool_pose = {},
                                                 const ModelLoadContext &load = {}) {
  return ModelFacade(kinematics, tool_pose, load);
}

}  // namespace rokae_xmate3_ros2::model_facade

#endif  // ROKAE_XMATE3_ROS2_MODEL_FACADE_WRAPPER_HPP
