#ifndef ROKAE_XMATE3_ROS2_GAZEBO_KINEMATICS_POLICY_HPP
#define ROKAE_XMATE3_ROS2_GAZEBO_KINEMATICS_POLICY_HPP

#include <string>

namespace gazebo {

struct KinematicsPolicy {
  enum class PrimaryBackend {
    Kdl,
    ImprovedDh,
  };

  enum class JacobianMode {
    Native,
    Geometric,
    FiniteDifferenceTestOnly,
  };

  enum class IkSeedMode {
    CurrentState,
    ImprovedDhBranches,
    Mixed,
  };

  enum class FallbackMode {
    None,
    ImprovedDh,
  };

  PrimaryBackend primary_backend = PrimaryBackend::Kdl;
  JacobianMode jacobian_mode = JacobianMode::Native;
  IkSeedMode ik_seed_mode = IkSeedMode::Mixed;
  FallbackMode fallback_mode = FallbackMode::ImprovedDh;
  bool require_single_backend_per_request = true;

  [[nodiscard]] std::string primaryBackendName() const {
    return primary_backend == PrimaryBackend::Kdl ? "kdl" : "improved_dh";
  }

  [[nodiscard]] std::string fallbackBackendName() const {
    return fallback_mode == FallbackMode::ImprovedDh ? "improved_dh" : "none";
  }
};

}  // namespace gazebo

#endif
