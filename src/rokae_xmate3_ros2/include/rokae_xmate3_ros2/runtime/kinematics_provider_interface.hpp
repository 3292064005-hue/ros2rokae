#ifndef ROKAE_XMATE3_ROS2_RUNTIME_KINEMATICS_PROVIDER_INTERFACE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_KINEMATICS_PROVIDER_INTERFACE_HPP

#include <array>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rokae_xmate3_ros2/spec/xmate_er3_truth.hpp"

namespace rokae_xmate3_ros2::kinematics {

using Matrix6d = Eigen::Matrix<double, 6, 6>;

struct CartesianIkOptions {
  std::vector<int> requested_conf;
  bool strict_conf = false;
  bool avoid_singularity = true;
  bool soft_limit_enabled = false;
  std::array<std::array<double, 2>, 6> soft_limits = rokae_xmate3_ros2::spec::xmate_er3_truth::kDefaultSoftLimits;
};

struct IkSelectionResult {
  bool success = false;
  std::vector<double> joints;
  std::string branch_id;
  std::string note;
  std::string message;
};

struct RequestTrace {
  std::string request_kind{"idle"};
  std::string primary_backend{"none"};
  std::string fallback_backend{"none"};
  bool fallback_used = false;
  std::string fallback_reason;
  std::string seed_source;
  std::string selected_branch;
  std::string note;
  double continuity_cost = std::numeric_limits<double>::infinity();
  double singularity_metric = 1.0;
};

struct RequestContractState {
  bool active = false;
  bool violated = false;
  std::string request_id;
  std::string locked_primary_backend{"none"};
  std::string locked_fallback_backend{"none"};
  std::string violation_reason;
  std::string last_request_kind;
  std::string last_fallback_reason;
  std::string last_seed_source;
};

/**
 * @brief Backend-neutral kinematics provider contract used by public model helpers and runtime planning code.
 *
 * This header intentionally contains only pure data contracts and the abstract provider interface. It must not
 * include Gazebo headers or expose concrete simulation provider types. Concrete providers live in implementation
 * headers or compiled translation units that explicitly opt into the selected runtime backend.
 */
class Provider {
 public:
  virtual ~Provider() = default;

  [[nodiscard]] virtual std::vector<double> forwardKinematicsRPY(const std::vector<double> &joints) = 0;
  [[nodiscard]] virtual std::vector<double> inverseKinematics(const std::vector<double> &target,
                                                              const std::vector<double> &current_joints) = 0;
  [[nodiscard]] virtual std::vector<std::vector<double>> inverseKinematicsMultiSolution(
      const std::vector<double> &target,
      const std::vector<double> &current_joints) = 0;
  [[nodiscard]] virtual std::vector<double> inverseKinematicsSeededFast(const std::vector<double> &target,
                                                                        const std::vector<double> &seed_joints) = 0;
  [[nodiscard]] virtual Matrix6d computeJacobian(const std::vector<double> &joints) = 0;
  [[nodiscard]] virtual IkSelectionResult selectBestIkSolution(
      const std::vector<std::vector<double>> &candidates,
      const std::vector<double> &target_pose,
      const std::vector<double> &seed_joints,
      const CartesianIkOptions &options) = 0;
  [[nodiscard]] virtual bool buildCartesianJointTrajectory(
      const std::vector<std::vector<double>> &cartesian_trajectory,
      const std::vector<double> &initial_seed,
      const CartesianIkOptions &options,
      std::vector<std::vector<double>> &joint_trajectory,
      std::vector<double> &last_joints,
      std::string &error_message) = 0;
  [[nodiscard]] virtual bool projectCartesianJointDerivatives(
      const std::vector<std::vector<double>> &cartesian_trajectory,
      const std::vector<std::vector<double>> &joint_trajectory,
      double trajectory_dt,
      std::vector<std::vector<double>> &joint_velocity_trajectory,
      std::vector<std::vector<double>> &joint_acceleration_trajectory) = 0;
  virtual void beginRequestContract(const std::string &request_id) const = 0;
  virtual void endRequestContract() const = 0;
  [[nodiscard]] virtual RequestContractState requestContractState() const = 0;
  [[nodiscard]] virtual RequestTrace lastTrace() const = 0;
};

}  // namespace rokae_xmate3_ros2::kinematics

#endif  // ROKAE_XMATE3_ROS2_RUNTIME_KINEMATICS_PROVIDER_INTERFACE_HPP
