#ifndef ROKAE_XMATE3_ROS2_RUNTIME_KINEMATICS_PROVIDER_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_KINEMATICS_PROVIDER_HPP

#include <memory>
#include <utility>

#include "rokae_xmate3_ros2/runtime/kinematics_provider_interface.hpp"
#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"

namespace rokae_xmate3_ros2::kinematics {

class GazeboProvider final : public Provider {
 public:
  explicit GazeboProvider(::gazebo::xMateER3Kinematics &kinematics) : kinematics_(&kinematics) {}

  [[nodiscard]] std::vector<double> forwardKinematicsRPY(const std::vector<double> &joints) override {
    return kinematics_->forwardKinematicsRPY(joints);
  }

  [[nodiscard]] std::vector<double> inverseKinematics(const std::vector<double> &target,
                                                      const std::vector<double> &current_joints) override {
    return kinematics_->inverseKinematics(target, current_joints);
  }

  [[nodiscard]] std::vector<std::vector<double>> inverseKinematicsMultiSolution(
      const std::vector<double> &target,
      const std::vector<double> &current_joints) override {
    return kinematics_->inverseKinematicsMultiSolution(target, current_joints);
  }

  [[nodiscard]] std::vector<double> inverseKinematicsSeededFast(const std::vector<double> &target,
                                                                const std::vector<double> &seed_joints) override {
    return kinematics_->inverseKinematicsSeededFast(target, seed_joints);
  }

  [[nodiscard]] Matrix6d computeJacobian(const std::vector<double> &joints) override {
    return kinematics_->computeJacobian(joints);
  }

  [[nodiscard]] IkSelectionResult selectBestIkSolution(const std::vector<std::vector<double>> &candidates,
                                                       const std::vector<double> &target_pose,
                                                       const std::vector<double> &seed_joints,
                                                       const CartesianIkOptions &options) override {
    ::gazebo::xMateER3Kinematics::CartesianIkOptions native_options;
    native_options.requested_conf = options.requested_conf;
    native_options.strict_conf = options.strict_conf;
    native_options.avoid_singularity = options.avoid_singularity;
    native_options.soft_limit_enabled = options.soft_limit_enabled;
    native_options.soft_limits = options.soft_limits;
    const auto native_result = kinematics_->selectBestIkSolution(candidates, target_pose, seed_joints, native_options);
    IkSelectionResult result;
    result.success = native_result.success;
    result.joints = native_result.joints;
    result.branch_id = native_result.branch_id;
    result.note = native_result.note;
    result.message = native_result.message;
    return result;
  }

  [[nodiscard]] bool buildCartesianJointTrajectory(const std::vector<std::vector<double>> &cartesian_trajectory,
                                                   const std::vector<double> &initial_seed,
                                                   const CartesianIkOptions &options,
                                                   std::vector<std::vector<double>> &joint_trajectory,
                                                   std::vector<double> &last_joints,
                                                   std::string &error_message) override {
    ::gazebo::xMateER3Kinematics::CartesianIkOptions native_options;
    native_options.requested_conf = options.requested_conf;
    native_options.strict_conf = options.strict_conf;
    native_options.avoid_singularity = options.avoid_singularity;
    native_options.soft_limit_enabled = options.soft_limit_enabled;
    native_options.soft_limits = options.soft_limits;
    return kinematics_->buildCartesianJointTrajectory(
        cartesian_trajectory, initial_seed, native_options, joint_trajectory, last_joints, error_message);
  }

  [[nodiscard]] bool projectCartesianJointDerivatives(
      const std::vector<std::vector<double>> &cartesian_trajectory,
      const std::vector<std::vector<double>> &joint_trajectory,
      double trajectory_dt,
      std::vector<std::vector<double>> &joint_velocity_trajectory,
      std::vector<std::vector<double>> &joint_acceleration_trajectory) override {
    return kinematics_->projectCartesianJointDerivatives(
        cartesian_trajectory,
        joint_trajectory,
        trajectory_dt,
        joint_velocity_trajectory,
        joint_acceleration_trajectory);
  }

  void beginRequestContract(const std::string &request_id) const override {
    kinematics_->beginRequestContract(request_id);
  }

  void endRequestContract() const override {
    kinematics_->endRequestContract();
  }

  [[nodiscard]] RequestContractState requestContractState() const override {
    const auto native_state = kinematics_->requestContractState();
    RequestContractState state;
    state.active = native_state.active;
    state.violated = native_state.violated;
    state.request_id = native_state.request_id;
    state.locked_primary_backend = native_state.locked_primary_backend;
    state.locked_fallback_backend = native_state.locked_fallback_backend;
    state.violation_reason = native_state.violation_reason;
    state.last_request_kind = native_state.last_request_kind;
    state.last_fallback_reason = native_state.last_fallback_reason;
    state.last_seed_source = native_state.last_seed_source;
    return state;
  }

  [[nodiscard]] RequestTrace lastTrace() const override {
    const auto native_trace = kinematics_->lastTrace();
    RequestTrace trace;
    trace.request_kind = native_trace.request_kind;
    trace.primary_backend = native_trace.primary_backend;
    trace.fallback_backend = native_trace.fallback_backend;
    trace.fallback_used = native_trace.fallback_used;
    trace.fallback_reason = native_trace.fallback_reason;
    trace.seed_source = native_trace.seed_source;
    trace.selected_branch = native_trace.selected_branch;
    trace.note = native_trace.note;
    trace.continuity_cost = native_trace.continuity_cost;
    trace.singularity_metric = native_trace.singularity_metric;
    return trace;
  }

 private:
  ::gazebo::xMateER3Kinematics *kinematics_ = nullptr;
};

class OwnedGazeboProvider final : public Provider {
 public:
  OwnedGazeboProvider() : owned_(std::make_unique<::gazebo::xMateER3Kinematics>()), adapter_(*owned_) {}

  [[nodiscard]] std::vector<double> forwardKinematicsRPY(const std::vector<double> &joints) override {
    return adapter_.forwardKinematicsRPY(joints);
  }

  [[nodiscard]] std::vector<double> inverseKinematics(const std::vector<double> &pose,
                                                      const std::vector<double> &seed) override {
    return adapter_.inverseKinematics(pose, seed);
  }

  [[nodiscard]] std::vector<std::vector<double>> inverseKinematicsMultiSolution(
      const std::vector<double> &pose,
      const std::vector<double> &seed) override {
    return adapter_.inverseKinematicsMultiSolution(pose, seed);
  }

  [[nodiscard]] std::vector<double> inverseKinematicsSeededFast(const std::vector<double> &pose,
                                                                const std::vector<double> &seed) override {
    return adapter_.inverseKinematicsSeededFast(pose, seed);
  }

  [[nodiscard]] Matrix6d computeJacobian(const std::vector<double> &joints) override {
    return adapter_.computeJacobian(joints);
  }

  [[nodiscard]] IkSelectionResult selectBestIkSolution(const std::vector<std::vector<double>> &candidates,
                                                       const std::vector<double> &target_pose,
                                                       const std::vector<double> &current_joints,
                                                       const CartesianIkOptions &options) override {
    return adapter_.selectBestIkSolution(candidates, target_pose, current_joints, options);
  }

  [[nodiscard]] bool buildCartesianJointTrajectory(const std::vector<std::vector<double>> &cartesian_trajectory,
                                                   const std::vector<double> &initial_seed,
                                                   const CartesianIkOptions &options,
                                                   std::vector<std::vector<double>> &joint_trajectory,
                                                   std::vector<double> &last_joints,
                                                   std::string &error_message) override {
    return adapter_.buildCartesianJointTrajectory(
        cartesian_trajectory, initial_seed, options, joint_trajectory, last_joints, error_message);
  }

  [[nodiscard]] bool projectCartesianJointDerivatives(
      const std::vector<std::vector<double>> &cartesian_trajectory,
      const std::vector<std::vector<double>> &joint_trajectory,
      double trajectory_dt,
      std::vector<std::vector<double>> &joint_velocity_trajectory,
      std::vector<std::vector<double>> &joint_acceleration_trajectory) override {
    return adapter_.projectCartesianJointDerivatives(
        cartesian_trajectory, joint_trajectory, trajectory_dt, joint_velocity_trajectory, joint_acceleration_trajectory);
  }

  void beginRequestContract(const std::string &request_id) const override { adapter_.beginRequestContract(request_id); }
  void endRequestContract() const override { adapter_.endRequestContract(); }
  [[nodiscard]] RequestContractState requestContractState() const override { return adapter_.requestContractState(); }
  [[nodiscard]] RequestTrace lastTrace() const override { return adapter_.lastTrace(); }

 private:
  std::unique_ptr<::gazebo::xMateER3Kinematics> owned_;
  GazeboProvider adapter_;
};

}  // namespace rokae_xmate3_ros2::kinematics

#endif
