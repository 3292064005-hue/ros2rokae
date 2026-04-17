#include "runtime/planning_utils.hpp"

#include <algorithm>
#include <cmath>
#include <string_view>

namespace rokae_xmate3_ros2::runtime {
namespace {

bool contains_token(std::string_view haystack, std::string_view needle) {
  return haystack.find(needle) != std::string_view::npos;
}

}  // namespace

double max_joint_step(const std::vector<double> &lhs, const std::vector<double> &rhs) {
  double max_step = 0.0;
  for (size_t i = 0; i < 6 && i < lhs.size() && i < rhs.size(); ++i) {
    max_step = std::max(max_step, std::fabs(lhs[i] - rhs[i]));
  }
  return max_step;
}

bool build_joint_trajectory_from_cartesian(
    ::gazebo::xMate3Kinematics &kinematics,
    const std::vector<std::vector<double>> &cartesian_trajectory,
    const std::vector<double> &initial_seed,
    const std::vector<int> &requested_conf,
    bool strict_conf,
    bool avoid_singularity,
    bool soft_limit_enabled,
    const std::array<std::array<double, 2>, 6> &soft_limits,
    std::vector<std::vector<double>> &joint_trajectory,
    std::vector<double> &last_joints,
    std::string &error_message) {
  ::gazebo::xMate3Kinematics::CartesianIkOptions options;
  options.requested_conf = requested_conf;
  options.strict_conf = strict_conf;
  options.avoid_singularity = avoid_singularity;
  options.soft_limit_enabled = soft_limit_enabled;
  options.soft_limits = soft_limits;
  return kinematics.buildCartesianJointTrajectory(
      cartesian_trajectory, initial_seed, options, joint_trajectory, last_joints, error_message);
}

bool project_joint_derivatives_from_cartesian(
    ::gazebo::xMate3Kinematics &kinematics,
    const std::vector<std::vector<double>> &cartesian_trajectory,
    const std::vector<std::vector<double>> &joint_trajectory,
    double trajectory_dt,
    std::vector<std::vector<double>> &joint_velocity_trajectory,
    std::vector<std::vector<double>> &joint_acceleration_trajectory) {
  return kinematics.projectCartesianJointDerivatives(
      cartesian_trajectory,
      joint_trajectory,
      trajectory_dt,
      joint_velocity_trajectory,
      joint_acceleration_trajectory);
}

std::string classify_motion_failure_reason(std::string_view message) {
  if (message.empty()) {
    return "unreachable_pose";
  }
  if (contains_token(message, "runtime is busy") || contains_token(message, "planner queue is busy")) {
    return "runtime_busy";
  }
  if (contains_token(message, "shutdown")) {
    return "shutdown_in_progress";
  }
  if (contains_token(message, "soft limit")) {
    return "soft_limit_violation";
  }
  if (contains_token(message, "continuity")) {
    return "continuity_risk_high";
  }
  if (contains_token(message, "branch")) {
    return "branch_switch_risk_high";
  }
  if (contains_token(message, "retim")) {
    return "retime_failed";
  }
  if (contains_token(message, "confData") || contains_token(message, "requested conf")) {
    return "conf_mismatch";
  }
  if (contains_token(message, "singular")) {
    return "near_singularity_rejected";
  }
  return "unreachable_pose";
}

std::string format_motion_failure(std::string_view reason, std::string_view detail) {
  std::string message = "[";
  message += reason.empty() ? std::string{"unreachable_pose"} : std::string{reason};
  message += "]";
  if (!detail.empty()) {
    message += " ";
    message += detail;
  }
  return message;
}

}  // namespace rokae_xmate3_ros2::runtime
