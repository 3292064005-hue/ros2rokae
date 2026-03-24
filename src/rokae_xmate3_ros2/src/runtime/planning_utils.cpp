#include "runtime/planning_utils.hpp"

#include <algorithm>
#include <cmath>

namespace rokae_xmate3_ros2::runtime {

double max_joint_step(const std::vector<double> &lhs, const std::vector<double> &rhs) {
  double max_step = 0.0;
  for (size_t i = 0; i < 6 && i < lhs.size() && i < rhs.size(); ++i) {
    max_step = std::max(max_step, std::fabs(lhs[i] - rhs[i]));
  }
  return max_step;
}

IkSelection select_ik_solution(
    ::gazebo::xMate3Kinematics &kinematics,
    const std::vector<std::vector<double>> &candidates,
    const std::vector<double> &target_pose,
    const std::vector<double> &seed_joints,
    const std::vector<int> &requested_conf,
    bool strict_conf,
    bool avoid_singularity,
    bool soft_limit_enabled,
    const std::array<std::array<double, 2>, 6> &soft_limits) {
  ::gazebo::xMate3Kinematics::CartesianIkOptions options;
  options.requested_conf = requested_conf;
  options.strict_conf = strict_conf;
  options.avoid_singularity = avoid_singularity;
  options.soft_limit_enabled = soft_limit_enabled;
  options.soft_limits = soft_limits;

  const auto selected = kinematics.selectBestIkSolution(candidates, target_pose, seed_joints, options);
  return {selected.success, selected.joints, selected.message};
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

}  // namespace rokae_xmate3_ros2::runtime
