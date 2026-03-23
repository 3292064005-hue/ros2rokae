#ifndef ROKAE_XMATE3_ROS2_RUNTIME_PLANNING_UTILS_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_PLANNING_UTILS_HPP

#include <array>
#include <string>
#include <vector>

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"

namespace rokae_xmate3_ros2::runtime {

struct IkSelection {
  bool success = false;
  std::vector<double> joints;
  std::string message;
};

[[nodiscard]] double joint_branch_jump_threshold() noexcept;

[[nodiscard]] double max_joint_step(const std::vector<double> &lhs,
                                    const std::vector<double> &rhs);

[[nodiscard]] IkSelection select_ik_solution(
    ::gazebo::xMate3Kinematics &kinematics,
    const std::vector<std::vector<double>> &candidates,
    const std::vector<double> &target_pose,
    const std::vector<double> &seed_joints,
    const std::vector<int> &requested_conf,
    bool strict_conf,
    bool avoid_singularity,
    bool soft_limit_enabled,
    const std::array<std::array<double, 2>, 6> &soft_limits);

[[nodiscard]] bool build_joint_trajectory_from_cartesian(
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
    std::string &error_message);

[[nodiscard]] bool project_joint_derivatives_from_cartesian(
    ::gazebo::xMate3Kinematics &kinematics,
    const std::vector<std::vector<double>> &cartesian_trajectory,
    const std::vector<std::vector<double>> &joint_trajectory,
    double trajectory_dt,
    std::vector<std::vector<double>> &joint_velocity_trajectory,
    std::vector<std::vector<double>> &joint_acceleration_trajectory);

}  // namespace rokae_xmate3_ros2::runtime

#endif
