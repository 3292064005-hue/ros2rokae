#include "runtime/planning_utils.hpp"

#include <algorithm>
#include <cmath>

#include "runtime/pose_utils.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr double kJointBranchJumpThreshold = 0.75;

Eigen::Matrix<double, 6, 1> cartesian_twist_from_samples(const std::vector<double> &previous_pose,
                                                         const std::vector<double> &next_pose,
                                                         double dt) {
  Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
  if (dt <= 1e-9) {
    return twist;
  }

  const auto previous_tf = pose_utils::poseToIsometry(previous_pose);
  const auto next_tf = pose_utils::poseToIsometry(next_pose);
  twist.head<3>() = (next_tf.translation() - previous_tf.translation()) / dt;

  const Eigen::Matrix3d rotation_delta = next_tf.linear() * previous_tf.linear().transpose();
  const Eigen::AngleAxisd angle_axis(rotation_delta);
  Eigen::Vector3d angular_delta = angle_axis.angle() * angle_axis.axis();
  if (!std::isfinite(angular_delta.norm()) || angle_axis.angle() < 1e-8) {
    angular_delta << rotation_delta(2, 1) - rotation_delta(1, 2),
        rotation_delta(0, 2) - rotation_delta(2, 0),
        rotation_delta(1, 0) - rotation_delta(0, 1);
    angular_delta *= 0.5;
  }
  twist.tail<3>() = angular_delta / dt;
  return twist;
}

bool is_finite_vector(const Eigen::Matrix<double, 6, 1> &vector) {
  for (int index = 0; index < vector.size(); ++index) {
    if (!std::isfinite(vector(index))) {
      return false;
    }
  }
  return true;
}

}  // namespace

double joint_branch_jump_threshold() noexcept { return kJointBranchJumpThreshold; }

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
  joint_velocity_trajectory.clear();
  joint_acceleration_trajectory.clear();

  if (cartesian_trajectory.size() != joint_trajectory.size() || joint_trajectory.empty() || trajectory_dt <= 1e-9) {
    return false;
  }

  const std::size_t point_count = joint_trajectory.size();
  const std::size_t axis_count = joint_trajectory.front().size();
  joint_velocity_trajectory.assign(point_count, std::vector<double>(axis_count, 0.0));
  joint_acceleration_trajectory.assign(point_count, std::vector<double>(axis_count, 0.0));
  if (point_count == 1 || axis_count == 0) {
    return true;
  }

  for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
    const std::vector<double> *previous_pose = nullptr;
    const std::vector<double> *next_pose = nullptr;
    double twist_dt = trajectory_dt;
    if (point_index == 0) {
      previous_pose = &cartesian_trajectory[0];
      next_pose = &cartesian_trajectory[1];
    } else if (point_index + 1 >= point_count) {
      previous_pose = &cartesian_trajectory[point_index - 1];
      next_pose = &cartesian_trajectory[point_index];
    } else {
      previous_pose = &cartesian_trajectory[point_index - 1];
      next_pose = &cartesian_trajectory[point_index + 1];
      twist_dt = 2.0 * trajectory_dt;
    }

    const auto twist = cartesian_twist_from_samples(*previous_pose, *next_pose, twist_dt);
    const auto jacobian = kinematics.computeJacobian(joint_trajectory[point_index]);
    const double singularity = std::clamp(kinematics.computeSingularityMeasure(joint_trajectory[point_index]), 0.0, 1.0);
    const double damping = 1e-4 + 0.05 * singularity;
    const Eigen::Matrix<double, 6, 6> jj_t =
        jacobian * jacobian.transpose() + damping * damping * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6, 1> qd =
        jacobian.transpose() * jj_t.ldlt().solve(twist);
    if (!is_finite_vector(qd)) {
      joint_velocity_trajectory.clear();
      joint_acceleration_trajectory.clear();
      return false;
    }
    for (std::size_t axis = 0; axis < axis_count && axis < static_cast<std::size_t>(qd.size()); ++axis) {
      joint_velocity_trajectory[point_index][axis] = qd(static_cast<int>(axis));
    }
  }

  if (point_count < 2) {
    return true;
  }

  for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
    for (std::size_t axis = 0; axis < axis_count; ++axis) {
      if (point_index == 0) {
        joint_acceleration_trajectory[point_index][axis] =
            (joint_velocity_trajectory[1][axis] - joint_velocity_trajectory[0][axis]) / trajectory_dt;
      } else if (point_index + 1 >= point_count) {
        joint_acceleration_trajectory[point_index][axis] =
            (joint_velocity_trajectory[point_index][axis] - joint_velocity_trajectory[point_index - 1][axis]) /
            trajectory_dt;
      } else {
        joint_acceleration_trajectory[point_index][axis] =
            (joint_velocity_trajectory[point_index + 1][axis] - joint_velocity_trajectory[point_index - 1][axis]) /
            (2.0 * trajectory_dt);
      }
    }
  }

  return true;
}

}  // namespace rokae_xmate3_ros2::runtime
