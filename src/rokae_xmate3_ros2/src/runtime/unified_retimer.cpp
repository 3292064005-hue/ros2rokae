#include "runtime/unified_retimer.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

#include "runtime/planning_utils.hpp"
#include "runtime/pose_utils.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

std::array<double, 6> uniform_axis_limits(double value, double minimum_limit) {
  std::array<double, 6> limits{};
  const double resolved = std::max(value, minimum_limit);
  limits.fill(resolved);
  return limits;
}

std::array<std::array<double, 2>, 6> disabled_soft_limits() {
  std::array<std::array<double, 2>, 6> limits{};
  for (auto &axis_limits : limits) {
    axis_limits = {{-1.0e9, 1.0e9}};
  }
  return limits;
}

double resolve_cartesian_speed_mm_per_s(const rokae_xmate3_ros2::srv::GenerateSTrajectory::Request &request,
                                        const std::vector<double> &start_pose,
                                        const std::vector<double> &target_pose) {
  const auto planner_config = ::gazebo::TrajectoryPlanner::config();
  const Eigen::Vector3d start(start_pose[0], start_pose[1], start_pose[2]);
  const Eigen::Vector3d target(target_pose[0], target_pose[1], target_pose[2]);
  const double path_length = (target - start).norm();
  const double smoothing_scale = 1.0 / (1.0 + 0.25 * std::clamp(request.blend_radius, 0.0, 1.0));
  const double velocity_limit_mps = std::max(request.max_velocity, 0.05) * smoothing_scale;
  const double acceleration_limit_mps2 = std::max(request.max_acceleration, 0.05);
  const double acceleration_bounded_speed =
      path_length > 1e-9 ? std::sqrt(std::max(acceleration_limit_mps2 * path_length, 0.0)) : velocity_limit_mps;
  const double speed_mps = std::clamp(
      std::min(velocity_limit_mps, std::max(acceleration_bounded_speed, 0.05)),
      planner_config.min_nrt_speed_mm_per_s / 1000.0,
      planner_config.max_nrt_speed_mm_per_s / 1000.0);
  return speed_mps * 1000.0;
}

}  // namespace

JointRetimerConfig makeUnifiedRetimerConfig(double sample_dt) {
  const auto planner_config = ::gazebo::TrajectoryPlanner::config();
  JointRetimerConfig config;
  config.joint_speed_limits_rad_per_sec = planner_config.joint_speed_limits_rad_per_sec;
  config.joint_acc_limits_rad_per_sec2 = planner_config.joint_acc_limits_rad_per_sec2;
  config.sample_dt = std::max(sample_dt, 1e-3);
  config.min_sample_dt = planner_config.min_sample_dt;
  config.max_sample_dt = planner_config.max_sample_dt;
  config.max_joint_step_rad = planner_config.max_joint_step_rad;
  return config;
}

UnifiedRetimerLimits makeUnifiedRetimerLimits(double max_velocity,
                                              double max_acceleration,
                                              double blend_radius) {
  constexpr double kPi = 3.14159265358979323846;
  const double blend_window = std::clamp(blend_radius, 0.0, 1.0);
  const double smoothing_scale = 1.0 / (1.0 + 0.25 * blend_window);

  UnifiedRetimerLimits limits;
  limits.velocity_limits =
      uniform_axis_limits(std::clamp(max_velocity > 0.0 ? max_velocity : kPi, 0.05, kPi) * smoothing_scale, 0.05);
  limits.acceleration_limits = uniform_axis_limits(
      std::clamp(max_acceleration > 0.0 ? max_acceleration : 2.0 * kPi, 0.05, 10.0 * kPi) * smoothing_scale *
          smoothing_scale,
      0.05);
  return limits;
}

QuinticRetimerResult retimeJointWithUnifiedConfig(const std::vector<double> &start,
                                                  const std::vector<double> &target,
                                                  double sample_dt,
                                                  double max_velocity,
                                                  double max_acceleration,
                                                  double blend_radius) {
  const auto config = makeUnifiedRetimerConfig(sample_dt);
  const auto limits = makeUnifiedRetimerLimits(max_velocity, max_acceleration, blend_radius);
  return retimeJointQuintic(
      start, target, config, limits.velocity_limits, limits.acceleration_limits);
}

ApproximateCartesianRetimerResult buildApproximateCartesianSTrajectory(
    ::gazebo::xMate3Kinematics &kinematics,
    const rokae_xmate3_ros2::srv::GenerateSTrajectory::Request &request,
    double sample_dt) {
  ApproximateCartesianRetimerResult result;

  std::vector<double> start(request.start_joint_pos.begin(), request.start_joint_pos.end());
  std::vector<double> target(request.target_joint_pos.begin(), request.target_joint_pos.end());
  const auto start_pose = kinematics.forwardKinematicsRPY(start);
  const auto target_pose = kinematics.forwardKinematicsRPY(target);
  const double speed_mm_per_s = resolve_cartesian_speed_mm_per_s(request, start_pose, target_pose);
  const auto cartesian_samples =
      ::gazebo::TrajectoryPlanner::planCartesianLine(start_pose, target_pose, speed_mm_per_s, sample_dt);
  if (cartesian_samples.empty()) {
    result.error_message = "cartesian path generation failed";
    return result;
  }

  std::vector<double> last_joints;
  if (!build_joint_trajectory_from_cartesian(kinematics,
                                             cartesian_samples.points,
                                             start,
                                             {},
                                             false,
                                             true,
                                             false,
                                             disabled_soft_limits(),
                                             result.joint_trajectory,
                                             last_joints,
                                             result.error_message)) {
    return result;
  }

  if (result.joint_trajectory.empty()) {
    result.error_message = "cartesian joint sampling produced no trajectory points";
    return result;
  }

  result.joint_trajectory.front() = start;
  result.joint_trajectory.back() = target;
  result.total_time = cartesian_samples.total_time;
  return result;
}

}  // namespace rokae_xmate3_ros2::runtime
