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

struct ReplayVectorSample {
  double time_from_start = 0.0;
  std::vector<double> position;
  std::vector<double> velocity;
};

[[nodiscard]] double sanitize_replay_time(double requested,
                                          double fallback_time,
                                          double min_step) {
  if (!std::isfinite(requested)) {
    return fallback_time;
  }
  return std::max(requested, fallback_time - min_step);
}

[[nodiscard]] std::vector<ReplayVectorSample> normalize_replay_samples(const ReplayPathAsset &asset,
                                                                       double fallback_dt) {
  std::vector<ReplayVectorSample> samples;
  samples.reserve(asset.samples.size());

  const double min_step = std::max(fallback_dt, 1e-3);
  double next_fallback_time = 0.0;
  double last_time = -min_step;
  for (const auto &sample : asset.samples) {
    ReplayVectorSample normalized;
    normalized.position.assign(sample.joint_position.begin(), sample.joint_position.end());
    normalized.velocity.assign(sample.joint_velocity.begin(), sample.joint_velocity.end());

    double time_from_start = sanitize_replay_time(sample.time_from_start_sec, next_fallback_time, min_step);
    if (time_from_start <= last_time) {
      time_from_start = last_time + min_step;
    }
    normalized.time_from_start = time_from_start;
    next_fallback_time = time_from_start + min_step;
    last_time = time_from_start;
    samples.push_back(std::move(normalized));
  }
  return samples;
}

[[nodiscard]] std::vector<double> interpolate_joint_vector(const std::vector<ReplayVectorSample> &samples,
                                                           double time_from_start,
                                                           bool velocity_space) {
  if (samples.empty()) {
    return {};
  }
  const auto &first = velocity_space ? samples.front().velocity : samples.front().position;
  if (samples.size() == 1 || time_from_start <= samples.front().time_from_start) {
    return first;
  }
  const auto &last = velocity_space ? samples.back().velocity : samples.back().position;
  if (time_from_start >= samples.back().time_from_start) {
    return last;
  }

  auto upper = std::lower_bound(
      samples.begin(), samples.end(), time_from_start,
      [](const ReplayVectorSample &sample, double value) { return sample.time_from_start < value; });
  if (upper == samples.begin()) {
    return first;
  }
  if (upper == samples.end()) {
    return last;
  }

  const auto &rhs = *upper;
  const auto &lhs = *(upper - 1);
  const double duration = std::max(rhs.time_from_start - lhs.time_from_start, 1e-9);
  const double alpha = std::clamp((time_from_start - lhs.time_from_start) / duration, 0.0, 1.0);

  const auto &lhs_vec = velocity_space ? lhs.velocity : lhs.position;
  const auto &rhs_vec = velocity_space ? rhs.velocity : rhs.position;
  std::vector<double> out(lhs_vec.size(), 0.0);
  for (std::size_t index = 0; index < out.size() && index < rhs_vec.size(); ++index) {
    out[index] = lhs_vec[index] + alpha * (rhs_vec[index] - lhs_vec[index]);
  }
  return out;
}

void populate_acceleration_trajectory(std::vector<std::vector<double>> &accelerations,
                                      const std::vector<std::vector<double>> &velocities,
                                      double dt) {
  accelerations.clear();
  if (velocities.empty()) {
    return;
  }
  const auto point_count = velocities.size();
  const auto axis_count = velocities.front().size();
  accelerations.assign(point_count, std::vector<double>(axis_count, 0.0));
  if (point_count < 2 || dt <= 1e-9) {
    return;
  }
  for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
    for (std::size_t axis = 0; axis < axis_count; ++axis) {
      if (point_index == 0) {
        accelerations[point_index][axis] =
            (velocities[1][axis] - velocities[0][axis]) / dt;
      } else if (point_index + 1 >= point_count) {
        accelerations[point_index][axis] =
            (velocities[point_index][axis] - velocities[point_index - 1][axis]) / dt;
      } else {
        accelerations[point_index][axis] =
            (velocities[point_index + 1][axis] - velocities[point_index - 1][axis]) / (2.0 * dt);
      }
    }
  }
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

QuinticRetimerResult retimeJointWithUnifiedLimits(const std::vector<double> &start,
                                                  const std::vector<double> &target,
                                                  double sample_dt,
                                                  const std::array<double, 6> &velocity_limits,
                                                  const std::array<double, 6> &acceleration_limits) {
  const auto config = makeUnifiedRetimerConfig(sample_dt);
  return retimeJointQuintic(start, target, config, velocity_limits, acceleration_limits);
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
  result.sample_dt = sample_dt;
  if (!project_joint_derivatives_from_cartesian(kinematics,
                                                cartesian_samples.points,
                                                result.joint_trajectory,
                                                sample_dt,
                                                result.joint_velocity_trajectory,
                                                result.joint_acceleration_trajectory)) {
    result.joint_velocity_trajectory.assign(
        result.joint_trajectory.size(), std::vector<double>(result.joint_trajectory.front().size(), 0.0));
    result.joint_acceleration_trajectory.assign(
        result.joint_trajectory.size(), std::vector<double>(result.joint_trajectory.front().size(), 0.0));
  }
  result.total_time = cartesian_samples.total_time;
  return result;
}

QuinticRetimerResult retimeReplayWithUnifiedConfig(const ReplayPathAsset &asset,
                                                   double rate,
                                                   double sample_dt) {
  QuinticRetimerResult result;
  if (asset.samples.empty()) {
    return result;
  }

  const double resolved_sample_dt = std::max(sample_dt, 1e-3);
  const double rate_scale = std::max(rate, 0.05);
  const auto normalized_samples = normalize_replay_samples(asset, resolved_sample_dt);
  if (normalized_samples.empty()) {
    return result;
  }

  const double original_total_time = normalized_samples.back().time_from_start;
  const double scaled_total_time = original_total_time / rate_scale;
  const int interval_count =
      scaled_total_time > 1e-9
          ? std::max(1, static_cast<int>(std::ceil(scaled_total_time / resolved_sample_dt)))
          : 1;

  result.sample_dt =
      scaled_total_time > 1e-9 ? scaled_total_time / static_cast<double>(interval_count) : resolved_sample_dt;
  result.positions.reserve(static_cast<std::size_t>(interval_count) + 1);
  result.velocities.reserve(static_cast<std::size_t>(interval_count) + 1);

  for (int index = 0; index <= interval_count; ++index) {
    const double scaled_time =
        scaled_total_time > 1e-9 ? result.sample_dt * static_cast<double>(index) : 0.0;
    const double original_time = std::min(scaled_time * rate_scale, original_total_time);

    auto position = interpolate_joint_vector(normalized_samples, original_time, false);
    auto velocity = interpolate_joint_vector(normalized_samples, original_time, true);
    if (velocity.size() < position.size()) {
      velocity.resize(position.size(), 0.0);
    }
    for (auto &value : velocity) {
      value *= rate_scale;
    }

    result.positions.push_back(std::move(position));
    result.velocities.push_back(std::move(velocity));
  }

  populate_acceleration_trajectory(result.accelerations, result.velocities, result.sample_dt);
  result.total_time =
      result.positions.size() > 1 ? result.sample_dt * static_cast<double>(result.positions.size() - 1) : 0.0;
  return result;
}

}  // namespace rokae_xmate3_ros2::runtime
