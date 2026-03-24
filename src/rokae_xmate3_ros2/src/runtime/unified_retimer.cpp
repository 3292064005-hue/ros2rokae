#include "runtime/unified_retimer.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

#include "runtime/planning_utils.hpp"
#include "runtime/pose_utils.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

struct QuinticBlendSample {
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
};

[[nodiscard]] QuinticBlendSample sample_quintic_blend(double u, double total_scale, double total_time) {
  const double clamped_u = std::clamp(u, 0.0, 1.0);
  const double u2 = clamped_u * clamped_u;
  const double u3 = u2 * clamped_u;
  const double u4 = u3 * clamped_u;
  const double u5 = u4 * clamped_u;
  const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
  const double blend_dot = 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
  const double blend_ddot = 60.0 * clamped_u - 180.0 * u2 + 120.0 * u3;

  QuinticBlendSample sample;
  sample.position = total_scale * blend;
  if (total_time > 1e-9) {
    sample.velocity = total_scale * blend_dot / total_time;
    sample.acceleration = total_scale * blend_ddot / (total_time * total_time);
  }
  return sample;
}

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

[[nodiscard]] RetimerMetadata make_metadata(std::string source_family,
                                            double effective_speed_scale = 1.0,
                                            bool clamped = false,
                                            std::string note = "nominal") {
  RetimerMetadata metadata;
  metadata.source_family = std::move(source_family);
  metadata.effective_speed_scale = effective_speed_scale;
  metadata.clamped = clamped;
  metadata.note = std::move(note);
  return metadata;
}

[[nodiscard]] UnifiedTrajectoryResult wrap_trajectory(QuinticRetimerResult trajectory,
                                                      std::string source_family,
                                                      double effective_speed_scale = 1.0,
                                                      bool clamped = false,
                                                      std::string note = "nominal") {
  UnifiedTrajectoryResult result;
  result.metadata = make_metadata(std::move(source_family), effective_speed_scale, clamped, std::move(note));
  result.metadata.sample_dt = trajectory.sample_dt;
  result.metadata.total_duration = trajectory.total_time;
  result.trajectory = std::move(trajectory);
  return result;
}

void sync_metadata(UnifiedTrajectoryResult &result) {
  result.metadata.sample_dt = result.trajectory.sample_dt;
  result.metadata.total_duration = result.trajectory.total_time;
}

[[nodiscard]] double sanitize_replay_time(double requested,
                                          double fallback_time,
                                          double min_step) {
  if (!std::isfinite(requested)) {
    return fallback_time;
  }
  return std::max(requested, fallback_time - min_step);
}

[[nodiscard]] double joint_segment_length(const std::vector<double> &lhs,
                                          const std::vector<double> &rhs) {
  double sum_sq = 0.0;
  for (std::size_t index = 0; index < lhs.size() && index < rhs.size(); ++index) {
    const double delta = rhs[index] - lhs[index];
    sum_sq += delta * delta;
  }
  return std::sqrt(sum_sq);
}

[[nodiscard]] std::vector<std::vector<double>> normalize_joint_waypoints(
    const std::vector<std::vector<double>> &waypoints) {
  std::vector<std::vector<double>> normalized;
  normalized.reserve(waypoints.size());
  for (const auto &waypoint : waypoints) {
    if (waypoint.empty()) {
      continue;
    }
    if (!normalized.empty() && joint_segment_length(normalized.back(), waypoint) < 1e-9) {
      continue;
    }
    normalized.push_back(waypoint);
  }
  return normalized;
}

[[nodiscard]] std::array<double, 6> joint_path_axis_travel(
    const std::vector<std::vector<double>> &waypoints) {
  std::array<double, 6> travel{};
  for (std::size_t point_index = 1; point_index < waypoints.size(); ++point_index) {
    const auto &lhs = waypoints[point_index - 1];
    const auto &rhs = waypoints[point_index];
    for (std::size_t axis = 0; axis < 6 && axis < lhs.size() && axis < rhs.size(); ++axis) {
      travel[axis] += std::fabs(rhs[axis] - lhs[axis]);
    }
  }
  return travel;
}

[[nodiscard]] std::vector<double> cumulative_joint_path_lengths(
    const std::vector<std::vector<double>> &waypoints) {
  std::vector<double> cumulative(waypoints.size(), 0.0);
  for (std::size_t point_index = 1; point_index < waypoints.size(); ++point_index) {
    cumulative[point_index] =
        cumulative[point_index - 1] + joint_segment_length(waypoints[point_index - 1], waypoints[point_index]);
  }
  return cumulative;
}

[[nodiscard]] int determine_path_interval_count(double total_time,
                                                double requested_dt,
                                                double total_axis_travel,
                                                double max_joint_step_rad,
                                                double min_sample_dt,
                                                double max_sample_dt) {
  if (total_time <= 1e-9) {
    return 1;
  }

  const double clamped_max_dt = std::clamp(requested_dt, min_sample_dt, max_sample_dt);
  const int time_intervals = std::max(1, static_cast<int>(std::ceil(total_time / clamped_max_dt)));
  const int path_intervals = max_joint_step_rad > 1e-9
                                 ? std::max(1, static_cast<int>(std::ceil(total_axis_travel / max_joint_step_rad)))
                                 : 1;
  const int min_dt_intervals = std::max(1, static_cast<int>(std::ceil(total_time / min_sample_dt)));
  return std::clamp(std::max(time_intervals, path_intervals), 1, min_dt_intervals);
}

struct JointPathInterpolation {
  std::vector<double> position;
  std::vector<double> tangent;
};

[[nodiscard]] JointPathInterpolation interpolate_joint_path(
    const std::vector<std::vector<double>> &waypoints,
    const std::vector<double> &cumulative_lengths,
    double path_length) {
  JointPathInterpolation sample;
  if (waypoints.empty()) {
    return sample;
  }
  if (waypoints.size() == 1 || cumulative_lengths.empty()) {
    sample.position = waypoints.front();
    sample.tangent.assign(sample.position.size(), 0.0);
    return sample;
  }

  const double clamped_path_length =
      std::clamp(path_length, 0.0, cumulative_lengths.back());
  auto upper = std::lower_bound(cumulative_lengths.begin(), cumulative_lengths.end(), clamped_path_length);
  std::size_t segment_index = 0;
  if (upper == cumulative_lengths.begin()) {
    segment_index = 0;
  } else if (upper == cumulative_lengths.end()) {
    segment_index = cumulative_lengths.size() - 2;
  } else {
    segment_index = static_cast<std::size_t>(std::distance(cumulative_lengths.begin(), upper) - 1);
  }

  while (segment_index + 1 < waypoints.size()) {
    const double segment_length = cumulative_lengths[segment_index + 1] - cumulative_lengths[segment_index];
    if (segment_length > 1e-9) {
      const double alpha =
          std::clamp((clamped_path_length - cumulative_lengths[segment_index]) / segment_length, 0.0, 1.0);
      sample.position.assign(waypoints[segment_index].size(), 0.0);
      sample.tangent.assign(waypoints[segment_index].size(), 0.0);
      for (std::size_t axis = 0; axis < waypoints[segment_index].size() &&
                                 axis < waypoints[segment_index + 1].size();
           ++axis) {
        const double lhs = waypoints[segment_index][axis];
        const double rhs = waypoints[segment_index + 1][axis];
        sample.position[axis] = lhs + alpha * (rhs - lhs);
        sample.tangent[axis] = (rhs - lhs) / segment_length;
      }
      return sample;
    }
    ++segment_index;
  }

  sample.position = waypoints.back();
  sample.tangent.assign(sample.position.size(), 0.0);
  return sample;
}

[[nodiscard]] double cartesian_segment_length(const std::vector<double> &lhs,
                                              const std::vector<double> &rhs) {
  const Eigen::Vector3d lhs_position(lhs[0], lhs[1], lhs[2]);
  const Eigen::Vector3d rhs_position(rhs[0], rhs[1], rhs[2]);
  const double position_length = (rhs_position - lhs_position).norm();
  const double orientation_length = pose_utils::angularDistance(lhs, rhs);
  return position_length + 0.1 * orientation_length;
}

[[nodiscard]] std::vector<double> cumulative_cartesian_path_lengths(
    const std::vector<std::vector<double>> &poses) {
  std::vector<double> cumulative(poses.size(), 0.0);
  for (std::size_t point_index = 1; point_index < poses.size(); ++point_index) {
    cumulative[point_index] =
        cumulative[point_index - 1] + cartesian_segment_length(poses[point_index - 1], poses[point_index]);
  }
  return cumulative;
}

[[nodiscard]] std::vector<double> interpolate_cartesian_pose(
    const std::vector<std::vector<double>> &poses,
    const std::vector<double> &cumulative_lengths,
    double path_length) {
  if (poses.empty()) {
    return {};
  }
  if (poses.size() == 1 || cumulative_lengths.empty()) {
    return poses.front();
  }

  const double clamped_path_length =
      std::clamp(path_length, 0.0, cumulative_lengths.back());
  auto upper = std::lower_bound(cumulative_lengths.begin(), cumulative_lengths.end(), clamped_path_length);
  std::size_t segment_index = 0;
  if (upper == cumulative_lengths.begin()) {
    segment_index = 0;
  } else if (upper == cumulative_lengths.end()) {
    segment_index = cumulative_lengths.size() - 2;
  } else {
    segment_index = static_cast<std::size_t>(std::distance(cumulative_lengths.begin(), upper) - 1);
  }

  while (segment_index + 1 < poses.size()) {
    const double segment_length = cumulative_lengths[segment_index + 1] - cumulative_lengths[segment_index];
    if (segment_length > 1e-9) {
      const double alpha =
          std::clamp((clamped_path_length - cumulative_lengths[segment_index]) / segment_length, 0.0, 1.0);
      const auto lhs = pose_utils::poseToIsometry(poses[segment_index]);
      const auto rhs = pose_utils::poseToIsometry(poses[segment_index + 1]);
      Eigen::Isometry3d interpolated = Eigen::Isometry3d::Identity();
      interpolated.translation() =
          lhs.translation() + alpha * (rhs.translation() - lhs.translation());
      const Eigen::Quaterniond lhs_q(lhs.linear());
      const Eigen::Quaterniond rhs_q(rhs.linear());
      interpolated.linear() = lhs_q.slerp(alpha, rhs_q).toRotationMatrix();
      return pose_utils::isometryToPose(interpolated);
    }
    ++segment_index;
  }

  return poses.back();
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

UnifiedTrajectoryResult retimeJointWithUnifiedConfig(const std::vector<double> &start,
                                                     const std::vector<double> &target,
                                                     double sample_dt,
                                                     double max_velocity,
                                                     double max_acceleration,
                                                     double blend_radius,
                                                     const std::string &source_family) {
  return retimeJointPathWithUnifiedConfig(
      {start, target}, sample_dt, max_velocity, max_acceleration, blend_radius, source_family);
}

UnifiedTrajectoryResult retimeJointWithUnifiedLimits(const std::vector<double> &start,
                                                     const std::vector<double> &target,
                                                     double sample_dt,
                                                     const std::array<double, 6> &velocity_limits,
                                                     const std::array<double, 6> &acceleration_limits,
                                                     const std::string &source_family) {
  return retimeJointPathWithUnifiedLimits(
      {start, target}, sample_dt, velocity_limits, acceleration_limits, source_family);
}

std::array<double, 6> scaledUnifiedVelocityLimits(double speed_mm_per_s) {
  const auto planner_config = ::gazebo::TrajectoryPlanner::config();
  auto limits = planner_config.joint_speed_limits_rad_per_sec;
  const double scale = std::max(
      std::clamp(speed_mm_per_s,
                 planner_config.min_nrt_speed_mm_per_s,
                 planner_config.max_nrt_speed_mm_per_s) /
          planner_config.max_nrt_speed_mm_per_s,
      0.05);
  for (double &value : limits) {
    value *= scale;
  }
  return limits;
}

std::array<double, 6> scaledUnifiedAccelerationLimits(double speed_mm_per_s) {
  const auto planner_config = ::gazebo::TrajectoryPlanner::config();
  auto limits = planner_config.joint_acc_limits_rad_per_sec2;
  const double scale = std::max(
      std::clamp(speed_mm_per_s,
                 planner_config.min_nrt_speed_mm_per_s,
                 planner_config.max_nrt_speed_mm_per_s) /
          planner_config.max_nrt_speed_mm_per_s,
      0.05);
  for (double &value : limits) {
    value *= scale * scale;
  }
  return limits;
}

UnifiedTrajectoryResult retimeJointPathWithUnifiedLimits(
    const std::vector<std::vector<double>> &waypoints,
    double sample_dt,
    const std::array<double, 6> &velocity_limits,
    const std::array<double, 6> &acceleration_limits,
    const std::string &source_family,
    double effective_speed_scale) {
  auto result = wrap_trajectory({}, source_family, effective_speed_scale);
  auto &trajectory = result.trajectory;
  const auto normalized_waypoints = normalize_joint_waypoints(waypoints);
  if (normalized_waypoints.empty()) {
    trajectory.error_message = "joint path is empty";
    result.metadata.note = "degenerate_path";
    return result;
  }

  if (normalized_waypoints.size() == 1) {
    trajectory.positions = normalized_waypoints;
    trajectory.velocities.assign(1, std::vector<double>(normalized_waypoints.front().size(), 0.0));
    trajectory.accelerations.assign(1, std::vector<double>(normalized_waypoints.front().size(), 0.0));
    trajectory.sample_dt = std::max(sample_dt, 1e-3);
    trajectory.total_time = 0.0;
    result.metadata.note = "degenerate_path";
    sync_metadata(result);
    return result;
  }

  const auto config = makeUnifiedRetimerConfig(sample_dt);
  const auto cumulative_lengths = cumulative_joint_path_lengths(normalized_waypoints);
  const double total_path_length = cumulative_lengths.empty() ? 0.0 : cumulative_lengths.back();
  if (total_path_length <= 1e-9) {
    trajectory.positions = {normalized_waypoints.front()};
    trajectory.velocities.assign(1, std::vector<double>(normalized_waypoints.front().size(), 0.0));
    trajectory.accelerations.assign(1, std::vector<double>(normalized_waypoints.front().size(), 0.0));
    trajectory.sample_dt = std::clamp(config.sample_dt, config.min_sample_dt, config.max_sample_dt);
    trajectory.total_time = 0.0;
    result.metadata.note = "degenerate_path";
    sync_metadata(result);
    return result;
  }

  std::vector<double> path_start(6, 0.0);
  const auto axis_travel = joint_path_axis_travel(normalized_waypoints);
  const std::vector<double> path_target(axis_travel.begin(), axis_travel.end());
  trajectory.total_time = computeJointRetimerDuration(path_start, path_target, velocity_limits, acceleration_limits);
  const double max_axis_travel =
      *std::max_element(axis_travel.begin(), axis_travel.end());
  const int interval_count = determine_path_interval_count(
      trajectory.total_time,
      config.sample_dt,
      max_axis_travel,
      config.max_joint_step_rad,
      config.min_sample_dt,
      config.max_sample_dt);
  trajectory.sample_dt = trajectory.total_time / static_cast<double>(interval_count);
  trajectory.positions.reserve(static_cast<std::size_t>(interval_count) + 1);
  trajectory.velocities.reserve(static_cast<std::size_t>(interval_count) + 1);
  trajectory.accelerations.reserve(static_cast<std::size_t>(interval_count) + 1);

  for (int index = 0; index <= interval_count; ++index) {
    const double u = static_cast<double>(index) / static_cast<double>(interval_count);
    const auto path_sample = sample_quintic_blend(u, total_path_length, trajectory.total_time);
    const auto interpolated =
        interpolate_joint_path(normalized_waypoints, cumulative_lengths, path_sample.position);
    if (interpolated.position.empty()) {
      continue;
    }

    std::vector<double> velocity(interpolated.position.size(), 0.0);
    std::vector<double> acceleration(interpolated.position.size(), 0.0);
    for (std::size_t axis = 0; axis < interpolated.position.size() && axis < interpolated.tangent.size(); ++axis) {
      velocity[axis] = interpolated.tangent[axis] * path_sample.velocity;
      acceleration[axis] = interpolated.tangent[axis] * path_sample.acceleration;
    }

    trajectory.positions.push_back(interpolated.position);
    trajectory.velocities.push_back(std::move(velocity));
    trajectory.accelerations.push_back(std::move(acceleration));
  }

  trajectory.total_time =
      trajectory.positions.size() > 1 ? trajectory.sample_dt * static_cast<double>(trajectory.positions.size() - 1) : 0.0;
  if (!trajectory.positions.empty()) {
    trajectory.positions.front() = normalized_waypoints.front();
    trajectory.positions.back() = normalized_waypoints.back();
  }
  sync_metadata(result);
  return result;
}

UnifiedTrajectoryResult retimeJointPathWithUnifiedConfig(const std::vector<std::vector<double>> &waypoints,
                                                         double sample_dt,
                                                         double max_velocity,
                                                         double max_acceleration,
                                                         double blend_radius,
                                                         const std::string &source_family,
                                                         double effective_speed_scale) {
  const auto limits = makeUnifiedRetimerLimits(max_velocity, max_acceleration, blend_radius);
  return retimeJointPathWithUnifiedLimits(
      waypoints, sample_dt, limits.velocity_limits, limits.acceleration_limits, source_family, effective_speed_scale);
}

UnifiedTrajectoryResult retimeJointPathWithUnifiedSpeed(const std::vector<std::vector<double>> &waypoints,
                                                        double sample_dt,
                                                        double speed_mm_per_s,
                                                        const std::string &source_family,
                                                        double effective_speed_scale) {
  return retimeJointPathWithUnifiedLimits(
      waypoints,
      sample_dt,
      scaledUnifiedVelocityLimits(speed_mm_per_s),
      scaledUnifiedAccelerationLimits(speed_mm_per_s),
      source_family,
      effective_speed_scale);
}

std::vector<std::vector<double>> resampleCartesianPosePath(const std::vector<std::vector<double>> &poses,
                                                           std::size_t sample_count) {
  if (poses.empty()) {
    return {};
  }
  if (sample_count <= 1 || poses.size() == 1) {
    return {poses.front()};
  }

  const auto cumulative_lengths = cumulative_cartesian_path_lengths(poses);
  const double total_path_length = cumulative_lengths.empty() ? 0.0 : cumulative_lengths.back();
  if (total_path_length <= 1e-9) {
    return std::vector<std::vector<double>>(sample_count, poses.front());
  }

  std::vector<std::vector<double>> out;
  out.reserve(sample_count);
  for (std::size_t index = 0; index < sample_count; ++index) {
    const double alpha = sample_count > 1 ? static_cast<double>(index) / static_cast<double>(sample_count - 1) : 0.0;
    out.push_back(interpolate_cartesian_pose(poses, cumulative_lengths, alpha * total_path_length));
  }
  out.front() = poses.front();
  out.back() = poses.back();
  return out;
}

UnifiedTrajectoryResult buildApproximateCartesianSTrajectory(
    ::gazebo::xMate3Kinematics &kinematics,
    const rokae_xmate3_ros2::srv::GenerateSTrajectory::Request &request,
    double sample_dt) {
  auto result = wrap_trajectory({}, "s_trajectory");
  auto &trajectory = result.trajectory;

  std::vector<double> start(request.start_joint_pos.begin(), request.start_joint_pos.end());
  std::vector<double> target(request.target_joint_pos.begin(), request.target_joint_pos.end());
  const auto start_pose = kinematics.forwardKinematicsRPY(start);
  const auto target_pose = kinematics.forwardKinematicsRPY(target);
  const double speed_mm_per_s = resolve_cartesian_speed_mm_per_s(request, start_pose, target_pose);
  const auto cartesian_samples =
      ::gazebo::TrajectoryPlanner::planCartesianLine(start_pose, target_pose, speed_mm_per_s, sample_dt);
  if (cartesian_samples.empty()) {
    trajectory.error_message = "cartesian path generation failed";
    result.metadata.note = "degenerate_path";
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
                                             trajectory.positions,
                                             last_joints,
                                             trajectory.error_message)) {
    return result;
  }

  if (trajectory.positions.empty()) {
    trajectory.error_message = "cartesian joint sampling produced no trajectory points";
    result.metadata.note = "degenerate_path";
    return result;
  }

  trajectory.positions.front() = start;
  trajectory.positions.back() = target;

  auto retimed = retimeJointPathWithUnifiedConfig(
      trajectory.positions,
      sample_dt,
      request.max_velocity,
      request.max_acceleration,
      request.blend_radius,
      "s_trajectory");
  if (retimed.empty()) {
    retimed.trajectory.error_message = retimed.trajectory.error_message.empty()
                                           ? "cartesian unified retimer failed"
                                           : retimed.trajectory.error_message;
    return retimed;
  }

  const auto resampled_cartesian =
      resampleCartesianPosePath(cartesian_samples.points, retimed.trajectory.positions.size());
  if (!project_joint_derivatives_from_cartesian(kinematics,
                                                resampled_cartesian,
                                                retimed.trajectory.positions,
                                                retimed.trajectory.sample_dt,
                                                retimed.trajectory.velocities,
                                                retimed.trajectory.accelerations)) {
    if (retimed.trajectory.velocities.size() != retimed.trajectory.positions.size()) {
      retimed.trajectory.velocities.assign(
          retimed.trajectory.positions.size(), std::vector<double>(retimed.trajectory.positions.front().size(), 0.0));
    }
    if (retimed.trajectory.accelerations.size() != retimed.trajectory.positions.size()) {
      retimed.trajectory.accelerations.assign(
          retimed.trajectory.positions.size(), std::vector<double>(retimed.trajectory.positions.front().size(), 0.0));
    }
  }
  retimed.metadata.source_family = "s_trajectory";
  sync_metadata(retimed);
  return retimed;
}

UnifiedTrajectoryResult retimeReplayWithUnifiedConfig(const ReplayPathAsset &asset,
                                                      double rate,
                                                      double sample_dt) {
  auto result = wrap_trajectory({}, "replay", std::max(rate, 0.05), false, "replay_retimed");
  auto &trajectory = result.trajectory;
  if (asset.samples.empty()) {
    trajectory.error_message = "Path is empty";
    result.metadata.note = "degenerate_path";
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

  trajectory.sample_dt =
      scaled_total_time > 1e-9 ? scaled_total_time / static_cast<double>(interval_count) : resolved_sample_dt;
  trajectory.positions.reserve(static_cast<std::size_t>(interval_count) + 1);
  trajectory.velocities.reserve(static_cast<std::size_t>(interval_count) + 1);

  for (int index = 0; index <= interval_count; ++index) {
    const double scaled_time =
        scaled_total_time > 1e-9 ? trajectory.sample_dt * static_cast<double>(index) : 0.0;
    const double original_time = std::min(scaled_time * rate_scale, original_total_time);

    auto position = interpolate_joint_vector(normalized_samples, original_time, false);
    auto velocity = interpolate_joint_vector(normalized_samples, original_time, true);
    if (velocity.size() < position.size()) {
      velocity.resize(position.size(), 0.0);
    }
    for (auto &value : velocity) {
      value *= rate_scale;
    }

    trajectory.positions.push_back(std::move(position));
    trajectory.velocities.push_back(std::move(velocity));
  }

  populate_acceleration_trajectory(trajectory.accelerations, trajectory.velocities, trajectory.sample_dt);
  trajectory.total_time =
      trajectory.positions.size() > 1 ? trajectory.sample_dt * static_cast<double>(trajectory.positions.size() - 1) : 0.0;
  sync_metadata(result);
  return result;
}

}  // namespace rokae_xmate3_ros2::runtime
