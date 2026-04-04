#include "runtime/joint_retimer.hpp"

#include <algorithm>
#include <cmath>

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr double kMinJointVelocityRadPerSec = 0.05;
constexpr double kMinJointAccelerationRadPerSec2 = 0.10;

int determine_interval_count(double total_time,
                             double requested_dt,
                             double max_joint_delta,
                             double max_joint_step_rad,
                             double min_sample_dt,
                             double max_sample_dt) {
  if (total_time <= 1e-9) {
    return 1;
  }

  const double clamped_max_dt = std::clamp(requested_dt, min_sample_dt, max_sample_dt);
  const int time_intervals = std::max(1, static_cast<int>(std::ceil(total_time / clamped_max_dt)));
  const int path_intervals = max_joint_step_rad > 1e-9
                                 ? std::max(1, static_cast<int>(std::ceil(max_joint_delta / max_joint_step_rad)))
                                 : 1;
  const int min_dt_intervals = std::max(1, static_cast<int>(std::ceil(total_time / min_sample_dt)));
  return std::clamp(std::max(time_intervals, path_intervals), 1, min_dt_intervals);
}

double resolve_sample_dt(const JointRetimerConfig &config) {
  return std::clamp(config.sample_dt, config.min_sample_dt, config.max_sample_dt);
}

}  // namespace

double computeJointRetimerDuration(const std::vector<double> &start,
                                   const std::vector<double> &target,
                                   const std::array<double, 6> &joint_speed_limits_rad_per_sec,
                                   const std::array<double, 6> &joint_acc_limits_rad_per_sec2) {
  double total_time = 0.0;
  for (std::size_t axis = 0; axis < start.size() && axis < target.size() &&
                             axis < joint_speed_limits_rad_per_sec.size();
       ++axis) {
    const double delta = std::fabs(target[axis] - start[axis]);
    if (delta < 1e-9) {
      continue;
    }
    const double velocity_limit =
        std::max(joint_speed_limits_rad_per_sec[axis], kMinJointVelocityRadPerSec);
    const double acceleration_limit =
        std::max(joint_acc_limits_rad_per_sec2[axis], kMinJointAccelerationRadPerSec2);
    StrictJerkLimitedScalarProfile profile;
    profile.configure(delta, velocity_limit, acceleration_limit, acceleration_limit);
    total_time = std::max(total_time, profile.total_time());
  }
  return total_time;
}

JointRetimerResult retimeJointStrictJerkLimited(
    const std::vector<double> &start,
    const std::vector<double> &target,
    const JointRetimerConfig &config,
    const std::array<double, 6> &joint_speed_limits_rad_per_sec,
    const std::array<double, 6> &joint_acc_limits_rad_per_sec2) {
  JointRetimerResult result;
  if (start.empty() || start.size() != target.size()) {
    return result;
  }

  double max_joint_delta = 0.0;
  for (std::size_t axis = 0; axis < start.size(); ++axis) {
    max_joint_delta = std::max(max_joint_delta, std::fabs(target[axis] - start[axis]));
  }

  if (max_joint_delta < 1e-9) {
    result.positions.push_back(start);
    result.velocities.push_back(std::vector<double>(start.size(), 0.0));
    result.accelerations.push_back(std::vector<double>(start.size(), 0.0));
    result.sample_dt = resolve_sample_dt(config);
    result.total_time = 0.0;
    return result;
  }

  const double requested_total_time = computeJointRetimerDuration(
      start, target, joint_speed_limits_rad_per_sec, joint_acc_limits_rad_per_sec2);
  const double resolved_sample_dt = resolve_sample_dt(config);
  const int interval_count = determine_interval_count(
      requested_total_time,
      config.sample_dt,
      max_joint_delta,
      config.max_joint_step_rad,
      config.min_sample_dt,
      config.max_sample_dt);
  result.sample_dt = resolved_sample_dt;
  result.total_time = static_cast<double>(interval_count) * result.sample_dt;

  result.positions.reserve(static_cast<std::size_t>(interval_count) + 1);
  result.velocities.reserve(static_cast<std::size_t>(interval_count) + 1);
  result.accelerations.reserve(static_cast<std::size_t>(interval_count) + 1);

  std::vector<StrictJerkLimitedScalarProfile> profiles(start.size());
  for (std::size_t axis = 0; axis < start.size(); ++axis) {
    const double velocity_limit = std::max(joint_speed_limits_rad_per_sec[axis], kMinJointVelocityRadPerSec);
    const double acceleration_limit = std::max(joint_acc_limits_rad_per_sec2[axis], kMinJointAccelerationRadPerSec2);
    profiles[axis].configure(target[axis] - start[axis], velocity_limit, acceleration_limit, acceleration_limit);
  }

  for (int index = 0; index <= interval_count; ++index) {
    const double t = static_cast<double>(index) * result.sample_dt;

    std::vector<double> position(start.size(), 0.0);
    std::vector<double> velocity(start.size(), 0.0);
    std::vector<double> acceleration(start.size(), 0.0);
    for (std::size_t axis = 0; axis < start.size(); ++axis) {
      const double axis_total_time = profiles[axis].total_time();
      const double sample_time = (axis_total_time <= 1e-12 || result.total_time <= 1e-12)
                                     ? axis_total_time
                                     : std::clamp(t / result.total_time, 0.0, 1.0) * axis_total_time;
      const auto sample = profiles[axis].sample(sample_time);
      position[axis] = start[axis] + sample.position;
      velocity[axis] = sample.velocity;
      acceleration[axis] = sample.acceleration;
    }
    result.positions.push_back(std::move(position));
    result.velocities.push_back(std::move(velocity));
    result.accelerations.push_back(std::move(acceleration));
  }

  return result;
}

::gazebo::TrajectorySamples toTrajectorySamples(const JointRetimerResult &result) {
  ::gazebo::TrajectorySamples samples;
  samples.points = result.positions;
  samples.velocities = result.velocities;
  samples.accelerations = result.accelerations;
  samples.sample_dt = result.sample_dt;
  samples.total_time = result.total_time;
  return samples;
}

}  // namespace rokae_xmate3_ros2::runtime
