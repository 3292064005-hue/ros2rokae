#include "runtime/joint_retimer.hpp"

#include <algorithm>
#include <cmath>

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr double kMinJointVelocityRadPerSec = 0.05;
constexpr double kMinJointAccelerationRadPerSec2 = 0.10;
constexpr double kQuinticPeakVelocityCoeff = 1.875;
constexpr double kQuinticPeakAccelerationCoeff = 10.0 / std::sqrt(3.0);

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
    const double velocity_bound = kQuinticPeakVelocityCoeff * delta / velocity_limit;
    const double acceleration_bound =
        std::sqrt(kQuinticPeakAccelerationCoeff * delta / acceleration_limit);
    total_time = std::max(total_time, std::max(velocity_bound, acceleration_bound));
  }
  return total_time;
}

QuinticRetimerResult retimeJointQuintic(
    const std::vector<double> &start,
    const std::vector<double> &target,
    const JointRetimerConfig &config,
    const std::array<double, 6> &joint_speed_limits_rad_per_sec,
    const std::array<double, 6> &joint_acc_limits_rad_per_sec2) {
  QuinticRetimerResult result;
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
    result.sample_dt = std::clamp(config.sample_dt, config.min_sample_dt, config.max_sample_dt);
    result.total_time = 0.0;
    return result;
  }

  result.total_time = computeJointRetimerDuration(
      start, target, joint_speed_limits_rad_per_sec, joint_acc_limits_rad_per_sec2);
  const int interval_count = determine_interval_count(
      result.total_time,
      config.sample_dt,
      max_joint_delta,
      config.max_joint_step_rad,
      config.min_sample_dt,
      config.max_sample_dt);
  result.sample_dt = result.total_time / static_cast<double>(interval_count);

  result.positions.reserve(static_cast<std::size_t>(interval_count) + 1);
  result.velocities.reserve(static_cast<std::size_t>(interval_count) + 1);
  result.accelerations.reserve(static_cast<std::size_t>(interval_count) + 1);

  for (int index = 0; index <= interval_count; ++index) {
    const double u = static_cast<double>(index) / static_cast<double>(interval_count);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double blend = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
    const double blend_dot = 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
    const double blend_ddot = 60.0 * u - 180.0 * u2 + 120.0 * u3;

    std::vector<double> position(start.size(), 0.0);
    std::vector<double> velocity(start.size(), 0.0);
    std::vector<double> acceleration(start.size(), 0.0);
    for (std::size_t axis = 0; axis < start.size(); ++axis) {
      const double delta = target[axis] - start[axis];
      position[axis] = start[axis] + delta * blend;
      velocity[axis] = delta * blend_dot / result.total_time;
      acceleration[axis] = delta * blend_ddot / (result.total_time * result.total_time);
    }
    result.positions.push_back(std::move(position));
    result.velocities.push_back(std::move(velocity));
    result.accelerations.push_back(std::move(acceleration));
  }

  result.total_time =
      result.positions.size() > 1 ? (result.positions.size() - 1) * result.sample_dt : 0.0;
  return result;
}

::gazebo::TrajectorySamples toTrajectorySamples(const QuinticRetimerResult &result) {
  ::gazebo::TrajectorySamples samples;
  samples.points = result.positions;
  samples.velocities = result.velocities;
  samples.accelerations = result.accelerations;
  samples.sample_dt = result.sample_dt;
  samples.total_time = result.total_time;
  return samples;
}

}  // namespace rokae_xmate3_ros2::runtime
