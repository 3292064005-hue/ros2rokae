#include "runtime/motion_runtime_internal.hpp"

#include <algorithm>
#include <cmath>

namespace rokae_xmate3_ros2::runtime {
double clamp_active_speed_scale(double scale) {
  return std::clamp(scale, 0.05, 2.0);
}

bool statusEquivalent(const RuntimeStatus &lhs, const RuntimeStatus &rhs) {
  return lhs.request_id == rhs.request_id && lhs.state == rhs.state && lhs.message == rhs.message &&
         lhs.total_segments == rhs.total_segments && lhs.completed_segments == rhs.completed_segments &&
         lhs.current_segment_index == rhs.current_segment_index &&
         lhs.terminal_success == rhs.terminal_success &&
         lhs.execution_backend == rhs.execution_backend &&
         lhs.control_owner == rhs.control_owner &&
         lhs.runtime_phase == rhs.runtime_phase;
}

std::string summarize_plan_notes(const MotionPlan &plan) {
  if (plan.notes.empty()) {
    return "plan ready";
  }
  std::string message = "plan ready";
  for (const auto &note : plan.notes) {
    if (note.empty()) {
      continue;
    }
    message += "; " + note;
  }
  return message;
}

std::vector<double> snapshot_to_vector(const std::array<double, 6> &values) {
  return std::vector<double>(values.begin(), values.end());
}

std::vector<double> fallback_vector(const std::vector<double> &candidate,
                                    const std::array<double, 6> &fallback) {
  if (!candidate.empty()) {
    return candidate;
  }
  return snapshot_to_vector(fallback);
}

std::vector<double> zero_vector_like(const std::vector<double> &reference) {
  return std::vector<double>(reference.size(), 0.0);
}

TrajectoryExecutionPoint build_start_override(const TrajectoryExecutionState &trajectory_state,
                                              const RobotSnapshot &snapshot) {
  TrajectoryExecutionPoint point;
  point.position = fallback_vector(trajectory_state.actual_position, snapshot.joint_position);
  point.velocity = fallback_vector(trajectory_state.actual_velocity, snapshot.joint_velocity);
  if (!trajectory_state.actual_acceleration.empty()) {
    point.acceleration = trajectory_state.actual_acceleration;
  } else {
    point.acceleration = zero_vector_like(point.position);
  }
  point.time_from_start = 0.0;
  return point;
}

double max_joint_error(const std::array<double, 6> &actual, const std::vector<double> &target) {
  double error = 0.0;
  for (std::size_t i = 0; i < actual.size() && i < target.size(); ++i) {
    error = std::max(error, std::fabs(actual[i] - target[i]));
  }
  return error;
}

std::vector<double> estimate_velocity_at_index(const PlannedSegment &segment, std::size_t point_index) {
  if (point_index < segment.joint_velocity_trajectory.size()) {
    return segment.joint_velocity_trajectory[point_index];
  }
  if (segment.joint_trajectory.empty() || segment.trajectory_dt <= 1e-9) {
    return {};
  }

  const auto width = segment.joint_trajectory.front().size();
  std::vector<double> velocity(width, 0.0);
  if (segment.joint_trajectory.size() == 1) {
    return velocity;
  }

  const auto &trajectory = segment.joint_trajectory;
  for (std::size_t axis = 0; axis < width; ++axis) {
    if (point_index == 0) {
      velocity[axis] = (trajectory[1][axis] - trajectory[0][axis]) / segment.trajectory_dt;
    } else if (point_index + 1 >= trajectory.size()) {
      velocity[axis] =
          (trajectory[point_index][axis] - trajectory[point_index - 1][axis]) / segment.trajectory_dt;
    } else {
      velocity[axis] =
          (trajectory[point_index + 1][axis] - trajectory[point_index - 1][axis]) /
          (2.0 * segment.trajectory_dt);
    }
  }
  return velocity;
}

std::vector<double> estimate_acceleration_at_index(const PlannedSegment &segment, std::size_t point_index) {
  if (point_index < segment.joint_acceleration_trajectory.size()) {
    return segment.joint_acceleration_trajectory[point_index];
  }
  if (segment.joint_trajectory.empty() || segment.trajectory_dt <= 1e-9) {
    return {};
  }

  const auto width = segment.joint_trajectory.front().size();
  std::vector<double> acceleration(width, 0.0);
  if (segment.joint_trajectory.size() < 3) {
    return acceleration;
  }

  const auto &trajectory = segment.joint_trajectory;
  for (std::size_t axis = 0; axis < width; ++axis) {
    if (point_index == 0) {
      acceleration[axis] =
          (trajectory[2][axis] - 2.0 * trajectory[1][axis] + trajectory[0][axis]) /
          (segment.trajectory_dt * segment.trajectory_dt);
    } else if (point_index + 1 >= trajectory.size()) {
      const auto last = trajectory.size() - 1;
      acceleration[axis] =
          (trajectory[last][axis] - 2.0 * trajectory[last - 1][axis] + trajectory[last - 2][axis]) /
          (segment.trajectory_dt * segment.trajectory_dt);
    } else {
      acceleration[axis] =
          (trajectory[point_index + 1][axis] - 2.0 * trajectory[point_index][axis] +
           trajectory[point_index - 1][axis]) /
          (segment.trajectory_dt * segment.trajectory_dt);
    }
  }
  return acceleration;
}

std::vector<FlattenedPlanPoint> flatten_plan(const MotionPlan &plan) {
  std::vector<FlattenedPlanPoint> flattened;
  double cumulative_time = 0.0;
  for (const auto &segment : plan.segments) {
    if (segment.joint_trajectory.empty()) {
      cumulative_time += segment.trajectory_total_time;
      continue;
    }
    for (std::size_t point_index = 0; point_index < segment.joint_trajectory.size(); ++point_index) {
      if (!flattened.empty() && point_index == 0) {
        continue;
      }
      FlattenedPlanPoint point;
      point.position = segment.joint_trajectory[point_index];
      point.velocity = estimate_velocity_at_index(segment, point_index);
      point.acceleration = estimate_acceleration_at_index(segment, point_index);
      point.absolute_time = cumulative_time + static_cast<double>(point_index) * segment.trajectory_dt;
      flattened.push_back(std::move(point));
    }
    cumulative_time += segment.trajectory_total_time;
  }
  return flattened;
}

TrajectoryExecutionGoal build_execution_goal(const MotionPlan &plan,
                                            double speed_scale,
                                            const TrajectoryExecutionPoint *start_override,
                                            double original_time_offset) {
  TrajectoryExecutionGoal goal;
  goal.request_id = plan.request_id;
  goal.total_segments = plan.segments.size();
  goal.original_time_offset = std::max(original_time_offset, 0.0);
  goal.original_time_scale = clamp_active_speed_scale(speed_scale);

  const auto flattened = flatten_plan(plan);
  if (flattened.empty()) {
    return goal;
  }

  std::vector<double> absolute_segment_end_times;
  absolute_segment_end_times.reserve(plan.segments.size());
  double cumulative_time = 0.0;
  for (const auto &segment : plan.segments) {
    cumulative_time += segment.trajectory_total_time;
    absolute_segment_end_times.push_back(cumulative_time);
  }

  while (goal.segment_index_offset < absolute_segment_end_times.size() &&
         absolute_segment_end_times[goal.segment_index_offset] <= goal.original_time_offset + kExecutionGoalRetimingEps) {
    ++goal.segment_index_offset;
  }

  if (start_override != nullptr) {
    auto override_point = *start_override;
    override_point.time_from_start = 0.0;
    goal.points.push_back(std::move(override_point));
  }

  for (const auto &point : flattened) {
    if (point.absolute_time + kExecutionGoalRetimingEps < goal.original_time_offset) {
      continue;
    }
    double relative_time = std::max(point.absolute_time - goal.original_time_offset, 0.0) / goal.original_time_scale;
    if (!goal.points.empty() &&
        relative_time <= goal.points.back().time_from_start + kExecutionGoalRetimingEps) {
      relative_time = goal.points.back().time_from_start + kExecutionGoalMinDt;
    }
    TrajectoryExecutionPoint trajectory_point;
    trajectory_point.position = point.position;
    trajectory_point.velocity = point.velocity;
    trajectory_point.acceleration = point.acceleration;
    for (double &value : trajectory_point.velocity) {
      value *= goal.original_time_scale;
    }
    for (double &value : trajectory_point.acceleration) {
      value *= goal.original_time_scale * goal.original_time_scale;
    }
    trajectory_point.time_from_start = relative_time;
    goal.points.push_back(std::move(trajectory_point));
  }

  if (goal.points.empty()) {
    TrajectoryExecutionPoint final_point;
    final_point.position = flattened.back().position;
    final_point.velocity.assign(final_point.position.size(), 0.0);
    final_point.acceleration.assign(final_point.position.size(), 0.0);
    final_point.time_from_start = 0.0;
    goal.points.push_back(std::move(final_point));
  }
  if (goal.points.size() == 1) {
    auto hold_point = goal.points.back();
    hold_point.time_from_start = std::max(kExecutionGoalMinDt, hold_point.time_from_start + kExecutionGoalMinDt);
    goal.points.push_back(std::move(hold_point));
  }

  for (std::size_t index = goal.segment_index_offset; index < absolute_segment_end_times.size(); ++index) {
    double relative_time =
        std::max(absolute_segment_end_times[index] - goal.original_time_offset, 0.0) / goal.original_time_scale;
    if (!goal.segment_end_times.empty() &&
        relative_time <= goal.segment_end_times.back() + kExecutionGoalRetimingEps) {
      relative_time = goal.segment_end_times.back() + kExecutionGoalMinDt;
    }
    goal.segment_end_times.push_back(relative_time);
  }

  const double final_required_time = goal.segment_end_times.empty() ? goal.points.back().time_from_start
                                                                    : goal.segment_end_times.back();
  if (goal.points.back().time_from_start + kExecutionGoalRetimingEps < final_required_time) {
    auto final_point = goal.points.back();
    final_point.time_from_start = final_required_time;
    goal.points.push_back(std::move(final_point));
  }

  return goal;
}

TrajectoryProgress compute_trajectory_progress(const TrajectoryExecutionGoal &goal,
                                               double desired_time_from_start) {
  TrajectoryProgress progress;
  progress.completed_segments = goal.segment_index_offset;
  progress.current_segment_index = goal.segment_index_offset;
  for (double segment_end_time : goal.segment_end_times) {
    if (desired_time_from_start + kExecutionGoalRetimingEps >= segment_end_time) {
      ++progress.completed_segments;
      ++progress.current_segment_index;
      continue;
    }
    break;
  }
  if (goal.total_segments > 0) {
    progress.current_segment_index =
        std::min(progress.current_segment_index, goal.total_segments - 1);
    progress.completed_segments =
        std::min(progress.completed_segments, goal.total_segments);
  }
  return progress;
}

}  // namespace rokae_xmate3_ros2::runtime
