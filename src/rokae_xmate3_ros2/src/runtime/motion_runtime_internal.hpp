#ifndef ROKAE_XMATE3_ROS2_RUNTIME_MOTION_RUNTIME_INTERNAL_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_MOTION_RUNTIME_INTERNAL_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include "runtime/motion_runtime.hpp"

namespace rokae_xmate3_ros2::runtime {

inline constexpr std::size_t kMaxCachedStatuses = 16;
inline constexpr double kExecutionGoalMinDt = 1e-3;
inline constexpr double kExecutionGoalRetimingEps = 1e-6;
inline constexpr double kTrajectoryCompletionJointToleranceRad = 0.03;

double clamp_active_speed_scale(double scale);
bool statusEquivalent(const RuntimeStatus &lhs, const RuntimeStatus &rhs);
std::string summarize_plan_notes(const MotionPlan &plan);

struct FlattenedPlanPoint {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
  double absolute_time = 0.0;
};

struct TrajectoryProgress {
  std::size_t completed_segments = 0;
  std::size_t current_segment_index = 0;
};

std::vector<double> snapshot_to_vector(const std::array<double, 6> &values);
std::vector<double> fallback_vector(const std::vector<double> &candidate,
                                    const std::array<double, 6> &fallback);
std::vector<double> zero_vector_like(const std::vector<double> &reference);
TrajectoryExecutionPoint build_start_override(const TrajectoryExecutionState &trajectory_state,
                                              const RobotSnapshot &snapshot);
double max_joint_error(const std::array<double, 6> &actual, const std::vector<double> &target);
std::vector<double> estimate_velocity_at_index(const PlannedSegment &segment, std::size_t point_index);
std::vector<double> estimate_acceleration_at_index(const PlannedSegment &segment, std::size_t point_index);
std::vector<FlattenedPlanPoint> flatten_plan(const MotionPlan &plan);
TrajectoryExecutionGoal build_execution_goal(const MotionPlan &plan,
                                            double speed_scale,
                                            const TrajectoryExecutionPoint *start_override,
                                            double original_time_offset);
TrajectoryProgress compute_trajectory_progress(const TrajectoryExecutionGoal &goal,
                                               double desired_time_from_start);

}  // namespace rokae_xmate3_ros2::runtime

#endif
