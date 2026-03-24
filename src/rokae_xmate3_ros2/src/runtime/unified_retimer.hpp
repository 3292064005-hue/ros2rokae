#ifndef ROKAE_XMATE3_ROS2_RUNTIME_UNIFIED_RETIMER_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_UNIFIED_RETIMER_HPP

#include <array>
#include <string>
#include <vector>

#include "runtime/joint_retimer.hpp"
#include "runtime/runtime_state.hpp"
#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "rokae_xmate3_ros2/srv/generate_s_trajectory.hpp"

namespace rokae_xmate3_ros2::runtime {

struct UnifiedRetimerLimits {
  std::array<double, 6> velocity_limits{};
  std::array<double, 6> acceleration_limits{};
};

struct RetimerMetadata {
  std::string source_family{"joint"};
  double total_duration{0.0};
  double sample_dt{0.0};
  double effective_speed_scale{1.0};
  bool clamped{false};
  std::string note{"nominal"};
};

struct UnifiedTrajectoryResult {
  QuinticRetimerResult trajectory;
  RetimerMetadata metadata;

  [[nodiscard]] bool empty() const noexcept { return trajectory.empty(); }
};

[[nodiscard]] JointRetimerConfig makeUnifiedRetimerConfig(double sample_dt);

[[nodiscard]] UnifiedRetimerLimits makeUnifiedRetimerLimits(double max_velocity,
                                                            double max_acceleration,
                                                            double blend_radius);

[[nodiscard]] UnifiedTrajectoryResult retimeJointWithUnifiedConfig(
    const std::vector<double> &start,
    const std::vector<double> &target,
    double sample_dt,
    double max_velocity,
    double max_acceleration,
    double blend_radius,
    const std::string &source_family = "joint");

[[nodiscard]] UnifiedTrajectoryResult retimeJointWithUnifiedLimits(
    const std::vector<double> &start,
    const std::vector<double> &target,
    double sample_dt,
    const std::array<double, 6> &velocity_limits,
    const std::array<double, 6> &acceleration_limits,
    const std::string &source_family = "joint");

[[nodiscard]] std::array<double, 6> scaledUnifiedVelocityLimits(double speed_mm_per_s);

[[nodiscard]] std::array<double, 6> scaledUnifiedAccelerationLimits(double speed_mm_per_s);

[[nodiscard]] UnifiedTrajectoryResult retimeJointPathWithUnifiedLimits(
    const std::vector<std::vector<double>> &waypoints,
    double sample_dt,
    const std::array<double, 6> &velocity_limits,
    const std::array<double, 6> &acceleration_limits,
    const std::string &source_family = "joint",
    double effective_speed_scale = 1.0);

[[nodiscard]] UnifiedTrajectoryResult retimeJointPathWithUnifiedConfig(
    const std::vector<std::vector<double>> &waypoints,
    double sample_dt,
    double max_velocity,
    double max_acceleration,
    double blend_radius,
    const std::string &source_family = "joint",
    double effective_speed_scale = 1.0);

[[nodiscard]] UnifiedTrajectoryResult retimeJointPathWithUnifiedSpeed(
    const std::vector<std::vector<double>> &waypoints,
    double sample_dt,
    double speed_mm_per_s,
    const std::string &source_family = "joint",
    double effective_speed_scale = 1.0);

[[nodiscard]] std::vector<std::vector<double>> resampleCartesianPosePath(
    const std::vector<std::vector<double>> &poses,
    std::size_t sample_count);

[[nodiscard]] UnifiedTrajectoryResult buildApproximateCartesianSTrajectory(
    ::gazebo::xMate3Kinematics &kinematics,
    const rokae_xmate3_ros2::srv::GenerateSTrajectory::Request &request,
    double sample_dt);

[[nodiscard]] UnifiedTrajectoryResult retimeReplayWithUnifiedConfig(
    const ReplayPathAsset &asset,
    double rate,
    double sample_dt);

}  // namespace rokae_xmate3_ros2::runtime

#endif
