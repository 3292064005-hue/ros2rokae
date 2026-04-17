#ifndef ROKAE_XMATE3_ROS2_RUNTIME_UNIFIED_RETIMER_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_UNIFIED_RETIMER_HPP

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
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

enum class RetimerSourceFamily : std::uint8_t {
  joint,
  cartesian,
  replay,
  s_trajectory,
};

enum class RetimerNote : std::uint8_t {
  nominal,
  limits_clamped,
  replay_retimed,
  speed_scale_applied,
  cartesian_fallback_to_joint,
  degenerate_path,
};

enum class RetimerPolicy : std::uint8_t {
  nominal,
  conservative,
  online_safe,
};

struct CanonicalTrajectorySamples {
  std::vector<std::vector<double>> positions;
  std::vector<std::vector<double>> velocities;
  std::vector<std::vector<double>> accelerations;
  double sample_dt{0.0};
  double total_time{0.0};
  std::string error_message;

  [[nodiscard]] bool empty() const noexcept { return positions.empty(); }
};

struct RetimerMetadata {
  RetimerSourceFamily source_family{RetimerSourceFamily::joint};
  RetimerPolicy policy{RetimerPolicy::nominal};
  double total_duration{0.0};
  double sample_dt{0.0};
  double effective_speed_scale{1.0};
  bool clamped{false};
  bool velocity_clamped{false};
  bool acceleration_clamped{false};
  bool jerk_constrained{false};
  bool blend_trim_applied{false};
  bool cartesian_fallback_used{false};
  std::string detail;
  RetimerNote note{RetimerNote::nominal};
};


struct RetimerValidationReport {
  bool ok{false};
  std::size_t waypoint_count{0};
  bool sample_dt_clamped{false};
  bool velocity_limit_clamped{false};
  bool acceleration_limit_clamped{false};
  bool degenerate_path{false};
  std::string error_message;
};

struct UnifiedTrajectoryResult {
  CanonicalTrajectorySamples samples;
  RetimerMetadata metadata;

  [[nodiscard]] bool empty() const noexcept { return samples.empty(); }
};

[[nodiscard]] const char *to_string(RetimerSourceFamily source_family) noexcept;
[[nodiscard]] const char *to_string(RetimerNote note) noexcept;
[[nodiscard]] const char *to_string(RetimerPolicy policy) noexcept;
[[nodiscard]] std::string describeRetimerMetadata(const RetimerMetadata &metadata,
                                                  std::string_view context = {},
                                                  std::string_view detail = {});

[[nodiscard]] JointRetimerConfig makeUnifiedRetimerConfig(double sample_dt, RetimerPolicy policy = RetimerPolicy::nominal);

[[nodiscard]] UnifiedRetimerLimits makeUnifiedRetimerLimits(double max_velocity,
                                                            double max_acceleration,
                                                            double blend_radius,
                                                            RetimerPolicy policy = RetimerPolicy::nominal);

[[nodiscard]] RetimerValidationReport validateUnifiedRetimerInput(
    const std::vector<std::vector<double>> &waypoints,
    double sample_dt,
    const std::array<double, 6> &velocity_limits,
    const std::array<double, 6> &acceleration_limits,
    RetimerPolicy policy = RetimerPolicy::nominal);

[[nodiscard]] UnifiedTrajectoryResult retimeJointWithUnifiedConfig(
    const std::vector<double> &start,
    const std::vector<double> &target,
    double sample_dt,
    double max_velocity,
    double max_acceleration,
    double blend_radius,
    RetimerSourceFamily source_family = RetimerSourceFamily::joint,
    RetimerPolicy policy = RetimerPolicy::nominal);

[[nodiscard]] UnifiedTrajectoryResult retimeJointWithUnifiedLimits(
    const std::vector<double> &start,
    const std::vector<double> &target,
    double sample_dt,
    const std::array<double, 6> &velocity_limits,
    const std::array<double, 6> &acceleration_limits,
    RetimerSourceFamily source_family = RetimerSourceFamily::joint,
    RetimerPolicy policy = RetimerPolicy::nominal);

[[nodiscard]] std::array<double, 6> scaledUnifiedVelocityLimits(double speed_mm_per_s);

[[nodiscard]] std::array<double, 6> scaledUnifiedAccelerationLimits(double speed_mm_per_s);

[[nodiscard]] UnifiedTrajectoryResult retimeJointPathWithUnifiedLimits(
    const std::vector<std::vector<double>> &waypoints,
    double sample_dt,
    const std::array<double, 6> &velocity_limits,
    const std::array<double, 6> &acceleration_limits,
    RetimerSourceFamily source_family = RetimerSourceFamily::joint,
    double effective_speed_scale = 1.0,
    RetimerPolicy policy = RetimerPolicy::nominal);

[[nodiscard]] UnifiedTrajectoryResult retimeJointPathWithUnifiedConfig(
    const std::vector<std::vector<double>> &waypoints,
    double sample_dt,
    double max_velocity,
    double max_acceleration,
    double blend_radius,
    RetimerSourceFamily source_family = RetimerSourceFamily::joint,
    double effective_speed_scale = 1.0,
    RetimerPolicy policy = RetimerPolicy::nominal);

[[nodiscard]] UnifiedTrajectoryResult retimeJointPathWithUnifiedSpeed(
    const std::vector<std::vector<double>> &waypoints,
    double sample_dt,
    double speed_mm_per_s,
    RetimerSourceFamily source_family = RetimerSourceFamily::joint,
    double effective_speed_scale = 1.0,
    RetimerPolicy policy = RetimerPolicy::nominal);

[[nodiscard]] std::vector<std::vector<double>> resampleCartesianPosePath(
    const std::vector<std::vector<double>> &poses,
    std::size_t sample_count);

[[nodiscard]] UnifiedTrajectoryResult buildApproximateCartesianSTrajectory(
    ::gazebo::xMate3Kinematics &kinematics,
    const rokae_xmate3_ros2::srv::GenerateSTrajectory::Request &request,
    double sample_dt,
    RetimerPolicy policy = RetimerPolicy::nominal);

[[nodiscard]] UnifiedTrajectoryResult retimeReplayWithUnifiedConfig(
    const ReplayPathAsset &asset,
    double rate,
    double sample_dt,
    RetimerPolicy policy = RetimerPolicy::nominal);

}  // namespace rokae_xmate3_ros2::runtime

#endif
