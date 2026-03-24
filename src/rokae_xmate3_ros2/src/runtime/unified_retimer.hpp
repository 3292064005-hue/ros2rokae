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

struct ApproximateCartesianRetimerResult {
  std::vector<std::vector<double>> joint_trajectory;
  std::vector<std::vector<double>> joint_velocity_trajectory;
  std::vector<std::vector<double>> joint_acceleration_trajectory;
  double sample_dt = 0.0;
  double total_time = 0.0;
  std::string error_message;

  [[nodiscard]] bool empty() const noexcept { return joint_trajectory.empty(); }
};

[[nodiscard]] JointRetimerConfig makeUnifiedRetimerConfig(double sample_dt);

[[nodiscard]] UnifiedRetimerLimits makeUnifiedRetimerLimits(double max_velocity,
                                                            double max_acceleration,
                                                            double blend_radius);

[[nodiscard]] QuinticRetimerResult retimeJointWithUnifiedConfig(
    const std::vector<double> &start,
    const std::vector<double> &target,
    double sample_dt,
    double max_velocity,
    double max_acceleration,
    double blend_radius);

[[nodiscard]] QuinticRetimerResult retimeJointWithUnifiedLimits(
    const std::vector<double> &start,
    const std::vector<double> &target,
    double sample_dt,
    const std::array<double, 6> &velocity_limits,
    const std::array<double, 6> &acceleration_limits);

[[nodiscard]] ApproximateCartesianRetimerResult buildApproximateCartesianSTrajectory(
    ::gazebo::xMate3Kinematics &kinematics,
    const rokae_xmate3_ros2::srv::GenerateSTrajectory::Request &request,
    double sample_dt);

[[nodiscard]] QuinticRetimerResult retimeReplayWithUnifiedConfig(
    const ReplayPathAsset &asset,
    double rate,
    double sample_dt);

}  // namespace rokae_xmate3_ros2::runtime

#endif
