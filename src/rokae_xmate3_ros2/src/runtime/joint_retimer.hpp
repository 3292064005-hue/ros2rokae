#ifndef ROKAE_XMATE3_ROS2_RUNTIME_JOINT_RETIMER_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_JOINT_RETIMER_HPP

#include <array>
#include <vector>

#include "rokae_xmate3_ros2/gazebo/trajectory_planner.hpp"

namespace rokae_xmate3_ros2::runtime {

struct JointRetimerConfig {
  std::array<double, 6> joint_speed_limits_rad_per_sec{
      3.14159265358979323846,
      3.14159265358979323846,
      3.14159265358979323846,
      3.14159265358979323846,
      3.14159265358979323846,
      3.14159265358979323846,
  };
  std::array<double, 6> joint_acc_limits_rad_per_sec2{
      7.853981633974483,
      7.853981633974483,
      7.853981633974483,
      7.853981633974483,
      7.853981633974483,
      7.853981633974483,
  };
  double sample_dt = 0.01;
  double min_sample_dt = 0.001;
  double max_sample_dt = 0.05;
  double max_joint_step_rad = 0.025;
};

struct QuinticRetimerResult {
  std::vector<std::vector<double>> positions;
  std::vector<std::vector<double>> velocities;
  std::vector<std::vector<double>> accelerations;
  double sample_dt = 0.001;
  double total_time = 0.0;

  [[nodiscard]] bool empty() const noexcept { return positions.empty(); }
};

[[nodiscard]] double computeJointRetimerDuration(
    const std::vector<double> &start,
    const std::vector<double> &target,
    const std::array<double, 6> &joint_speed_limits_rad_per_sec,
    const std::array<double, 6> &joint_acc_limits_rad_per_sec2);

[[nodiscard]] QuinticRetimerResult retimeJointQuintic(
    const std::vector<double> &start,
    const std::vector<double> &target,
    const JointRetimerConfig &config,
    const std::array<double, 6> &joint_speed_limits_rad_per_sec,
    const std::array<double, 6> &joint_acc_limits_rad_per_sec2);

[[nodiscard]] ::gazebo::TrajectorySamples toTrajectorySamples(const QuinticRetimerResult &result);

}  // namespace rokae_xmate3_ros2::runtime

#endif
