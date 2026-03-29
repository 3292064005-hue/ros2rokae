#ifndef ROKAE_XMATE3_ROS2_RUNTIME_REQUEST_ADAPTER_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_REQUEST_ADAPTER_HPP

#include <array>
#include <string>
#include <vector>

#include "runtime/runtime_types.hpp"
#include "rokae_xmate3_ros2/action/move_append.hpp"
#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"

namespace rokae_xmate3_ros2::runtime {

struct ReplayPathAsset;

struct MotionRequestContext {
  std::string request_id;
  std::vector<double> start_joints;
  std::vector<double> tool_pose;
  std::vector<double> wobj_pose;
  double default_speed = 50.0;
  int default_zone = 0;
  bool strict_conf = false;
  bool avoid_singularity = true;
  bool soft_limit_enabled = false;
  double speed_scale = 1.0;
  std::array<std::array<double, 2>, 6> soft_limits = rokae_xmate3_ros2::spec::xmate3::kDefaultSoftLimits;
  double trajectory_dt = 0.01;
};

[[nodiscard]] bool build_motion_request(
    const rokae_xmate3_ros2::action::MoveAppend::Goal &goal,
    const MotionRequestContext &context,
    MotionRequest &request,
    std::string &error_message);

[[nodiscard]] bool build_replay_request(
    const ReplayPathAsset &replay_asset,
    double rate,
    const MotionRequestContext &context,
    MotionRequest &request,
    std::string &error_message);

}  // namespace rokae_xmate3_ros2::runtime

#endif
