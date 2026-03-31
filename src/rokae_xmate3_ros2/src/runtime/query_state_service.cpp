#include "runtime/service_facade.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <utility>

#include <Eigen/Dense>

#include "runtime/planner_core.hpp"
#include "runtime/planning_utils.hpp"
#include "runtime/pose_utils.hpp"
#include "runtime/service_facade_utils.hpp"
#include "runtime/unified_retimer.hpp"
#include "rokae_xmate3_ros2/gazebo/model_facade.hpp"

namespace rokae_xmate3_ros2::runtime {

void QueryFacade::handleGetPowerState(const rokae_xmate3_ros2::srv::GetPowerState::Request &req,
                                      rokae_xmate3_ros2::srv::GetPowerState::Response &res) const {
  (void)req;
  res.state.state = session_state_.powerOn() ? rokae_xmate3_ros2::msg::PowerState::ON
                                                : rokae_xmate3_ros2::msg::PowerState::OFF;
  res.success = true;
}

void QueryFacade::handleGetInfo(const rokae_xmate3_ros2::srv::GetInfo::Request &req,
                                rokae_xmate3_ros2::srv::GetInfo::Response &res) const {
  (void)req;
  res.model = "xMate3";
  res.robot_type = "3";
  res.serial_number = "SIMULATION";
  res.firmware_version = "2.1.0";
  res.sdk_version = "2.1.0";
  res.joint_num = 6;
  res.success = true;
}

void QueryFacade::handleGetOperateMode(const rokae_xmate3_ros2::srv::GetOperateMode::Request &req,
                                       rokae_xmate3_ros2::srv::GetOperateMode::Response &res) const {
  (void)req;
  res.mode = session_state_.operateMode();
  res.success = true;
}

void QueryFacade::handleQueryControllerLog(
    const rokae_xmate3_ros2::srv::QueryControllerLog::Request &req,
    rokae_xmate3_ros2::srv::QueryControllerLog::Response &res) const {
  res.logs = data_store_state_.queryLogs(req.count);
  res.success = true;
}

void QueryFacade::handleGetJointPos(const rokae_xmate3_ros2::srv::GetJointPos::Request &req,
                                    rokae_xmate3_ros2::srv::GetJointPos::Response &res) const {
  (void)req;
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  for (int i = 0; i < 6 && i < joint_num_; ++i) {
    res.joint_positions[i] = pos[i];
  }
  res.success = true;
}

void QueryFacade::handleGetJointVel(const rokae_xmate3_ros2::srv::GetJointVel::Request &req,
                                    rokae_xmate3_ros2::srv::GetJointVel::Response &res) const {
  (void)req;
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  for (int i = 0; i < 6 && i < joint_num_; ++i) {
    res.joint_vel[i] = vel[i];
  }
  res.success = true;
}

void QueryFacade::handleGetJointTorques(const rokae_xmate3_ros2::srv::GetJointTorques::Request &req,
                                       rokae_xmate3_ros2::srv::GetJointTorques::Response &res) const {
  (void)req;
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  for (int i = 0; i < 6 && i < joint_num_; ++i) {
    res.joint_torque[i] = tau[i];
  }
  res.success = true;
}

void QueryFacade::handleGetPosture(const rokae_xmate3_ros2::srv::GetPosture::Request &req,
                                   rokae_xmate3_ros2::srv::GetPosture::Response &res) const {
  std::string message;
  if (!detail::validate_coordinate_type(req.coordinate_type, message)) {
    res.success = false;
    res.message = message;
    return;
  }
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  const auto flange_pose = kinematics_.forwardKinematicsRPY(detail::snapshot_joints(pos));
  const auto pose = detail::resolve_pose_for_coordinate(flange_pose, tooling_state_.toolset(), req.coordinate_type);
  for (int i = 0; i < 6; ++i) {
    res.posture[i] = pose[i];
  }
  res.success = true;
  res.message.clear();
}

void QueryFacade::handleGetCartPosture(const rokae_xmate3_ros2::srv::GetCartPosture::Request &req,
                                       rokae_xmate3_ros2::srv::GetCartPosture::Response &res) const {
  std::string message;
  if (!detail::validate_coordinate_type(req.coordinate_type, message)) {
    res.success = false;
    res.message = message;
    return;
  }
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  const auto flange_pose = kinematics_.forwardKinematicsRPY(detail::snapshot_joints(pos));
  const auto pose = detail::resolve_pose_for_coordinate(flange_pose, tooling_state_.toolset(), req.coordinate_type);
  res.x = pose[0];
  res.y = pose[1];
  res.z = pose[2];
  res.rx = pose[3];
  res.ry = pose[4];
  res.rz = pose[5];
  res.success = true;
  res.message.clear();
}

void QueryFacade::handleGetBaseFrame(const rokae_xmate3_ros2::srv::GetBaseFrame::Request &req,
                                     rokae_xmate3_ros2::srv::GetBaseFrame::Response &res) const {
  (void)req;
  const auto base_pose = tooling_state_.baseFrame();
  for (size_t i = 0; i < 6 && i < base_pose.size(); ++i) {
    res.base_frame[i] = base_pose[i];
  }
  res.success = true;
}

void QueryFacade::handleGetToolset(const rokae_xmate3_ros2::srv::GetToolset::Request &req,
                                   rokae_xmate3_ros2::srv::GetToolset::Response &res) const {
  (void)req;
  const auto toolset = tooling_state_.toolset();
  res.tool_name = toolset.tool_name;
  res.wobj_name = toolset.wobj_name;
  res.tool_pose = toolset.tool_pose;
  res.wobj_pose = toolset.wobj_pose;
  res.success = true;
}

void QueryFacade::handleSetToolset(const rokae_xmate3_ros2::srv::SetToolset::Request &req,
                                   rokae_xmate3_ros2::srv::SetToolset::Response &res) const {
  tooling_state_.setToolset(req.tool_name, req.wobj_name, req.tool_pose, req.wobj_pose);
  res.success = true;
}

void QueryFacade::handleSetToolsetByName(const rokae_xmate3_ros2::srv::SetToolsetByName::Request &req,
                                         rokae_xmate3_ros2::srv::SetToolsetByName::Response &res) const {
  res.success = tooling_state_.setToolsetByName(req.tool_name, req.wobj_name);
  if (!res.success) {
    res.message = "unknown tool_name or wobj_name";
  }
}

void QueryFacade::handleGetSoftLimit(const rokae_xmate3_ros2::srv::GetSoftLimit::Request &req,
                                     rokae_xmate3_ros2::srv::GetSoftLimit::Response &res) const {
  (void)req;
  const auto soft_limit = motion_options_state_.softLimit();
  res.enable = soft_limit.enabled;
  for (int i = 0; i < 6; ++i) {
    res.limits[i * 2] = soft_limit.limits[i][0];
    res.limits[i * 2 + 1] = soft_limit.limits[i][1];
  }
  res.success = true;
}

void QueryFacade::handleGetRtJointData(const rokae_xmate3_ros2::srv::GetRtJointData::Request &req,
                                       rokae_xmate3_ros2::srv::GetRtJointData::Response &res) const {
  (void)req;
  if (session_state_.rtControlMode() < 0) {
    res.success = false;
    res.error_code = 2001;
    res.error_msg = "realtime control mode is not active";
    return;
  }
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  for (int i = 0; i < 6 && i < joint_num_; ++i) {
    res.joint_position[i] = pos[i];
    res.joint_velocity[i] = vel[i];
    res.joint_torque[i] = tau[i];
  }
  res.stamp = detail::ToBuiltinTime(time_provider_());
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void QueryFacade::handleGetAvoidSingularity(
    const rokae_xmate3_ros2::srv::GetAvoidSingularity::Request &req,
    rokae_xmate3_ros2::srv::GetAvoidSingularity::Response &res) const {
  (void)req;
  res.success = true;
  res.enabled = motion_options_state_.avoidSingularityEnabled();
  res.message = motion_options_state_.avoidSingularityEnabled() ? "enabled" : "disabled";
}

}  // namespace rokae_xmate3_ros2::runtime
