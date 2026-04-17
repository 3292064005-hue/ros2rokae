#include "runtime/service_facade.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <numeric>
#include <optional>
#include <utility>

#include <Eigen/Dense>

#include "runtime/planner_core.hpp"
#include "runtime/planning_utils.hpp"
#include "runtime/pose_utils.hpp"
#include "runtime/service_facade_utils.hpp"
#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"
#include "runtime/unified_retimer.hpp"
#include "rokae_xmate3_ros2/gazebo/model_facade.hpp"

namespace rokae_xmate3_ros2::runtime {

void QueryFacade::handleGetPowerState(const rokae_xmate3_ros2::srv::GetPowerState::Request &req,
                                      rokae_xmate3_ros2::srv::GetPowerState::Response &res) const {
  (void)req;
  const auto authority = authorityView();
  res.state.state = authority.power_on ? rokae_xmate3_ros2::msg::PowerState::ON
                                       : rokae_xmate3_ros2::msg::PowerState::OFF;
  res.success = true;
  res.message = "query_authority=runtime_request_coordinator";
}

void QueryFacade::handleGetInfo(const rokae_xmate3_ros2::srv::GetInfo::Request &req,
                                rokae_xmate3_ros2::srv::GetInfo::Response &res) const {
  (void)req;
  res.model = rokae_xmate3_ros2::spec::xmate3::robotModelName();
  res.robot_type = "3";
  res.serial_number = rokae_xmate3_ros2::spec::xmate3::runtimeSerialNumber();
  res.firmware_version = rokae_xmate3_ros2::spec::xmate3::controlSystemVersion();
  res.sdk_version = rokae_xmate3_ros2::spec::xmate3::controlSystemVersion();
  res.joint_num = static_cast<int32_t>(rokae_xmate3_ros2::spec::xmate3::kDoF);
  res.success = true;
  res.message = "query_authority=runtime_request_coordinator";
}

void QueryFacade::handleGetRuntimeStateSnapshot(
    const rokae_xmate3_ros2::srv::GetRuntimeStateSnapshot::Request &req,
    rokae_xmate3_ros2::srv::GetRuntimeStateSnapshot::Response &res) const {
  (void)req;
  try {
    std::array<double, 6> pos{};
    std::array<double, 6> vel{};
    std::array<double, 6> tau{};
    readAuthorityJointState(pos, vel, tau);

    const auto toolset = tooling_state_.toolset();
    const auto diagnostics = diagnostics_state_.snapshot();
    const auto runtime_view = authorityView();

    res.connected = runtime_view.connected;
    res.power_on = runtime_view.power_on;
    res.drag_mode = runtime_view.drag_mode;
    res.simulation_mode = session_state_.simulationMode();
    res.collision_detection_enabled = session_state_.collisionDetectionEnabled();
    res.motion_mode = runtime_view.motion_mode;
    res.operate_mode = session_state_.operateMode().mode;
    res.rt_mode = session_state_.rtControlMode();
    res.joint_position = pos;
    res.joint_velocity = vel;
    res.joint_torque = tau;

    const auto base_pose = tooling_state_.baseFrame();
    for (std::size_t i = 0; i < res.base_frame.size() && i < base_pose.size(); ++i) {
      res.base_frame[i] = base_pose[i];
    }

    res.tool_name = toolset.tool_name;
    res.wobj_name = toolset.wobj_name;
    res.tool_pose = toolset.tool_pose;
    res.wobj_pose = toolset.wobj_pose;
    res.tool_mass = toolset.tool_mass;
    res.tool_com = toolset.tool_com;
    res.backend_mode = diagnostics.backend_mode;
    res.active_profile = diagnostics.active_profile;
    res.runtime_phase = to_string(runtime_view.status.runtime_phase);
    res.control_owner = diagnostics.control_owner;
    res.rt_subscription_plan = diagnostics.rt_subscription_plan;
    res.profile_capability_summary = diagnostics.profile_capability_summary;
    res.runtime_option_summary = diagnostics.runtime_option_summary;
    res.planning_capability_summary = diagnostics.planning_capability_summary;
    res.model_exactness_summary = diagnostics.model_exactness_summary;
    res.catalog_provenance_summary = diagnostics.catalog_provenance_summary;
    res.success = true;
    res.message = "runtime-owned read snapshot; query_authority=runtime_request_coordinator";
  } catch (const std::exception &ex) {
    res.success = false;
    res.message = std::string{"runtime snapshot failed: "} + ex.what();
  } catch (...) {
    res.success = false;
    res.message = "runtime snapshot failed: unknown error";
  }
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
  const auto effective_count = std::min<std::uint32_t>(req.count, 10U);
  res.logs = data_store_state_.queryLogs(
      effective_count,
      req.level == 0 ? std::nullopt : std::optional<std::uint8_t>(req.level));
  res.success = true;
  if (req.count > effective_count) {
    res.message = "query_controller_log count clamped to 10";
  }
}

void QueryFacade::handleGetJointPos(const rokae_xmate3_ros2::srv::GetJointPos::Request &req,
                                    rokae_xmate3_ros2::srv::GetJointPos::Response &res) const {
  (void)req;
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  readAuthorityJointState(pos, vel, tau);
  for (int i = 0; i < 6 && i < joint_num_; ++i) {
    res.joint_positions[i] = pos[i];
  }
  res.success = true;
  res.message = "query_authority=runtime_request_coordinator";
}

void QueryFacade::handleGetJointVel(const rokae_xmate3_ros2::srv::GetJointVel::Request &req,
                                    rokae_xmate3_ros2::srv::GetJointVel::Response &res) const {
  (void)req;
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  readAuthorityJointState(pos, vel, tau);
  for (int i = 0; i < 6 && i < joint_num_; ++i) {
    res.joint_vel[i] = vel[i];
  }
  res.success = true;
  res.message = "query_authority=runtime_request_coordinator";
}

void QueryFacade::handleGetJointTorques(const rokae_xmate3_ros2::srv::GetJointTorques::Request &req,
                                       rokae_xmate3_ros2::srv::GetJointTorques::Response &res) const {
  (void)req;
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  readAuthorityJointState(pos, vel, tau);
  for (int i = 0; i < 6 && i < joint_num_; ++i) {
    res.joint_torque[i] = tau[i];
  }
  res.success = true;
  res.message = "query_authority=runtime_request_coordinator";
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
  readAuthorityJointState(pos, vel, tau);
  const auto flange_pose = kinematics_.forwardKinematicsRPY(detail::snapshot_joints(pos));
  const auto pose = detail::resolve_pose_for_coordinate(flange_pose, tooling_state_.toolset(), req.coordinate_type);
  for (int i = 0; i < 6; ++i) {
    res.posture[i] = pose[i];
  }
  res.success = true;
  res.message = "query_authority=runtime_request_coordinator";
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
  readAuthorityJointState(pos, vel, tau);
  const auto flange_pose = kinematics_.forwardKinematicsRPY(detail::snapshot_joints(pos));
  const auto pose = detail::resolve_pose_for_coordinate(flange_pose, tooling_state_.toolset(), req.coordinate_type);
  res.x = pose[0];
  res.y = pose[1];
  res.z = pose[2];
  res.rx = pose[3];
  res.ry = pose[4];
  res.rz = pose[5];
  res.success = true;
  res.message = "query_authority=runtime_request_coordinator";
}

void QueryFacade::handleGetBaseFrame(const rokae_xmate3_ros2::srv::GetBaseFrame::Request &req,
                                     rokae_xmate3_ros2::srv::GetBaseFrame::Response &res) const {
  (void)req;
  const auto base_pose = tooling_state_.baseFrame();
  for (size_t i = 0; i < 6 && i < base_pose.size(); ++i) {
    res.base_frame[i] = base_pose[i];
  }
  res.success = true;
  res.message = "query_authority=runtime_request_coordinator";
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
  res.message = "query_authority=runtime_request_coordinator";
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
  res.message = "query_authority=runtime_request_coordinator";
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
  readAuthorityJointState(pos, vel, tau);
  for (int i = 0; i < 6 && i < joint_num_; ++i) {
    res.joint_position[i] = pos[i];
    res.joint_velocity[i] = vel[i];
    res.joint_torque[i] = tau[i];
  }
  res.stamp = detail::ToBuiltinTime(time_provider_());
  res.success = true;
  res.error_code = 0;
  res.error_msg = "query_authority=runtime_request_coordinator";
}

void QueryFacade::handleGetAvoidSingularity(
    const rokae_xmate3_ros2::srv::GetAvoidSingularity::Request &req,
    rokae_xmate3_ros2::srv::GetAvoidSingularity::Response &res) const {
  (void)req;
  res.success = false;
  res.enabled = false;
  res.message = "avoid singularity is not supported on the xMate6 compatibility lane";
}

}  // namespace rokae_xmate3_ros2::runtime
