#include "runtime/service_facade.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <utility>

#include "runtime/planning_utils.hpp"
#include "runtime/pose_utils.hpp"
#include "runtime/unified_retimer.hpp"
#include "rokae_xmate3_ros2/gazebo/approximate_model.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

static builtin_interfaces::msg::Time ToBuiltinTime(const rclcpp::Time &time) {
  builtin_interfaces::msg::Time stamp;
  const auto ns = time.nanoseconds();
  stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return stamp;
}

std::string basename_without_extension(const std::string &path) {
  std::string name = path;
  const auto slash = name.find_last_of("/\\");
  if (slash != std::string::npos) {
    name = name.substr(slash + 1);
  }
  const auto dot = name.find_last_of('.');
  if (dot != std::string::npos) {
    name = name.substr(0, dot);
  }
  return name.empty() ? std::string("default_rl_project") : name;
}

std::vector<double> snapshot_joints(const std::array<double, 6> &position) {
  return std::vector<double>(position.begin(), position.end());
}

std::vector<double> resolve_pose_for_coordinate(const std::vector<double> &flange_pose,
                                                const ToolsetSnapshot &toolset,
                                                int coordinate_type) {
  if (coordinate_type == 1) {
    return pose_utils::convertFlangeInBaseToEndInRef(flange_pose, toolset.tool_pose, toolset.wobj_pose);
  }
  return flange_pose;
}

bool validate_coordinate_type(int coordinate_type, std::string &message) {
  if (coordinate_type == 0 || coordinate_type == 1) {
    return true;
  }
  message = "unsupported coordinate_type";
  return false;
}

}  // namespace

ControlFacade::ControlFacade(SessionState &session_state,
                             MotionOptionsState &motion_options_state,
                             BackendInterface *backend,
                             MotionRuntime *motion_runtime,
                             MotionRequestCoordinator *request_coordinator)
    : session_state_(session_state),
      motion_options_state_(motion_options_state),
      backend_(backend),
      motion_runtime_(motion_runtime),
      request_coordinator_(request_coordinator) {}

void ControlFacade::stopRuntime(const std::string &message) const {
  if (request_coordinator_ != nullptr) {
    request_coordinator_->stop(message);
    return;
  }
  if (motion_runtime_ != nullptr) {
    motion_runtime_->stop(message);
  }
}

void ControlFacade::clearBackendControl() const {
  if (backend_ != nullptr) {
    backend_->clearControl();
  }
}

void ControlFacade::syncBrakeState(bool locked) const {
  if (backend_ == nullptr) {
    return;
  }
  const auto snapshot = backend_->readSnapshot();
  if (backend_->brakesLocked() != locked) {
    backend_->setBrakeLock(snapshot, locked);
  }
  if (locked) {
    backend_->clearControl();
  }
}

void ControlFacade::handleConnect(const rokae_xmate3_ros2::srv::Connect::Request &req,
                                  rokae_xmate3_ros2::srv::Connect::Response &res) const {
  session_state_.connect(req.remote_ip);
  res.success = true;
  res.message = "Connected to xCore Gazebo Controller";
}

void ControlFacade::handleDisconnect(const rokae_xmate3_ros2::srv::Disconnect::Request &req,
                                     rokae_xmate3_ros2::srv::Disconnect::Response &res) const {
  (void)req;
  stopRuntime("disconnect");
  session_state_.setDragMode(false);
  session_state_.setPowerOn(false);
  session_state_.disconnect();
  syncBrakeState(true);
  res.success = true;
}

void ControlFacade::handleSetPowerState(const rokae_xmate3_ros2::srv::SetPowerState::Request &req,
                                        rokae_xmate3_ros2::srv::SetPowerState::Response &res) const {
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
  session_state_.setPowerOn(req.on);
  if (!req.on) {
    session_state_.setDragMode(false);
    stopRuntime("power off");
    syncBrakeState(true);
  } else {
    syncBrakeState(false);
  }
  res.success = true;
}

void ControlFacade::handleSetOperateMode(const rokae_xmate3_ros2::srv::SetOperateMode::Request &req,
                                         rokae_xmate3_ros2::srv::SetOperateMode::Response &res) const {
  session_state_.setOperateMode(req.mode);
  res.success = true;
}

void ControlFacade::handleClearServoAlarm(const rokae_xmate3_ros2::srv::ClearServoAlarm::Request &req,
                                          rokae_xmate3_ros2::srv::ClearServoAlarm::Response &res) const {
  (void)req;
  res.success = true;
}

void ControlFacade::handleEnableCollisionDetection(
    const rokae_xmate3_ros2::srv::EnableCollisionDetection::Request &req,
    rokae_xmate3_ros2::srv::EnableCollisionDetection::Response &res) const {
  std::array<double, 6> sensitivity{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  if (!req.sensitivity.empty()) {
    if (req.sensitivity.size() < 6) {
      res.success = false;
      res.message = "collision sensitivity requires 6 joint values";
      return;
    }
    for (int i = 0; i < 6; ++i) {
      sensitivity[i] = std::clamp(req.sensitivity[i], 0.1, 10.0);
    }
  }
  session_state_.setCollisionDetectionEnabled(true);
  session_state_.setCollisionDetectionConfig(sensitivity, req.behaviour, req.fallback);
  res.success = true;
  res.message = "collision detection enabled";
}

void ControlFacade::handleDisableCollisionDetection(
    const rokae_xmate3_ros2::srv::DisableCollisionDetection::Request &req,
    rokae_xmate3_ros2::srv::DisableCollisionDetection::Response &res) const {
  (void)req;
  session_state_.setCollisionDetectionEnabled(false);
  res.success = true;
  res.message = "collision detection disabled";
}

void ControlFacade::handleSetSoftLimit(const rokae_xmate3_ros2::srv::SetSoftLimit::Request &req,
                                       rokae_xmate3_ros2::srv::SetSoftLimit::Response &res) const {
  auto soft_limit = motion_options_state_.softLimit();
  if (req.limits.size() >= 12) {
    for (int i = 0; i < 6; ++i) {
      soft_limit.limits[i][0] = req.limits[i * 2];
      soft_limit.limits[i][1] = req.limits[i * 2 + 1];
    }
  }
  motion_options_state_.setSoftLimit(req.enable, soft_limit.limits);
  res.success = true;
}

void ControlFacade::handleSetMotionControlMode(
    const rokae_xmate3_ros2::srv::SetMotionControlMode::Request &req,
    rokae_xmate3_ros2::srv::SetMotionControlMode::Response &res) const {
  if (session_state_.motionMode() != req.mode) {
    stopRuntime("motion control mode changed");
    clearBackendControl();
  }
  session_state_.setMotionMode(req.mode);
  res.success = true;
}

void ControlFacade::handleMoveReset(const rokae_xmate3_ros2::srv::MoveReset::Request &req,
                                    rokae_xmate3_ros2::srv::MoveReset::Response &res) const {
  (void)req;
  if (request_coordinator_ != nullptr) {
    request_coordinator_->reset();
  } else if (motion_runtime_ != nullptr) {
    motion_runtime_->reset();
  }
  clearBackendControl();
  res.success = true;
}

void ControlFacade::handleMoveStart(const rokae_xmate3_ros2::srv::MoveStart::Request &req,
                                    rokae_xmate3_ros2::srv::MoveStart::Response &res) const {
  (void)req;
  if (!session_state_.powerOn()) {
    res.success = false;
    res.message = "Robot not powered on";
    return;
  }
  res.success = true;
}

void ControlFacade::handleStop(const rokae_xmate3_ros2::srv::Stop::Request &req,
                               rokae_xmate3_ros2::srv::Stop::Response &res) const {
  (void)req;
  stopRuntime("stop service called");
  clearBackendControl();
  res.success = true;
}

void ControlFacade::handleSetDefaultSpeed(const rokae_xmate3_ros2::srv::SetDefaultSpeed::Request &req,
                                          rokae_xmate3_ros2::srv::SetDefaultSpeed::Response &res) const {
  if (req.speed < 5 || req.speed > 4000) {
    res.success = false;
    res.message = "default speed must be within [5, 4000] mm/s";
    return;
  }
  motion_options_state_.setDefaultSpeed(req.speed);
  res.success = true;
}

void ControlFacade::handleSetDefaultZone(const rokae_xmate3_ros2::srv::SetDefaultZone::Request &req,
                                         rokae_xmate3_ros2::srv::SetDefaultZone::Response &res) const {
  const auto zone_range = motion_options_state_.zoneValidRange();
  if (req.zone < zone_range[0] || req.zone > zone_range[1]) {
    res.success = false;
    res.message = "default zone must be within [" + std::to_string(zone_range[0]) + ", " +
                  std::to_string(zone_range[1]) + "] mm";
    return;
  }
  motion_options_state_.setDefaultZone(req.zone);
  res.success = true;
}

void ControlFacade::handleSetDefaultConfOpt(
    const rokae_xmate3_ros2::srv::SetDefaultConfOpt::Request &req,
    rokae_xmate3_ros2::srv::SetDefaultConfOpt::Response &res) const {
  motion_options_state_.setDefaultConfOpt(req.forced);
  res.success = true;
}

void ControlFacade::handleAdjustSpeedOnline(
    const rokae_xmate3_ros2::srv::AdjustSpeedOnline::Request &req,
    rokae_xmate3_ros2::srv::AdjustSpeedOnline::Response &res) const {
  if (req.scale < 0.05 || req.scale > 2.0) {
    res.success = false;
    res.message = "speed scale must be within [0.05, 2.0]";
    return;
  }
  motion_options_state_.setSpeedScale(req.scale);
  if (motion_runtime_ != nullptr) {
    motion_runtime_->setActiveSpeedScale(req.scale);
  }
  res.success = true;
}

void ControlFacade::handleSetRtControlMode(const rokae_xmate3_ros2::srv::SetRtControlMode::Request &req,
                                           rokae_xmate3_ros2::srv::SetRtControlMode::Response &res) const {
  if (!session_state_.powerOn()) {
    res.success = false;
    res.error_code = 1002;
    res.error_msg = "robot power is off";
    return;
  }
  if (req.mode < 0 || req.mode > 4) {
    res.success = false;
    res.error_code = 1001;
    res.error_msg = "invalid realtime control mode";
    return;
  }
  stopRuntime("rt control mode changed");
  clearBackendControl();
  session_state_.setRtControlMode(req.mode);
  session_state_.setMotionMode(1);
  syncBrakeState(false);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void ControlFacade::handleSetSimulationMode(
    const rokae_xmate3_ros2::srv::SetSimulationMode::Request &req,
    rokae_xmate3_ros2::srv::SetSimulationMode::Response &res) const {
  session_state_.setSimulationMode(req.state);
  res.success = true;
}

void ControlFacade::handleEnableDrag(const rokae_xmate3_ros2::srv::EnableDrag::Request &req,
                                     rokae_xmate3_ros2::srv::EnableDrag::Response &res) const {
  (void)req;
  session_state_.setDragMode(true);
  stopRuntime("drag mode enabled");
  syncBrakeState(false);
  clearBackendControl();
  res.success = true;
}

void ControlFacade::handleDisableDrag(const rokae_xmate3_ros2::srv::DisableDrag::Request &req,
                                      rokae_xmate3_ros2::srv::DisableDrag::Response &res) const {
  (void)req;
  session_state_.setDragMode(false);
  clearBackendControl();
  res.success = true;
}

void ControlFacade::handleSetAvoidSingularity(
    const rokae_xmate3_ros2::srv::SetAvoidSingularity::Request &req,
    rokae_xmate3_ros2::srv::SetAvoidSingularity::Response &res) const {
  motion_options_state_.setAvoidSingularity(req.enable);
  res.success = true;
  res.message = motion_options_state_.avoidSingularityEnabled() ? "enabled" : "disabled";
}

QueryFacade::QueryFacade(SessionState &session_state,
                         MotionOptionsState &motion_options_state,
                         ToolingState &tooling_state,
                         DataStoreState &data_store_state,
                         ProgramState &program_state,
                         gazebo::xMate3Kinematics &kinematics,
                         JointStateFetcher joint_state_fetcher,
                         TimeProvider time_provider,
                         TrajectoryDtProvider trajectory_dt_provider,
                         int joint_num)
    : session_state_(session_state),
      motion_options_state_(motion_options_state),
      tooling_state_(tooling_state),
      data_store_state_(data_store_state),
      program_state_(program_state),
      kinematics_(kinematics),
      joint_state_fetcher_(std::move(joint_state_fetcher)),
      time_provider_(std::move(time_provider)),
      trajectory_dt_provider_(std::move(trajectory_dt_provider)),
      joint_num_(joint_num) {}

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

void QueryFacade::handleGetJointTorque(const rokae_xmate3_ros2::srv::GetJointTorque::Request &req,
                                       rokae_xmate3_ros2::srv::GetJointTorque::Response &res) const {
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
  if (!validate_coordinate_type(req.coordinate_type, message)) {
    res.success = false;
    res.message = message;
    return;
  }
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  const auto flange_pose = kinematics_.forwardKinematicsRPY(snapshot_joints(pos));
  const auto pose = resolve_pose_for_coordinate(flange_pose, tooling_state_.toolset(), req.coordinate_type);
  for (int i = 0; i < 6; ++i) {
    res.posture[i] = pose[i];
  }
  res.success = true;
  res.message.clear();
}

void QueryFacade::handleGetCartPosture(const rokae_xmate3_ros2::srv::GetCartPosture::Request &req,
                                       rokae_xmate3_ros2::srv::GetCartPosture::Response &res) const {
  std::string message;
  if (!validate_coordinate_type(req.coordinate_type, message)) {
    res.success = false;
    res.message = message;
    return;
  }
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  const auto flange_pose = kinematics_.forwardKinematicsRPY(snapshot_joints(pos));
  const auto pose = resolve_pose_for_coordinate(flange_pose, tooling_state_.toolset(), req.coordinate_type);
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

void QueryFacade::handleCalcFk(const rokae_xmate3_ros2::srv::CalcFk::Request &req,
                               rokae_xmate3_ros2::srv::CalcFk::Response &res) const {
  std::vector<double> joints(req.joint_positions.begin(), req.joint_positions.end());
  const auto flange_pose = kinematics_.forwardKinematicsRPY(joints);
  const auto pose = resolve_pose_for_coordinate(flange_pose, tooling_state_.toolset(), 1);
  for (int i = 0; i < 6; ++i) {
    res.posture[i] = pose[i];
  }
  res.success = true;
}

void QueryFacade::handleCalcIk(const rokae_xmate3_ros2::srv::CalcIk::Request &req,
                               rokae_xmate3_ros2::srv::CalcIk::Response &res) const {
  std::vector<double> target(req.target_posture.begin(), req.target_posture.end());
  target = pose_utils::convertEndInRefToFlangeInBase(
      target, tooling_state_.toolset().tool_pose, tooling_state_.toolset().wobj_pose);
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  const auto current = snapshot_joints(pos);
  auto candidates = kinematics_.inverseKinematicsMultiSolution(target, current);
  const auto seeded_fast = kinematics_.inverseKinematicsSeededFast(target, current);
  if (!seeded_fast.empty()) {
    candidates.insert(candidates.begin(), seeded_fast);
  }
  const auto soft_limit = motion_options_state_.softLimit();
  const auto selected = select_ik_solution(
      kinematics_, candidates, target, current, req.conf_data,
      motion_options_state_.defaultConfOptForced(), true, soft_limit.enabled, soft_limit.limits);
  if (!selected.success) {
    res.success = false;
    res.message = selected.message;
    return;
  }
  for (size_t i = 0; i < 6 && i < selected.joints.size(); ++i) {
    res.joint_positions[i] = selected.joints[i];
  }
  res.success = true;
  res.message.clear();
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
  res.stamp = ToBuiltinTime(time_provider_());
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

void QueryFacade::handleCalcJointTorque(
    const rokae_xmate3_ros2::srv::CalcJointTorque::Request &req,
    rokae_xmate3_ros2::srv::CalcJointTorque::Response &res) const {
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(req.joint_pos[i]) || !std::isfinite(req.joint_vel[i]) ||
        !std::isfinite(req.joint_acc[i]) || !std::isfinite(req.external_force[i])) {
      res.success = false;
      res.error_code = 4002;
      res.error_msg = "joint torque inputs must be finite";
      return;
    }
  }
  const auto toolset = tooling_state_.toolset();
  const auto model_facade =
      rokae_xmate3_ros2::gazebo_model::configuredModelFacade(
          kinematics_,
          {toolset.tool_pose[0], toolset.tool_pose[1], toolset.tool_pose[2],
           toolset.tool_pose[3], toolset.tool_pose[4], toolset.tool_pose[5]},
          {toolset.tool_mass, toolset.tool_com});
  res.joint_torque = model_facade.inverseDynamics(req.joint_pos, req.joint_vel, req.joint_acc, req.external_force);
  res.gravity_torque = model_facade.gravity(req.joint_pos);
  res.coriolis_torque = model_facade.coriolis(req.joint_pos, req.joint_vel);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void QueryFacade::handleGenerateSTrajectory(
    const rokae_xmate3_ros2::srv::GenerateSTrajectory::Request &req,
    rokae_xmate3_ros2::srv::GenerateSTrajectory::Response &res) const {
  std::vector<double> start(req.start_joint_pos.begin(), req.start_joint_pos.end());
  std::vector<double> target(req.target_joint_pos.begin(), req.target_joint_pos.end());
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(start[i]) || !std::isfinite(target[i])) {
      res.success = false;
      res.error_code = 3003;
      res.error_msg = "trajectory inputs must be finite";
      return;
    }
  }
  if (start.size() != 6 || target.size() != 6) {
    res.success = false;
    res.error_code = 3003;
    res.error_msg = "joint-space S-trajectory requires 6 joint values";
    return;
  }

  double max_delta = 0.0;
  for (size_t i = 0; i < start.size(); ++i) {
    max_delta = std::max(max_delta, std::fabs(target[i] - start[i]));
  }
  if (max_delta < 1e-9) {
    rokae_xmate3_ros2::msg::JointPos6 joint_msg;
    for (int i = 0; i < 6; ++i) {
      joint_msg.pos[i] = start[i];
    }
    res.trajectory_points = {joint_msg};
    res.total_time = 0.0;
    res.stamp = ToBuiltinTime(time_provider_());
    res.success = true;
    res.error_code = 0;
    return;
  }

  const double sample_dt = std::max(trajectory_dt_provider_(), 1e-3);
  std::vector<std::vector<double>> trajectory_positions;
  double total_time = 0.0;
  bool cartesian_fallback_to_joint = false;
  if (req.is_cartesian) {
    const auto cartesian_retimed = buildApproximateCartesianSTrajectory(kinematics_, req, sample_dt);
    if (cartesian_retimed.empty()) {
      cartesian_fallback_to_joint = true;
    } else {
      trajectory_positions = cartesian_retimed.joint_trajectory;
      total_time = cartesian_retimed.total_time;
    }
  }

  if (!req.is_cartesian || cartesian_fallback_to_joint) {
    const auto retimed = retimeJointWithUnifiedConfig(
        start, target, sample_dt, req.max_velocity, req.max_acceleration, req.blend_radius);
    if (retimed.empty()) {
      res.success = false;
      res.error_code = 3003;
      res.error_msg = "trajectory planning failed";
      return;
    }
    trajectory_positions = retimed.positions;
    total_time = retimed.total_time;
  }

  res.trajectory_points.clear();
  res.trajectory_points.reserve(trajectory_positions.size());
  for (const auto &point : trajectory_positions) {
    rokae_xmate3_ros2::msg::JointPos6 joint_msg;
    for (int j = 0; j < 6; ++j) {
      joint_msg.pos[j] = point[j];
    }
    res.trajectory_points.push_back(joint_msg);
  }
  res.total_time = total_time;
  res.stamp = ToBuiltinTime(time_provider_());
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void QueryFacade::handleMapCartesianToJointTorque(
    const rokae_xmate3_ros2::srv::MapCartesianToJointTorque::Request &req,
    rokae_xmate3_ros2::srv::MapCartesianToJointTorque::Response &res) const {
  std::vector<double> joints(req.joint_pos.begin(), req.joint_pos.end());
  if (joints.size() != 6) {
    res.success = false;
    res.error_code = 6002;
    res.error_msg = "joint_pos size is invalid";
    return;
  }
  const Eigen::MatrixXd jacobian = kinematics_.computeJacobian(joints);
  Eigen::Matrix<double, 6, 1> wrench;
  for (int i = 0; i < 6; ++i) {
    wrench(i) = req.cart_force[i];
  }
  const Eigen::Matrix<double, 6, 1> tau = jacobian.transpose() * wrench;
  for (int i = 0; i < 6; ++i) {
    res.joint_torque[i] = tau(i);
  }
  res.stamp = ToBuiltinTime(time_provider_());
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void QueryFacade::handleGetEndTorque(const rokae_xmate3_ros2::srv::GetEndTorque::Request &req,
                                     rokae_xmate3_ros2::srv::GetEndTorque::Response &res) const {
  (void)req;
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau_array{};
  joint_state_fetcher_(pos, vel, tau_array);
  const auto joints = snapshot_joints(pos);
  Eigen::Matrix<double, 6, 1> tau = Eigen::Matrix<double, 6, 1>::Zero();
  for (int i = 0; i < 6; ++i) {
    tau(i) = tau_array[i];
  }
  const Eigen::MatrixXd jacobian = kinematics_.computeJacobian(joints);
  const Eigen::Matrix<double, 6, 1> wrench =
      jacobian.transpose().completeOrthogonalDecomposition().solve(tau);
  for (int i = 0; i < 6; ++i) {
    res.end_torque[i] = wrench(i);
  }
  res.success = true;
  res.message.clear();
}

IoProgramFacade::IoProgramFacade(DataStoreState &data_store_state, ProgramState &program_state, TimeProvider time_provider)
    : data_store_state_(data_store_state), program_state_(program_state), time_provider_(std::move(time_provider)) {}

void IoProgramFacade::handleSendCustomData(
    const rokae_xmate3_ros2::srv::SendCustomData::Request &req,
    rokae_xmate3_ros2::srv::SendCustomData::Response &res) const {
  data_store_state_.setCustomData(req.data_topic, req.custom_data);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.response_data = std::string("ACK:") + req.data_topic;
  res.send_time = ToBuiltinTime(time_provider_());
}

void IoProgramFacade::handleRegisterDataCallback(
    const rokae_xmate3_ros2::srv::RegisterDataCallback::Request &req,
    rokae_xmate3_ros2::srv::RegisterDataCallback::Response &res) const {
  if (req.data_topic.empty() || req.callback_id.empty()) {
    res.success = false;
    res.error_code = 12001;
    res.error_msg = "data_topic and callback_id must not be empty";
    return;
  }
  if (data_store_state_.hasCallback(req.callback_id)) {
    res.success = false;
    res.error_code = 12004;
    res.error_msg = "callback_id already exists";
    return;
  }
  data_store_state_.registerCallback(req.callback_id, req.data_topic);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.register_time = ToBuiltinTime(time_provider_());
}

void IoProgramFacade::handleReadRegister(const rokae_xmate3_ros2::srv::ReadRegister::Request &req,
                                         rokae_xmate3_ros2::srv::ReadRegister::Response &res) const {
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.value = data_store_state_.registerValue(req.key);
}

void IoProgramFacade::handleWriteRegister(const rokae_xmate3_ros2::srv::WriteRegister::Request &req,
                                          rokae_xmate3_ros2::srv::WriteRegister::Response &res) const {
  data_store_state_.setRegister(req.key, req.value);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void IoProgramFacade::handleLoadRlProject(const rokae_xmate3_ros2::srv::LoadRLProject::Request &req,
                                          rokae_xmate3_ros2::srv::LoadRLProject::Response &res) const {
  const auto effective_project_name = basename_without_extension(req.project_path);
  program_state_.loadRlProject(req.project_path, effective_project_name);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.project_name = program_state_.loadedRlProjectName();
  res.load_time = 0.0;
}

void IoProgramFacade::handleStartRlProject(const rokae_xmate3_ros2::srv::StartRLProject::Request &req,
                                           rokae_xmate3_ros2::srv::StartRLProject::Response &res) const {
  if (!program_state_.rlProjectLoaded() || req.project_id != program_state_.loadedRlProjectName()) {
    res.success = false;
    res.error_code = 7001;
    res.error_msg = "RL project is not loaded";
    return;
  }
  if (program_state_.rlProjectRunning()) {
    res.success = false;
    res.error_code = 7002;
    res.error_msg = "RL project is already running";
    return;
  }
  program_state_.setRlProjectRunning(true, 1);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.current_episode = program_state_.rlCurrentEpisode();
  res.start_time = ToBuiltinTime(time_provider_());
}

void IoProgramFacade::handleStopRlProject(const rokae_xmate3_ros2::srv::StopRLProject::Request &req,
                                          rokae_xmate3_ros2::srv::StopRLProject::Response &res) const {
  if (!program_state_.rlProjectRunning() || req.project_id != program_state_.loadedRlProjectName()) {
    res.success = false;
    res.error_code = 8001;
    res.error_msg = "RL project is not running";
    return;
  }
  program_state_.setRlProjectRunning(false, program_state_.rlCurrentEpisode());
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.finished_episode = std::max(program_state_.rlCurrentEpisode(), 1);
  res.stop_time = ToBuiltinTime(time_provider_());
}

void IoProgramFacade::handleGetDI(const rokae_xmate3_ros2::srv::GetDI::Request &req,
                                  rokae_xmate3_ros2::srv::GetDI::Response &res) const {
  res.state = data_store_state_.getDI(req.board, req.port);
  res.success = true;
}

void IoProgramFacade::handleGetDO(const rokae_xmate3_ros2::srv::GetDO::Request &req,
                                  rokae_xmate3_ros2::srv::GetDO::Response &res) const {
  res.state = data_store_state_.getDO(req.board, req.port);
  res.success = true;
}

void IoProgramFacade::handleSetDI(const rokae_xmate3_ros2::srv::SetDI::Request &req,
                                  rokae_xmate3_ros2::srv::SetDI::Response &res) const {
  data_store_state_.setDI(req.board, req.port, req.state);
  res.success = true;
}

void IoProgramFacade::handleSetDO(const rokae_xmate3_ros2::srv::SetDO::Request &req,
                                  rokae_xmate3_ros2::srv::SetDO::Response &res) const {
  data_store_state_.setDO(req.board, req.port, req.state);
  res.success = true;
}

void IoProgramFacade::handleGetAI(const rokae_xmate3_ros2::srv::GetAI::Request &req,
                                  rokae_xmate3_ros2::srv::GetAI::Response &res) const {
  res.value = data_store_state_.getAI(req.board, req.port);
  res.success = true;
}

void IoProgramFacade::handleSetAO(const rokae_xmate3_ros2::srv::SetAO::Request &req,
                                  rokae_xmate3_ros2::srv::SetAO::Response &res) const {
  data_store_state_.setAO(req.board, req.port, req.value);
  res.success = true;
}

PathFacade::PathFacade(ProgramState &program_state,
                       ToolingState &tooling_state,
                       MotionRequestCoordinator *request_coordinator,
                       JointStateFetcher joint_state_fetcher,
                       TrajectoryDtProvider trajectory_dt_provider,
                       RequestIdGenerator request_id_generator)
    : program_state_(program_state),
      tooling_state_(tooling_state),
      request_coordinator_(request_coordinator),
      joint_state_fetcher_(std::move(joint_state_fetcher)),
      trajectory_dt_provider_(std::move(trajectory_dt_provider)),
      request_id_generator_(std::move(request_id_generator)) {}

void PathFacade::handleStartRecordPath(const rokae_xmate3_ros2::srv::StartRecordPath::Request &req,
                                       rokae_xmate3_ros2::srv::StartRecordPath::Response &res) const {
  (void)req;
  program_state_.startRecordingPath(tooling_state_.toolset(), "sdk_record");
  res.success = true;
}

void PathFacade::handleStopRecordPath(const rokae_xmate3_ros2::srv::StopRecordPath::Request &req,
                                      rokae_xmate3_ros2::srv::StopRecordPath::Response &res) const {
  (void)req;
  program_state_.stopRecordingPath();
  res.success = true;
}

void PathFacade::handleCancelRecordPath(
    const rokae_xmate3_ros2::srv::CancelRecordPath::Request &req,
    rokae_xmate3_ros2::srv::CancelRecordPath::Response &res) const {
  (void)req;
  program_state_.cancelRecordingPath();
  res.success = true;
}

void PathFacade::handleSaveRecordPath(const rokae_xmate3_ros2::srv::SaveRecordPath::Request &req,
                                      rokae_xmate3_ros2::srv::SaveRecordPath::Response &res) const {
  program_state_.saveRecordedPath(req.name);
  res.success = true;
}

void PathFacade::handleReplayPath(const rokae_xmate3_ros2::srv::ReplayPath::Request &req,
                                  rokae_xmate3_ros2::srv::ReplayPath::Response &res) const {
  ReplayPathAsset replay_asset;
  if (!program_state_.getReplayAsset(req.name, replay_asset)) {
    res.success = false;
    res.message = "Path not found";
    return;
  }
  if (replay_asset.samples.empty()) {
    res.success = false;
    res.message = "Path is empty";
    return;
  }
  if (request_coordinator_ == nullptr) {
    res.success = false;
    res.message = "Runtime is not initialized";
    return;
  }
  if (!request_coordinator_->canAcceptRequest()) {
    res.success = false;
    res.message = "Runtime is busy";
    return;
  }
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  if (!replay_asset.toolset.tool_name.empty() || !replay_asset.toolset.wobj_name.empty()) {
    tooling_state_.setToolset(replay_asset.toolset.tool_name,
                              replay_asset.toolset.wobj_name,
                              replay_asset.toolset.tool_pose,
                              replay_asset.toolset.wobj_pose);
    tooling_state_.setBaseFrame(replay_asset.toolset.base_pose);
    tooling_state_.setToolDynamics(replay_asset.toolset.tool_name,
                                   replay_asset.toolset.tool_mass,
                                   replay_asset.toolset.tool_com);
  }
  const auto submission = request_coordinator_->submitReplayPath(
      replay_asset,
      req.rate,
      pos,
      trajectory_dt_provider_(),
      request_id_generator_(std::string("replay_") + req.name));
  res.success = submission.success;
  res.message = submission.message;
}

void PathFacade::handleRemovePath(const rokae_xmate3_ros2::srv::RemovePath::Request &req,
                                  rokae_xmate3_ros2::srv::RemovePath::Response &res) const {
  program_state_.removeSavedPath(req.name, req.remove_all);
  res.success = true;
}

void PathFacade::handleQueryPathLists(const rokae_xmate3_ros2::srv::QueryPathLists::Request &req,
                                      rokae_xmate3_ros2::srv::QueryPathLists::Response &res) const {
  (void)req;
  res.path_names = program_state_.querySavedPaths();
  res.success = true;
}


}  // namespace rokae_xmate3_ros2::runtime
