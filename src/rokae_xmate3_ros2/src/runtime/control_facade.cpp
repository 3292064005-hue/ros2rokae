#include "runtime/service_facade.hpp"

#include <algorithm>
#include <cstdint>
#include <sstream>

#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"

namespace rokae_xmate3_ros2::runtime {

namespace {

constexpr std::uint8_t kOperateModeManual = static_cast<std::uint8_t>(rokae::OperateMode::manual);
constexpr std::uint8_t kOperateModeAutomatic = static_cast<std::uint8_t>(rokae::OperateMode::automatic);
constexpr int kMotionModeMin = kSessionMotionModeMin;
constexpr int kMotionModeMax = kSessionMotionModeMax;
constexpr int kMotionModeNrt = kSessionMotionModeNrt;
constexpr int kMotionModeRt = kSessionMotionModeRt;
constexpr int kMotionModeRl = kSessionMotionModeRl;

bool requireConnected(const SessionState &session_state, bool &success, std::string &message) {
  if (session_state.connected()) {
    return true;
  }
  success = false;
  message = "Robot not connected";
  return false;
}

bool requireConnected(const SessionState &session_state, bool &success, int32_t &error_code, std::string &error_msg) {
  if (session_state.connected()) {
    return true;
  }
  success = false;
  error_code = 1000;
  error_msg = "robot not connected";
  return false;
}

}  // namespace

ControlFacade::ControlFacade(SessionState &session_state,
                             MotionOptionsState &motion_options_state,
                             ToolingState &tooling_state,
                             BackendInterface *backend,
                             MotionRuntime *motion_runtime,
                             MotionRequestCoordinator *request_coordinator)
    : session_state_(session_state),
      motion_options_state_(motion_options_state),
      tooling_state_(tooling_state),
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
  if (session_state_.connected() && session_state_.remoteIp() == req.remote_ip) {
    res.success = true;
    res.message = "connect request treated as idempotent success";
    return;
  }
  session_state_.connect(req.remote_ip);
  res.success = true;
  res.message = "Connected to xCore Gazebo Controller";
}

void ControlFacade::handleDisconnect(const rokae_xmate3_ros2::srv::Disconnect::Request &req,
                                     rokae_xmate3_ros2::srv::Disconnect::Response &res) const {
  (void)req;
  if (!session_state_.connected()) {
    res.success = true;
    res.message = "disconnect request treated as idempotent success";
    return;
  }
  stopRuntime("disconnect");
  session_state_.setDragMode(false);
  session_state_.setPowerOn(false);
  session_state_.disconnect();
  syncBrakeState(true);
  res.success = true;
  res.message = "disconnected";
}

void ControlFacade::handleSetPowerState(const rokae_xmate3_ros2::srv::SetPowerState::Request &req,
                                        rokae_xmate3_ros2::srv::SetPowerState::Response &res) const {
  if (!requireConnected(session_state_, res.success, res.message)) {
    return;
  }
  if (session_state_.powerOn() == req.on) {
    res.success = true;
    res.message = "power state already matches request";
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
  res.message = req.on ? "power on" : "power off";
}

void ControlFacade::handleSetOperateMode(const rokae_xmate3_ros2::srv::SetOperateMode::Request &req,
                                         rokae_xmate3_ros2::srv::SetOperateMode::Response &res) const {
  if (!requireConnected(session_state_, res.success, res.message)) {
    return;
  }
  if (req.mode != kOperateModeManual && req.mode != kOperateModeAutomatic) {
    res.success = false;
    res.message = "operate mode must be manual or automatic";
    return;
  }
  if (session_state_.operateMode().mode == req.mode) {
    res.success = true;
    res.message = "operate mode already matches request";
    return;
  }
  session_state_.setOperateMode(req.mode);
  res.success = true;
  res.message = "operate mode updated";
}

void ControlFacade::handleClearServoAlarm(const rokae_xmate3_ros2::srv::ClearServoAlarm::Request &req,
                                          rokae_xmate3_ros2::srv::ClearServoAlarm::Response &res) const {
  (void)req;
  if (!requireConnected(session_state_, res.success, res.message)) {
    return;
  }
  res.success = true;
  res.message = "servo alarm cleared";
}

void ControlFacade::handleEnableCollisionDetection(
    const rokae_xmate3_ros2::srv::EnableCollisionDetection::Request &req,
    rokae_xmate3_ros2::srv::EnableCollisionDetection::Response &res) const {
  if (!requireConnected(session_state_, res.success, res.message)) {
    return;
  }
  constexpr double kMinSensitivity = 0.01;
  constexpr double kMaxSensitivity = 2.0;
  std::array<double, 6> sensitivity{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  if (!req.sensitivity.empty() && req.sensitivity.size() != sensitivity.size()) {
    res.success = false;
    res.message = "collision sensitivity requires exactly 6 joint values";
    return;
  }
  for (std::size_t i = 0; i < req.sensitivity.size() && i < sensitivity.size(); ++i) {
    if (req.sensitivity[i] < kMinSensitivity || req.sensitivity[i] > kMaxSensitivity) {
      res.success = false;
      res.message = "collision sensitivity must stay within [0.01, 2.0]";
      return;
    }
    sensitivity[i] = req.sensitivity[i];
  }
  if (req.behaviour > 3U) {
    res.success = false;
    res.message = "collision behaviour must be one of stop0/stop1/stop2/suppleStop";
    return;
  }
  if (req.behaviour == 3U) {
    if (req.fallback < 0.0 || req.fallback > 1.0) {
      res.success = false;
      res.message = "suppleStop fallback must stay within [0.0, 1.0]";
      return;
    }
  } else if (req.fallback < 0.0) {
    res.success = false;
    res.message = "collision fallback distance must be non-negative";
    return;
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
  if (!requireConnected(session_state_, res.success, res.message)) {
    return;
  }
  session_state_.setCollisionDetectionEnabled(false);
  res.success = true;
  res.message = "collision detection disabled";
}

void ControlFacade::handleSetSoftLimit(const rokae_xmate3_ros2::srv::SetSoftLimit::Request &req,
                                       rokae_xmate3_ros2::srv::SetSoftLimit::Response &res) const {
  auto soft_limit = motion_options_state_.softLimit();
  if (req.enable) {
    if (!session_state_.connected()) {
      res.success = false;
      res.message = "soft limit can only be enabled while connected";
      return;
    }
    if (session_state_.operateMode().mode != static_cast<uint8_t>(rokae::OperateMode::manual)) {
      res.success = false;
      res.message = "soft limit requires manual operate mode";
      return;
    }
    if (session_state_.powerOn()) {
      res.success = false;
      res.message = "soft limit requires robot power off";
      return;
    }
    if (req.limits.size() < 12) {
      res.success = false;
      res.message = "soft limit requires 6 lower/upper joint pairs";
      return;
    }

    for (std::size_t axis = 0; axis < 6; ++axis) {
      const double lower = req.limits[axis * 2];
      const double upper = req.limits[axis * 2 + 1];
      if (!(lower < upper)) {
        res.success = false;
        res.message = "soft limit lower bound must be smaller than upper bound";
        return;
      }
      if (lower < rokae_xmate3_ros2::spec::xmate3::kJointLimitMin[axis] ||
          upper > rokae_xmate3_ros2::spec::xmate3::kJointLimitMax[axis]) {
        std::ostringstream oss;
        oss << "soft limit joint " << axis << " exceeds mechanical joint limits";
        res.success = false;
        res.message = oss.str();
        return;
      }
      soft_limit.limits[axis][0] = lower;
      soft_limit.limits[axis][1] = upper;
    }

    if (backend_ == nullptr) {
      res.success = false;
      res.message = "soft limit validation requires a backend snapshot";
      return;
    }
    const auto snapshot = backend_->readSnapshot();
    for (std::size_t axis = 0; axis < snapshot.joint_position.size(); ++axis) {
      if (snapshot.joint_position[axis] < soft_limit.limits[axis][0] ||
          snapshot.joint_position[axis] > soft_limit.limits[axis][1]) {
        std::ostringstream oss;
        oss << "current joint " << axis << " is outside the requested soft limit range";
        res.success = false;
        res.message = oss.str();
        return;
      }
    }
  }

  motion_options_state_.setSoftLimit(req.enable, soft_limit.limits);
  res.success = true;
  res.message = req.enable ? "soft limit enabled" : "soft limit disabled";
}

void ControlFacade::handleSetMotionControlMode(
    const rokae_xmate3_ros2::srv::SetMotionControlMode::Request &req,
    rokae_xmate3_ros2::srv::SetMotionControlMode::Response &res) const {
  if (!requireConnected(session_state_, res.success, res.message)) {
    return;
  }
  if (req.mode < kMotionModeMin || req.mode > kMotionModeMax) {
    res.success = false;
    res.message = "motion control mode must be one of Idle/NrtCommand/RtCommand-compatible values";
    return;
  }
  if (session_state_.motionMode() == req.mode) {
    res.success = true;
    res.message = "motion control mode already matches request";
    return;
  }
  stopRuntime("motion control mode changed");
  if (motion_runtime_ != nullptr) {
    motion_runtime_->clearForModeChange("motion control mode changed");
  }
  clearBackendControl();
  session_state_.setMotionMode(req.mode);
  res.success = true;
  res.message = "motion control mode updated";
}

void ControlFacade::handleMoveReset(const rokae_xmate3_ros2::srv::MoveReset::Request &req,
                                    rokae_xmate3_ros2::srv::MoveReset::Response &res) const {
  (void)req;
  if (!requireConnected(session_state_, res.success, res.message)) {
    return;
  }
  if (request_coordinator_ != nullptr) {
    request_coordinator_->reset();
  } else if (motion_runtime_ != nullptr) {
    motion_runtime_->reset();
  }
  clearBackendControl();
  res.success = true;
  res.message = "move queue reset";
}

void ControlFacade::handleMoveStart(const rokae_xmate3_ros2::srv::MoveStart::Request &req,
                                    rokae_xmate3_ros2::srv::MoveStart::Response &res) const {
  (void)req;
  if (!requireConnected(session_state_, res.success, res.message)) {
    return;
  }
  if (session_state_.motionMode() != kMotionModeNrt) {
    res.success = false;
    res.message = "moveStart requires NrtCommand-compatible motion mode";
    return;
  }
  if (!session_state_.powerOn()) {
    res.success = false;
    res.message = "Robot not powered on";
    return;
  }
  if (session_state_.dragMode()) {
    res.success = false;
    res.message = "moveStart is not allowed while drag mode is enabled";
    return;
  }
  if (motion_runtime_ != nullptr) {
    const auto view = motion_runtime_->view();
    if (!view.queue_initialized) {
      res.success = false;
      res.message = "moveStart requires moveReset to initialize the NRT queue";
      return;
    }
    if (!view.has_request && !view.queue_has_pending_commands) {
      res.success = false;
      res.message = "moveStart rejected because no queued NRT request exists";
      return;
    }
  }
  res.success = true;
  res.message = "move start accepted";
}

void ControlFacade::handleStop(const rokae_xmate3_ros2::srv::Stop::Request &req,
                               rokae_xmate3_ros2::srv::Stop::Response &res) const {
  (void)req;
  if (!requireConnected(session_state_, res.success, res.message)) {
    return;
  }
  stopRuntime("stop service called");
  clearBackendControl();
  res.success = true;
  res.message = "stop accepted";
}

void ControlFacade::handleSetDefaultSpeed(const rokae_xmate3_ros2::srv::SetDefaultSpeed::Request &req,
                                          rokae_xmate3_ros2::srv::SetDefaultSpeed::Response &res) const {
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
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
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
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
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
  motion_options_state_.setDefaultConfOpt(req.forced);
  res.success = true;
}

void ControlFacade::handleAdjustSpeedOnline(
    const rokae_xmate3_ros2::srv::AdjustSpeedOnline::Request &req,
    rokae_xmate3_ros2::srv::AdjustSpeedOnline::Response &res) const {
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
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
  if (!requireConnected(session_state_, res.success, res.error_code, res.error_msg)) {
    return;
  }
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
  if (motion_runtime_ != nullptr) {
    motion_runtime_->clearForModeChange("rt control mode changed");
  }
  clearBackendControl();
  session_state_.setRtControlMode(req.mode);
  session_state_.setMotionMode(kSessionMotionModeRt);
  syncBrakeState(false);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void ControlFacade::handleSetSimulationMode(
    const rokae_xmate3_ros2::srv::SetSimulationMode::Request &req,
    rokae_xmate3_ros2::srv::SetSimulationMode::Response &res) const {
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
  session_state_.setSimulationMode(req.state);
  res.success = true;
}

void ControlFacade::handleSetToolset(const rokae_xmate3_ros2::srv::SetToolset::Request &req,
                                     rokae_xmate3_ros2::srv::SetToolset::Response &res) const {
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
  tooling_state_.setToolset(req.tool_name, req.wobj_name, req.tool_pose, req.wobj_pose);
  res.success = true;
  res.message = "toolset updated";
}

void ControlFacade::handleSetToolsetByName(const rokae_xmate3_ros2::srv::SetToolsetByName::Request &req,
                                           rokae_xmate3_ros2::srv::SetToolsetByName::Response &res) const {
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
  res.success = tooling_state_.setToolsetByName(req.tool_name, req.wobj_name);
  if (!res.success) {
    res.message = "unknown tool_name or wobj_name";
    return;
  }
  res.message = "toolset selection updated";
}

void ControlFacade::handleEnableDrag(const rokae_xmate3_ros2::srv::EnableDrag::Request &req,
                                     rokae_xmate3_ros2::srv::EnableDrag::Response &res) const {
  (void)req;
  if (!requireConnected(session_state_, res.success, res.message)) {
    return;
  }
  if (session_state_.operateMode().mode != kOperateModeManual) {
    res.success = false;
    res.message = "drag mode requires manual operate mode";
    return;
  }
  if (session_state_.powerOn()) {
    res.success = false;
    res.message = "drag mode requires robot power off";
    return;
  }
  if (request_coordinator_ != nullptr && !request_coordinator_->canAcceptRequest()) {
    res.success = false;
    res.message = "Runtime is busy";
    return;
  }
  session_state_.setDragMode(true);
  stopRuntime("drag mode enabled");
  syncBrakeState(false);
  clearBackendControl();
  res.success = true;
  res.message = "drag mode enabled";
}

void ControlFacade::handleDisableDrag(const rokae_xmate3_ros2::srv::DisableDrag::Request &req,
                                      rokae_xmate3_ros2::srv::DisableDrag::Response &res) const {
  (void)req;
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
  session_state_.setDragMode(false);
  clearBackendControl();
  res.success = true;
  res.message = "drag mode disabled";
}

void ControlFacade::handleSetAvoidSingularity(
    const rokae_xmate3_ros2::srv::SetAvoidSingularity::Request &req,
    rokae_xmate3_ros2::srv::SetAvoidSingularity::Response &res) const {
  (void)req;
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
  res.success = false;
  res.message = "avoid singularity is not supported on the xMate6 compatibility lane";
}


}  // namespace rokae_xmate3_ros2::runtime
