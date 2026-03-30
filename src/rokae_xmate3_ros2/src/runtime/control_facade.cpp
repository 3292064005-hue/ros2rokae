#include "runtime/service_facade.hpp"

#include <algorithm>

namespace rokae_xmate3_ros2::runtime {

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
  session_state_.setMotionMode(static_cast<int>(rokae::MotionControlMode::RtCommand));
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


}  // namespace rokae_xmate3_ros2::runtime
