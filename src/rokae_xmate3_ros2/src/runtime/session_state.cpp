#include "runtime/runtime_state.hpp"

namespace rokae_xmate3_ros2::runtime {

void SessionState::connect(const std::string &remote_ip) {
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = true;
  remote_ip_ = remote_ip;
}

void SessionState::disconnect() {
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = false;
  power_on_ = false;
  drag_mode_ = false;
  rt_control_mode_ = -1;
  motion_mode_ = kSessionMotionModeNrt;
}

bool SessionState::connected() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return connected_;
}

std::string SessionState::remoteIp() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return remote_ip_;
}

void SessionState::setPowerOn(bool on) {
  std::lock_guard<std::mutex> lock(mutex_);
  power_on_ = on;
  if (!power_on_) {
    rt_control_mode_ = -1;
  }
}

bool SessionState::powerOn() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return power_on_;
}

void SessionState::setDragMode(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  drag_mode_ = enabled;
}

bool SessionState::dragMode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return drag_mode_;
}

void SessionState::setSimulationMode(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  simulation_mode_ = enabled;
}

bool SessionState::simulationMode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return simulation_mode_;
}

void SessionState::setCollisionDetectionEnabled(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  collision_detection_enabled_ = enabled;
}

bool SessionState::collisionDetectionEnabled() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return collision_detection_enabled_;
}

void SessionState::setCollisionDetectionConfig(const std::array<double, 6> &sensitivity,
                                               std::uint8_t behaviour,
                                               double fallback) {
  std::lock_guard<std::mutex> lock(mutex_);
  collision_sensitivity_ = sensitivity;
  collision_behaviour_ = behaviour;
  collision_fallback_ = fallback;
}

CollisionDetectionSnapshot SessionState::collisionDetection() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return CollisionDetectionSnapshot{
      collision_detection_enabled_,
      collision_sensitivity_,
      collision_behaviour_,
      collision_fallback_,
  };
}

void SessionState::setMotionMode(int mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  motion_mode_ = mode;
}

int SessionState::motionMode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return motion_mode_;
}

void SessionState::setOperateMode(uint8_t mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  operate_mode_.mode = mode;
}

rokae_xmate3_ros2::msg::OperateMode SessionState::operateMode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return operate_mode_;
}

void SessionState::setRtControlMode(int mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  rt_control_mode_ = mode;
}

int SessionState::rtControlMode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rt_control_mode_;
}

OperationStateContext SessionState::makeOperationStateContext(bool rl_project_running) const {
  std::lock_guard<std::mutex> lock(mutex_);
  OperationStateContext context;
  context.connected = connected_;
  context.power_on = power_on_;
  context.drag_mode = drag_mode_;
  context.rl_project_running = rl_project_running;
  context.rt_control_mode = rt_control_mode_;
  return context;
}


}  // namespace rokae_xmate3_ros2::runtime
