#include "runtime/runtime_state.hpp"

#include <algorithm>
#include <cmath>

namespace rokae_xmate3_ros2::runtime {

void RuntimeDiagnosticsState::configure(const std::string &backend_mode,
                                        const std::vector<std::string> &capability_flags,
                                        const std::string &active_profile) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.backend_mode = backend_mode.empty() ? std::string{"unknown"} : backend_mode;
  snapshot_.capability_flags = capability_flags;
  snapshot_.active_profile = active_profile.empty() ? std::string{"unknown"} : active_profile;
}

void RuntimeDiagnosticsState::updateRuntimeStatus(const RuntimeStatus &status) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.control_owner = to_string(status.control_owner);
  snapshot_.runtime_phase = to_string(status.runtime_phase);
  if (!status.request_id.empty() && !status.terminal()) {
    snapshot_.active_request_count = std::max<std::uint32_t>(snapshot_.active_request_count, 1u);
  } else if (snapshot_.shutdown_phase == "running") {
    snapshot_.active_request_count = 0;
  }
  if (status.execution_backend == ExecutionBackend::jtc && !status.terminal()) {
    snapshot_.active_goal_count = std::max<std::uint32_t>(snapshot_.active_goal_count, 1u);
  } else if (snapshot_.shutdown_phase == "running") {
    snapshot_.active_goal_count = 0;
  }
  if (status.state == ExecutionState::failed && !status.message.empty()) {
    snapshot_.last_plan_failure = status.message;
  }
  if (status.message.find("retimer[") != std::string::npos) {
    snapshot_.last_retimer_note = status.message;
  }
}

void RuntimeDiagnosticsState::updateShutdownContract(const RuntimeContractView &view) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.shutdown_phase = to_string(view.shutdown_phase);
  snapshot_.active_request_count = static_cast<std::uint32_t>(view.active_request_count);
  snapshot_.active_goal_count = static_cast<std::uint32_t>(view.active_goal_count);
}

void RuntimeDiagnosticsState::notePlanFailure(const std::string &message) {
  if (message.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.last_plan_failure = message;
}

void RuntimeDiagnosticsState::noteRetimerNote(const std::string &message) {
  if (message.empty() || message.find("retimer[") == std::string::npos) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.last_retimer_note = message;
}

void RuntimeDiagnosticsState::setLastServoDt(double dt) {
  if (!std::isfinite(dt)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.last_servo_dt = dt;
}

void RuntimeDiagnosticsState::setSessionModes(int motion_mode, int rt_mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.motion_mode = motion_mode;
  snapshot_.rt_mode = rt_mode;
}

void RuntimeDiagnosticsState::setActiveProfile(const std::string &active_profile) {
  if (active_profile.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.active_profile = active_profile;
}

void RuntimeDiagnosticsState::setLoopMetrics(double loop_hz, double state_stream_hz, double command_latency_ms) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.loop_hz = std::isfinite(loop_hz) ? std::max(0.0, loop_hz) : 0.0;
  snapshot_.state_stream_hz = std::isfinite(state_stream_hz) ? std::max(0.0, state_stream_hz) : 0.0;
  snapshot_.command_latency_ms = std::isfinite(command_latency_ms) ? std::max(0.0, command_latency_ms) : 0.0;
}

RuntimeDiagnosticsSnapshot RuntimeDiagnosticsState::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}


}  // namespace rokae_xmate3_ros2::runtime
