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
  snapshot_.rt_prearm_status = snapshot_.active_profile.find("rt") != std::string::npos ? std::string{"pending"} : std::string{"not_applicable"};
}

void RuntimeDiagnosticsState::updateRuntimeStatus(const RuntimeStatus &status) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.control_owner = to_string(status.control_owner);
  snapshot_.runtime_phase = to_string(status.runtime_phase);
  snapshot_.active_execution_backend = to_string(status.execution_backend);
  if (!status.request_id.empty() && !status.terminal()) {
    snapshot_.active_request_count = 1u;
    snapshot_.active_request_id = status.request_id;
  } else {
    snapshot_.active_request_count = 0u;
    snapshot_.active_request_id.clear();
  }
  if (status.execution_backend == ExecutionBackend::jtc && !status.terminal()) {
    snapshot_.active_goal_count = 1u;
  } else {
    snapshot_.active_goal_count = 0u;
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

void RuntimeDiagnosticsState::setRtSubscriptionPlan(const std::string &summary) {
  if (summary.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.rt_subscription_plan = summary;
}

void RuntimeDiagnosticsState::setRtPrearmStatus(const std::string &status) {
  if (status.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.rt_prearm_status = status;
}

void RuntimeDiagnosticsState::setRtWatchdogSummary(const std::string &summary,
                                                   std::uint32_t late_cycle_count,
                                                   double max_gap_ms) {
  if (summary.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.rt_watchdog_summary = summary;
  snapshot_.rt_late_cycle_count = late_cycle_count;
  snapshot_.rt_max_gap_ms = std::isfinite(max_gap_ms) ? std::max(0.0, max_gap_ms) : 0.0;
}


void RuntimeDiagnosticsState::setProfileCapabilitySummary(const std::string &summary) {
  if (summary.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.profile_capability_summary = summary;
}

void RuntimeDiagnosticsState::setRuntimeOptionSummary(const std::string &summary) {
  if (summary.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.runtime_option_summary = summary;
}

void RuntimeDiagnosticsState::setCatalogSizes(std::uint32_t tool_count,
                                              std::uint32_t wobj_count,
                                              std::uint32_t project_count,
                                              std::uint32_t register_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.tool_catalog_size = tool_count;
  snapshot_.wobj_catalog_size = wobj_count;
  snapshot_.project_catalog_size = project_count;
  snapshot_.register_catalog_size = register_count;
}

RuntimeDiagnosticsSnapshot RuntimeDiagnosticsState::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}


}  // namespace rokae_xmate3_ros2::runtime
