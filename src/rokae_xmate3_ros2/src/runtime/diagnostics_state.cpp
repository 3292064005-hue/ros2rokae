#include "runtime/diagnostics_state.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr std::size_t kRecentRuntimeEventLimit = 8;

std::string extract_candidate_name(const std::string &summary) {
  const std::string needle = "candidate=";
  const auto pos = summary.find(needle);
  if (pos == std::string::npos) {
    return {};
  }
  const auto begin = pos + needle.size();
  const auto end = summary.find_first_of("; ]", begin);
  return summary.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
}

std::string refresh_event_bus_summary(std::uint32_t runtime_event_count,
                                      std::uint32_t planning_rejection_count,
                                      std::uint32_t watchdog_trigger_count) {
  return "events=" + std::to_string(runtime_event_count) + "; planning_rejections=" +
         std::to_string(planning_rejection_count) + "; watchdog_triggers=" +
         std::to_string(watchdog_trigger_count);
}
}

namespace rokae_xmate3_ros2::runtime {

void RuntimeDiagnosticsState::configure(const std::string &backend_mode,
                                        const std::vector<std::string> &capability_flags,
                                        const std::string &active_profile) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.backend_mode = backend_mode.empty() ? std::string{"unknown"} : backend_mode;
  snapshot_.capability_flags = capability_flags;
  snapshot_.active_profile = active_profile.empty() ? std::string{"unknown"} : active_profile;
  snapshot_.last_runtime_event = "reset";
  snapshot_.recent_runtime_events.clear();
  snapshot_.runtime_event_count = 0;
  snapshot_.planning_rejection_count = 0;
  snapshot_.watchdog_trigger_count = 0;
  snapshot_.event_bus_summary = refresh_event_bus_summary(snapshot_.runtime_event_count,
                                                          snapshot_.planning_rejection_count,
                                                          snapshot_.watchdog_trigger_count);
  snapshot_.rt_prearm_status =
      snapshot_.active_profile.find("rt") != std::string::npos ? std::string{"pending"} : std::string{"not_applicable"};
}

void RuntimeDiagnosticsState::updateRuntimeStatus(const RuntimeStatus &status) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.control_owner = to_string(status.control_owner);
  snapshot_.runtime_phase = to_string(status.runtime_phase);
  snapshot_.active_execution_backend = to_string(status.execution_backend);
  const std::string event_summary = status.last_event + (status.message.empty() ? std::string{} : (":" + status.message));
  const bool first_observed_event = snapshot_.recent_runtime_events.empty();
  if (first_observed_event || snapshot_.last_runtime_event != status.last_event) {
    snapshot_.last_runtime_event = status.last_event;
    snapshot_.runtime_event_count += 1;
    if (status.last_event == "planning_rejected") {
      snapshot_.planning_rejection_count += 1;
    } else if (status.last_event == "watchdog_triggered") {
      snapshot_.watchdog_trigger_count += 1;
    }
    snapshot_.event_bus_summary = refresh_event_bus_summary(snapshot_.runtime_event_count,
                                                            snapshot_.planning_rejection_count,
                                                            snapshot_.watchdog_trigger_count);
    snapshot_.recent_runtime_events.push_back(event_summary);
    if (snapshot_.recent_runtime_events.size() > kRecentRuntimeEventLimit) {
      snapshot_.recent_runtime_events.erase(snapshot_.recent_runtime_events.begin(),
                                            snapshot_.recent_runtime_events.begin() +
                                                static_cast<long>(snapshot_.recent_runtime_events.size() - kRecentRuntimeEventLimit));
    }
  } else if (!snapshot_.recent_runtime_events.empty() && snapshot_.recent_runtime_events.back() != event_summary) {
    snapshot_.recent_runtime_events.back() = event_summary;
  } else if (snapshot_.recent_runtime_events.empty()) {
    snapshot_.recent_runtime_events.push_back(event_summary);
  }
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
  if (status.last_event == "plan_queued" || status.last_event == "planning_rejected") {
    snapshot_.last_plan_summary = status.message;
    const auto selected_candidate = extract_candidate_name(status.message);
    if (!selected_candidate.empty()) {
      snapshot_.last_selected_candidate = selected_candidate;
    }
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

void RuntimeDiagnosticsState::notePlanSummary(const std::string &summary, const std::string &selected_candidate) {
  if (summary.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.last_plan_summary = summary;
  if (!selected_candidate.empty()) {
    snapshot_.last_selected_candidate = selected_candidate;
  }
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
                                                   double max_gap_ms,
                                                   double avg_gap_ms,
                                                   std::uint32_t consecutive_late_cycles,
                                                   std::uint32_t stale_state_count,
                                                   std::uint32_t command_starvation_windows,
                                                   const std::string &last_trigger_reason) {
  if (summary.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.rt_watchdog_summary = summary;
  snapshot_.rt_late_cycle_count = late_cycle_count;
  snapshot_.rt_max_gap_ms = std::isfinite(max_gap_ms) ? std::max(0.0, max_gap_ms) : 0.0;
  snapshot_.rt_avg_gap_ms = std::isfinite(avg_gap_ms) ? std::max(0.0, avg_gap_ms) : 0.0;
  snapshot_.rt_consecutive_late_cycles = consecutive_late_cycles;
  snapshot_.rt_stale_state_count = stale_state_count;
  snapshot_.rt_command_starvation_windows = command_starvation_windows;
  snapshot_.rt_last_trigger_reason = last_trigger_reason.empty() ? std::string{"nominal"} : last_trigger_reason;
}

void RuntimeDiagnosticsState::setProfileCapabilitySummary(const std::string &summary) {
  if (summary.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.profile_capability_summary = summary;
}

void RuntimeDiagnosticsState::setPlanningCapabilitySummary(const std::string &summary) {
  if (summary.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.planning_capability_summary = summary;
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
