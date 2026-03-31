#include "runtime/runtime_state_machine.hpp"

namespace rokae_xmate3_ros2::runtime {

const char *to_string(RuntimeEventType type) noexcept {
  switch (type) {
    case RuntimeEventType::planning_requested:
      return "planning_requested";
    case RuntimeEventType::planning_rejected:
      return "planning_rejected";
    case RuntimeEventType::plan_queued:
      return "plan_queued";
    case RuntimeEventType::execution_started:
      return "execution_started";
    case RuntimeEventType::progress_updated:
      return "progress_updated";
    case RuntimeEventType::trajectory_retimed:
      return "trajectory_retimed";
    case RuntimeEventType::watchdog_triggered:
      return "watchdog_triggered";
    case RuntimeEventType::completed:
      return "completed";
    case RuntimeEventType::completed_relaxed:
      return "completed_relaxed";
    case RuntimeEventType::failed:
      return "failed";
    case RuntimeEventType::stopped:
      return "stopped";
    case RuntimeEventType::owner_changed:
      return "owner_changed";
    case RuntimeEventType::phase_override:
      return "phase_override";
    case RuntimeEventType::reset:
    default:
      return "reset";
  }
}

void RuntimeStateMachine::applyTerminal(RuntimeStatus &status,
                                        RuntimePhase &runtime_phase,
                                        const RuntimeEvent &event,
                                        ExecutionState terminal_state,
                                        RuntimePhase terminal_phase) {
  status.last_event = to_string(event.type);
  status.state = terminal_state;
  status.terminal_success = event.terminal_success;
  if (event.execution_backend != ExecutionBackend::none) {
    status.execution_backend = event.execution_backend;
  }
  status.completed_segments = event.completed_segments;
  status.current_segment_index = event.current_segment_index;
  if (event.total_segments != 0 || status.total_segments == 0) {
    status.total_segments = event.total_segments;
  }
  if (!event.request_id.empty()) {
    status.request_id = event.request_id;
  }
  if (!event.message.empty()) {
    status.message = event.message;
  }
  runtime_phase = terminal_phase;
  status.runtime_phase = runtime_phase;
}

void RuntimeStateMachine::apply(RuntimeStatus &status,
                                RuntimePhase &runtime_phase,
                                const RuntimeEvent &event) const {
  switch (event.type) {
    case RuntimeEventType::reset:
      status = RuntimeStatus{};
      status.last_event = to_string(event.type);
      runtime_phase = RuntimePhase::idle;
      status.runtime_phase = runtime_phase;
      return;
    case RuntimeEventType::planning_requested:
      status = RuntimeStatus{};
      status.last_event = to_string(event.type);
      status.request_id = event.request_id;
      status.state = ExecutionState::planning;
      status.completed_segments = 0;
      status.current_segment_index = 0;
      status.message = event.message.empty() ? std::string{"planning"} : event.message;
      status.total_segments = event.total_segments;
      status.execution_backend = ExecutionBackend::none;
      runtime_phase = RuntimePhase::planning;
      status.runtime_phase = runtime_phase;
      return;
    case RuntimeEventType::planning_rejected:
      applyTerminal(status, runtime_phase, event, ExecutionState::failed, RuntimePhase::faulted);
      return;
    case RuntimeEventType::plan_queued:
      status.last_event = to_string(event.type);
      if (!event.request_id.empty()) {
        status.request_id = event.request_id;
      }
      status.state = ExecutionState::queued;
      status.total_segments = event.total_segments;
      status.completed_segments = event.completed_segments;
      status.current_segment_index = event.current_segment_index;
      status.execution_backend = ExecutionBackend::none;
      status.message = event.message.empty() ? std::string{"queued"} : event.message;
      runtime_phase = RuntimePhase::planning;
      status.runtime_phase = runtime_phase;
      return;
    case RuntimeEventType::execution_started:
      status.last_event = to_string(event.type);
      if (!event.request_id.empty()) {
        status.request_id = event.request_id;
      }
      status.state = ExecutionState::executing;
      status.execution_backend = event.execution_backend;
      status.total_segments = event.total_segments;
      status.completed_segments = event.completed_segments;
      status.current_segment_index = event.current_segment_index;
      status.message = event.message.empty() ? std::string{"executing"} : event.message;
      runtime_phase = RuntimePhase::executing;
      status.runtime_phase = runtime_phase;
      return;
    case RuntimeEventType::progress_updated:
      status.last_event = to_string(event.type);
      if (!event.request_id.empty()) {
        status.request_id = event.request_id;
      }
      if (status.state == ExecutionState::idle || status.state == ExecutionState::queued) {
        status.state = ExecutionState::executing;
      }
      if (event.execution_backend != ExecutionBackend::none) {
        status.execution_backend = event.execution_backend;
      }
      if (event.total_segments != 0 || status.total_segments == 0) {
        status.total_segments = event.total_segments;
      }
      status.completed_segments = event.completed_segments;
      status.current_segment_index = event.current_segment_index;
      if (!event.message.empty()) {
        status.message = event.message;
      }
      runtime_phase = RuntimePhase::executing;
      status.runtime_phase = runtime_phase;
      return;
    case RuntimeEventType::trajectory_retimed:
      status.last_event = to_string(event.type);
      if (!event.request_id.empty()) {
        status.request_id = event.request_id;
      }
      if (!event.message.empty()) {
        status.message = event.message;
      }
      status.runtime_phase = runtime_phase;
      return;
    case RuntimeEventType::watchdog_triggered:
      applyTerminal(status, runtime_phase, event, ExecutionState::failed, RuntimePhase::faulted);
      return;
    case RuntimeEventType::completed:
      applyTerminal(status, runtime_phase, event, ExecutionState::completed, RuntimePhase::idle);
      return;
    case RuntimeEventType::completed_relaxed:
      applyTerminal(status, runtime_phase, event, ExecutionState::completed_relaxed, RuntimePhase::idle);
      return;
    case RuntimeEventType::failed:
      applyTerminal(status, runtime_phase, event, ExecutionState::failed, RuntimePhase::faulted);
      return;
    case RuntimeEventType::stopped:
      applyTerminal(status, runtime_phase, event, ExecutionState::stopped, RuntimePhase::idle);
      return;
    case RuntimeEventType::owner_changed:
      status.last_event = to_string(event.type);
      status.control_owner = event.owner;
      if (!event.message.empty()) {
        status.message = event.message;
      }
      status.runtime_phase = runtime_phase;
      return;
    case RuntimeEventType::phase_override:
      status.last_event = to_string(event.type);
      runtime_phase = event.phase;
      status.runtime_phase = runtime_phase;
      if (!event.message.empty()) {
        status.message = event.message;
      }
      return;
  }
}

}  // namespace rokae_xmate3_ros2::runtime
