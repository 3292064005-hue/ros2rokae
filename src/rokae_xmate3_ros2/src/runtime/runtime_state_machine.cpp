#include "runtime/runtime_state_machine.hpp"

namespace rokae_xmate3_ros2::runtime {

void RuntimeStateMachine::applyTerminal(RuntimeStatus &status,
                                        RuntimePhase &runtime_phase,
                                        const RuntimeEvent &event,
                                        ExecutionState terminal_state,
                                        RuntimePhase terminal_phase) {
  status.state = terminal_state;
  status.terminal_success = event.terminal_success;
  status.execution_backend = event.execution_backend;
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
      runtime_phase = RuntimePhase::idle;
      status.runtime_phase = runtime_phase;
      return;
    case RuntimeEventType::planning_requested:
      status = RuntimeStatus{};
      status.request_id = event.request_id;
      status.state = ExecutionState::planning;
      status.message = event.message.empty() ? std::string{"planning"} : event.message;
      status.total_segments = event.total_segments;
      status.execution_backend = ExecutionBackend::none;
      runtime_phase = RuntimePhase::planning;
      status.runtime_phase = runtime_phase;
      return;
    case RuntimeEventType::plan_queued:
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
      status.control_owner = event.owner;
      if (!event.message.empty()) {
        status.message = event.message;
      }
      status.runtime_phase = runtime_phase;
      return;
    case RuntimeEventType::phase_override:
      runtime_phase = event.phase;
      status.runtime_phase = runtime_phase;
      if (!event.message.empty()) {
        status.message = event.message;
      }
      return;
  }
}

}  // namespace rokae_xmate3_ros2::runtime
