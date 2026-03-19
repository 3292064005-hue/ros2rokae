#include "runtime/operation_state_adapter.hpp"

#include <sstream>

#include "rokae_xmate3_ros2/msg/operation_state.hpp"

namespace rokae_xmate3_ros2::runtime {

uint8_t resolve_operation_state(const RuntimeView &view,
                                const OperationStateContext &context) noexcept {
  if (context.drag_mode) {
    return rokae_xmate3_ros2::msg::OperationState::DRAG;
  }
  if (!context.connected || !context.power_on) {
    return rokae_xmate3_ros2::msg::OperationState::IDLE;
  }
  if (context.rl_project_running) {
    return rokae_xmate3_ros2::msg::OperationState::RL_PROGRAM;
  }
  if (view.active_motion) {
    return rokae_xmate3_ros2::msg::OperationState::MOVING;
  }
  if (context.rt_control_mode >= 0) {
    return rokae_xmate3_ros2::msg::OperationState::RT_CONTROLLING;
  }
  return rokae_xmate3_ros2::msg::OperationState::IDLE;
}

RuntimeLogEvent build_runtime_log_event(const RuntimeStatus &status,
                                        std::uint64_t last_revision) {
  RuntimeLogEvent event;
  if (status.request_id.empty() || status.revision == 0 || status.revision == last_revision) {
    return event;
  }

  std::ostringstream stream;
  const auto state_text = to_string(status.state);
  const auto &message_text = status.message.empty() ? std::string(state_text) : status.message;
  stream << "[runtime] request=" << status.request_id
         << " state=" << state_text
         << " segment=" << status.current_segment_index << "/" << status.total_segments
         << " completed=" << status.completed_segments
         << " message=" << message_text;

  event.should_log = true;
  event.warning = status.state == ExecutionState::failed || status.state == ExecutionState::stopped;
  event.revision = status.revision;
  event.text = stream.str();
  return event;
}

}  // namespace rokae_xmate3_ros2::runtime
