#ifndef ROKAE_XMATE3_ROS2_RUNTIME_STATE_MACHINE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_STATE_MACHINE_HPP

#include <cstddef>
#include <string>

#include "runtime/runtime_types.hpp"

namespace rokae_xmate3_ros2::runtime {

enum class RuntimeEventType {
  reset,
  planning_requested,
  planning_rejected,
  plan_queued,
  execution_started,
  progress_updated,
  trajectory_retimed,
  watchdog_triggered,
  completed,
  completed_relaxed,
  failed,
  stopped,
  owner_changed,
  phase_override,
};

[[nodiscard]] const char *to_string(RuntimeEventType type) noexcept;

struct RuntimeEvent {
  RuntimeEventType type = RuntimeEventType::reset;
  std::string request_id;
  std::string message;
  std::size_t total_segments = 0;
  std::size_t completed_segments = 0;
  std::size_t current_segment_index = 0;
  ExecutionBackend execution_backend = ExecutionBackend::none;
  ControlOwner owner = ControlOwner::none;
  RuntimePhase phase = RuntimePhase::idle;
  bool terminal_success = false;
};

class RuntimeStateMachine {
 public:
  void apply(RuntimeStatus &status, RuntimePhase &runtime_phase, const RuntimeEvent &event) const;

 private:
  static void applyTerminal(RuntimeStatus &status,
                            RuntimePhase &runtime_phase,
                            const RuntimeEvent &event,
                            ExecutionState terminal_state,
                            RuntimePhase terminal_phase);
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
