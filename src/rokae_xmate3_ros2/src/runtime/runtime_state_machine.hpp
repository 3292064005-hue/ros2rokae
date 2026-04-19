#ifndef ROKAE_XMATE3_ROS2_RUNTIME_STATE_MACHINE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_STATE_MACHINE_HPP

#include <cstddef>
#include <string>

#include "runtime/runtime_types.hpp"

namespace rokae_xmate3_ros2::runtime {

enum class RuntimeEventType {
  reset,
  request_queued,
  planning_requested,
  planning_rejected,
  plan_queued,
  execution_started,
  progress_updated,
  trajectory_retimed,
  paused,
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
  bool has_observed_state = false;
  ExecutionState observed_state = ExecutionState::idle;
  ExecutionBackend execution_backend = ExecutionBackend::none;
  ControlOwner owner = ControlOwner::none;
  RuntimePhase phase = RuntimePhase::idle;
  bool terminal_success = false;
};

/**
 * @brief Single authority for runtime request lifecycle state transitions.
 *
 * The state machine owns the canonical mapping from planner/executor/watchdog events into
 * RuntimeStatus and RuntimePhase. Launch flows, diagnostics, and query services must treat the
 * resulting state as the only truth source for request lifecycle reporting.
 */
class RuntimeStateMachine {
 public:
  /**
   * @brief Apply one lifecycle event to the current runtime status snapshot.
   * @param status In/out canonical runtime status record.
   * @param runtime_phase In/out coarse runtime phase authority.
   * @param event Input event emitted by planner, executor, ownership, or watchdog paths.
   * @note Boundary behavior: terminal events preserve the previously selected execution backend when
   *       the incoming event does not provide one.
   */
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
