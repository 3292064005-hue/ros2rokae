#ifndef ROKAE_XMATE3_ROS2_RUNTIME_SHUTDOWN_COORDINATOR_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_SHUTDOWN_COORDINATOR_HPP

#include <string>

#include "runtime/runtime_types.hpp"

namespace rokae_xmate3_ros2::runtime {

struct ShutdownContractView {
  bool accepted = false;
  ControlOwner owner = ControlOwner::none;
  bool owner_none = true;
  bool runtime_idle = true;
  bool backend_quiescent = true;
  bool safe_to_delete = false;
  bool safe_to_stop_world = false;
  RuntimePhase runtime_phase = RuntimePhase::idle;
  ShutdownPhase shutdown_phase = ShutdownPhase::running;
  std::size_t active_request_count = 0;
  std::size_t active_goal_count = 0;
  std::string message{"idle"};
};

class ShutdownCoordinator {
 public:
  struct Inputs {
    RuntimeContractView runtime;
    bool update_loop_detached = false;
    bool backend_quiescent = true;
    bool faulted = false;
  };

  void begin(const std::string &reason) {
    requested_ = true;
    if (!reason.empty()) {
      last_reason_ = reason;
    }
  }

  void reset() {
    requested_ = false;
    phase_ = ShutdownPhase::running;
    last_reason_ = "idle";
    detached_seen_ = false;
    delete_ready_seen_ = false;
    stop_world_ready_seen_ = false;
  }

  [[nodiscard]] bool requested() const noexcept { return requested_; }
  [[nodiscard]] ShutdownPhase phase() const noexcept { return phase_; }
  void finalize(const std::string &reason = "shutdown finished") {
    phase_ = ShutdownPhase::finished;
    last_reason_ = reason;
  }

  [[nodiscard]] ShutdownContractView observe(const Inputs &inputs) {
    ShutdownContractView view;
    view.accepted = requested_;
    view.owner = inputs.runtime.owner;
    view.owner_none = inputs.runtime.owner == ControlOwner::none;
    view.active_request_count = inputs.runtime.active_request_count;
    view.active_goal_count = inputs.runtime.active_goal_count;
    view.runtime_idle =
        inputs.runtime.active_request_count == 0 &&
        inputs.runtime.runtime_phase != RuntimePhase::planning &&
        inputs.runtime.runtime_phase != RuntimePhase::executing;
    view.backend_quiescent = inputs.backend_quiescent && inputs.runtime.active_goal_count == 0;
    view.runtime_phase = inputs.runtime.runtime_phase;

    if (!requested_) {
      phase_ = ShutdownPhase::running;
      last_reason_ = "shutdown not requested";
    } else if (inputs.faulted || inputs.runtime.runtime_phase == RuntimePhase::faulted) {
      phase_ = ShutdownPhase::faulted;
      last_reason_ = "shutdown faulted";
    } else {
      switch (phase_) {
        case ShutdownPhase::running:
          phase_ = ShutdownPhase::draining;
          last_reason_ = "shutdown draining";
          break;
        case ShutdownPhase::draining:
          last_reason_ = "shutdown draining";
          if (inputs.update_loop_detached) {
            detached_seen_ = true;
            phase_ = ShutdownPhase::backend_detached;
            last_reason_ = "backend detached";
          }
          break;
        case ShutdownPhase::backend_detached:
          last_reason_ = "backend detached";
          if (view.owner_none && view.runtime_idle && view.backend_quiescent) {
            delete_ready_seen_ = true;
            phase_ = ShutdownPhase::safe_to_delete;
            last_reason_ = "safe to delete";
          }
          break;
        case ShutdownPhase::safe_to_delete:
          last_reason_ = "safe to delete";
          if (view.owner_none && view.runtime_idle && view.backend_quiescent) {
            stop_world_ready_seen_ = true;
            phase_ = ShutdownPhase::safe_to_stop_world;
            last_reason_ = "safe to stop world";
          }
          break;
        case ShutdownPhase::safe_to_stop_world:
          last_reason_ = "safe to stop world";
          break;
        case ShutdownPhase::finished:
          last_reason_ = "shutdown finished";
          break;
        case ShutdownPhase::faulted:
          last_reason_ = "shutdown faulted";
          break;
      }
    }

    view.safe_to_delete = inputs.runtime.safe_to_delete || delete_ready_seen_ ||
                          phase_ == ShutdownPhase::safe_to_delete ||
                          phase_ == ShutdownPhase::safe_to_stop_world ||
                          phase_ == ShutdownPhase::finished;
    view.safe_to_stop_world =
        inputs.runtime.safe_to_stop_world || stop_world_ready_seen_ ||
        phase_ == ShutdownPhase::safe_to_stop_world || phase_ == ShutdownPhase::finished;
    view.shutdown_phase = phase_;
    view.message = last_reason_;
    return view;
  }

 private:
  bool requested_ = false;
  ShutdownPhase phase_ = ShutdownPhase::running;
  std::string last_reason_{"idle"};
  bool detached_seen_ = false;
  bool delete_ready_seen_ = false;
  bool stop_world_ready_seen_ = false;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif  // ROKAE_XMATE3_ROS2_RUNTIME_SHUTDOWN_COORDINATOR_HPP
