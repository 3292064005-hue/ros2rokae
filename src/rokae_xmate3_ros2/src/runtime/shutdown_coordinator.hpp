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
  struct RuntimeFacts {
    ControlOwner owner = ControlOwner::none;
    RuntimePhase runtime_phase = RuntimePhase::idle;
    std::size_t active_request_count = 0;
    std::size_t active_goal_count = 0;
    std::string message{"idle"};
  };

  struct BackendFacts {
    bool update_loop_detached = false;
    bool backend_quiescent = true;
    bool safe_to_delete_ready = false;
    bool safe_to_stop_world_ready = false;
    std::string message{"backend ready"};
  };

  struct Inputs {
    RuntimeFacts runtime;
    BackendFacts backend;
    bool faulted = false;
  };

  void begin(const std::string &reason) {
    if (!requested_ || phase_ == ShutdownPhase::finished || phase_ == ShutdownPhase::faulted) {
      phase_ = ShutdownPhase::running;
      detached_seen_ = false;
      delete_ready_seen_ = false;
      stop_world_ready_seen_ = false;
    }
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
    if (!requested_) {
      phase_ = ShutdownPhase::running;
      last_reason_ = "shutdown not requested";
    } else if (inputs.faulted || inputs.runtime.runtime_phase == RuntimePhase::faulted) {
      phase_ = ShutdownPhase::faulted;
      last_reason_ = "shutdown faulted";
    } else {
      advancePhase(inputs);
    }

    return contractView(inputs);
  }

 private:
  [[nodiscard]] static bool runtimeIdle(const RuntimeFacts &runtime) noexcept {
    return runtime.active_request_count == 0 &&
           runtime.runtime_phase != RuntimePhase::planning &&
           runtime.runtime_phase != RuntimePhase::executing;
  }

  [[nodiscard]] static bool ownerNone(const RuntimeFacts &runtime) noexcept {
    return runtime.owner == ControlOwner::none;
  }

  [[nodiscard]] static bool backendQuiescent(const Inputs &inputs) noexcept {
    return inputs.backend.backend_quiescent && inputs.runtime.active_goal_count == 0;
  }

  [[nodiscard]] static bool safeToDeleteReady(const Inputs &inputs) noexcept {
    return inputs.backend.safe_to_delete_ready ||
           (inputs.backend.update_loop_detached && backendQuiescent(inputs));
  }

  [[nodiscard]] static bool safeToStopWorldReady(const Inputs &inputs) noexcept {
    return inputs.backend.safe_to_stop_world_ready || safeToDeleteReady(inputs);
  }

  void advancePhase(const Inputs &inputs) {
    switch (phase_) {
      case ShutdownPhase::running:
        phase_ = ShutdownPhase::draining;
        last_reason_ = "shutdown draining";
        break;
      case ShutdownPhase::draining:
        last_reason_ = "shutdown draining";
        if (inputs.backend.update_loop_detached) {
          detached_seen_ = true;
          phase_ = ShutdownPhase::backend_detached;
          last_reason_ = "backend detached";
        }
        break;
      case ShutdownPhase::backend_detached:
        last_reason_ = "backend detached";
        if (ownerNone(inputs.runtime) && runtimeIdle(inputs.runtime) &&
            backendQuiescent(inputs) && safeToDeleteReady(inputs)) {
          delete_ready_seen_ = true;
          phase_ = ShutdownPhase::safe_to_delete;
          last_reason_ = "safe to delete";
        }
        break;
      case ShutdownPhase::safe_to_delete:
        last_reason_ = "safe to delete";
        if (ownerNone(inputs.runtime) && runtimeIdle(inputs.runtime) &&
            backendQuiescent(inputs) && safeToStopWorldReady(inputs)) {
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

  [[nodiscard]] ShutdownContractView contractView(const Inputs &inputs) const {
    ShutdownContractView view;
    view.accepted = requested_;
    view.owner = inputs.runtime.owner;
    view.owner_none = ownerNone(inputs.runtime);
    view.runtime_idle = runtimeIdle(inputs.runtime);
    view.backend_quiescent = backendQuiescent(inputs);
    view.safe_to_delete =
        delete_ready_seen_ || phase_ == ShutdownPhase::safe_to_delete ||
        phase_ == ShutdownPhase::safe_to_stop_world || phase_ == ShutdownPhase::finished;
    view.safe_to_stop_world =
        stop_world_ready_seen_ ||
        phase_ == ShutdownPhase::safe_to_stop_world || phase_ == ShutdownPhase::finished;
    view.runtime_phase = inputs.runtime.runtime_phase;
    view.shutdown_phase = phase_;
    view.active_request_count = inputs.runtime.active_request_count;
    view.active_goal_count = inputs.runtime.active_goal_count;
    view.message = !last_reason_.empty() ? last_reason_ :
                  (!inputs.backend.message.empty() ? inputs.backend.message : inputs.runtime.message);
    return view;
  }

  bool requested_ = false;
  ShutdownPhase phase_ = ShutdownPhase::running;
  std::string last_reason_{"idle"};
  bool detached_seen_ = false;
  bool delete_ready_seen_ = false;
  bool stop_world_ready_seen_ = false;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif  // ROKAE_XMATE3_ROS2_RUNTIME_SHUTDOWN_COORDINATOR_HPP
