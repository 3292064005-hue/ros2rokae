#ifndef ROKAE_XMATE3_ROS2_RUNTIME_SHUTDOWN_COORDINATOR_PUBLIC_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_SHUTDOWN_COORDINATOR_PUBLIC_HPP

#include <string>

#include "rokae_xmate3_ros2/runtime/runtime_contract.hpp"

namespace rokae_xmate3_ros2::runtime {

using ShutdownContractView = RuntimeContractView;

struct RuntimeContractFacts {
  ControlOwner owner = ControlOwner::none;
  RuntimePhase runtime_phase = RuntimePhase::idle;
  std::size_t active_request_count = 0;
  std::size_t active_goal_count = 0;
  bool backend_quiescent = false;
  bool plugin_detached = false;
  bool faulted = false;
  std::string message{"idle"};
};

class ShutdownCoordinator {
 public:
  void beginShutdown(const std::string &reason = "shutdown requested") {
    if (!requested_ || phase_ == ShutdownPhase::finished || phase_ == ShutdownPhase::faulted) {
      phase_ = ShutdownPhase::running;
    }
    requested_ = true;
    last_message_ = reason.empty() ? std::string{"shutdown requested"} : reason;
  }

  void clearFault() {
    requested_ = false;
    phase_ = ShutdownPhase::running;
    facts_ = RuntimeContractFacts{};
    last_message_ = "shutdown cleared";
  }

  [[nodiscard]] bool requested() const noexcept { return requested_; }
  [[nodiscard]] ShutdownPhase shutdownPhase() const noexcept { return phase_; }

  void finalize(const std::string &reason = "shutdown finished") {
    phase_ = ShutdownPhase::finished;
    last_message_ = reason.empty() ? std::string{"shutdown finished"} : reason;
  }

  void updateFacts(const RuntimeContractFacts &facts) {
    ingestFacts(facts);
    advancePhase();
  }

  [[nodiscard]] RuntimeContractView currentView() const {
    return emitView();
  }

 private:
  void ingestFacts(const RuntimeContractFacts &facts) {
    facts_ = facts;
  }

  void advancePhase() {
    if (!requested_) {
      phase_ = ShutdownPhase::running;
      last_message_ = facts_.message.empty() ? std::string{"shutdown not requested"} : facts_.message;
      return;
    }

    if (facts_.faulted || facts_.runtime_phase == RuntimePhase::faulted) {
      phase_ = ShutdownPhase::faulted;
      last_message_ = facts_.message.empty() ? std::string{"shutdown faulted"} : facts_.message;
      return;
    }

    advanceStablePhase();
  }

  [[nodiscard]] RuntimeContractView emitView() const {
    RuntimeContractView view;
    view.contract_version = kRuntimeContractVersion;
    view.code = contractCode();
    view.owner = facts_.owner;
    view.runtime_phase = facts_.runtime_phase;
    view.shutdown_phase = phase_;
    view.active_request_count = facts_.active_request_count;
    view.active_goal_count = facts_.active_goal_count;
    view.backend_quiescent = facts_.backend_quiescent;
    view.safe_to_delete =
        phase_ == ShutdownPhase::safe_to_delete ||
        phase_ == ShutdownPhase::safe_to_stop_world ||
        phase_ == ShutdownPhase::finished;
    view.safe_to_stop_world =
        phase_ == ShutdownPhase::safe_to_stop_world ||
        phase_ == ShutdownPhase::finished;
    view.message = !last_message_.empty() ? last_message_ : facts_.message;
    return view;
  }

  [[nodiscard]] bool runtimeDrained() const noexcept {
    return facts_.owner == ControlOwner::none &&
           facts_.active_request_count == 0 &&
           facts_.active_goal_count == 0 &&
           facts_.runtime_phase == RuntimePhase::idle;
  }

  [[nodiscard]] bool backendDetachedReady() const noexcept {
    return runtimeDrained() && facts_.backend_quiescent;
  }

  [[nodiscard]] bool safeToDeleteReady() const noexcept {
    return backendDetachedReady() && facts_.plugin_detached;
  }

  [[nodiscard]] bool safeToStopWorldReady() const noexcept {
    return phase_ == ShutdownPhase::safe_to_delete && safeToDeleteReady();
  }

  [[nodiscard]] RuntimeContractCode contractCode() const noexcept {
    if (phase_ == ShutdownPhase::faulted || facts_.faulted || facts_.runtime_phase == RuntimePhase::faulted) {
      return RuntimeContractCode::faulted;
    }
    if (!requested_) {
      return RuntimeContractCode::ok;
    }
    switch (phase_) {
      case ShutdownPhase::running:
        return RuntimeContractCode::shutdown_requested;
      case ShutdownPhase::draining:
        return RuntimeContractCode::runtime_draining;
      case ShutdownPhase::backend_detached:
        return RuntimeContractCode::backend_detached;
      case ShutdownPhase::safe_to_delete:
        return RuntimeContractCode::safe_to_delete;
      case ShutdownPhase::safe_to_stop_world:
        return RuntimeContractCode::safe_to_stop_world;
      case ShutdownPhase::finished:
        return RuntimeContractCode::finished;
      case ShutdownPhase::faulted:
        return RuntimeContractCode::faulted;
    }
    return RuntimeContractCode::ok;
  }

  void advanceStablePhase() {
    switch (phase_) {
      case ShutdownPhase::running:
        phase_ = ShutdownPhase::draining;
        last_message_ = "shutdown draining";
        break;
      case ShutdownPhase::draining:
        last_message_ = facts_.message.empty() ? std::string{"shutdown draining"} : facts_.message;
        if (backendDetachedReady()) {
          phase_ = ShutdownPhase::backend_detached;
          last_message_ = "backend detached";
        }
        break;
      case ShutdownPhase::backend_detached:
        last_message_ = facts_.message.empty() ? std::string{"backend detached"} : facts_.message;
        if (safeToDeleteReady()) {
          phase_ = ShutdownPhase::safe_to_delete;
          last_message_ = "safe to delete";
        }
        break;
      case ShutdownPhase::safe_to_delete:
        last_message_ = facts_.message.empty() ? std::string{"safe to delete"} : facts_.message;
        if (safeToStopWorldReady()) {
          phase_ = ShutdownPhase::safe_to_stop_world;
          last_message_ = "safe to stop world";
        }
        break;
      case ShutdownPhase::safe_to_stop_world:
        last_message_ = facts_.message.empty() ? std::string{"safe to stop world"} : facts_.message;
        break;
      case ShutdownPhase::finished:
        last_message_ = "shutdown finished";
        break;
      case ShutdownPhase::faulted:
        last_message_ = facts_.message.empty() ? std::string{"shutdown faulted"} : facts_.message;
        break;
    }
  }

  bool requested_ = false;
  ShutdownPhase phase_ = ShutdownPhase::running;
  RuntimeContractFacts facts_{};
  std::string last_message_{"idle"};
};

}  // namespace rokae_xmate3_ros2::runtime

#endif  // ROKAE_XMATE3_ROS2_RUNTIME_SHUTDOWN_COORDINATOR_PUBLIC_HPP
