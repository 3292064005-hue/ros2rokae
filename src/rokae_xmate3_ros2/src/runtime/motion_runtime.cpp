#include "runtime/motion_runtime_internal.hpp"

#include <utility>

namespace rokae_xmate3_ros2::runtime {

MotionRuntime::MotionRuntime() : planner_thread_(&MotionRuntime::plannerLoop, this) {}

MotionRuntime::~MotionRuntime() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_ = true;
  }
  planner_cv_.notify_all();
  status_cv_.notify_all();
  if (planner_thread_.joinable()) {
    planner_thread_.join();
  }
}

void MotionRuntime::attachBackend(BackendInterface *backend) {
  std::lock_guard<std::mutex> lock(mutex_);
  backend_ = backend;
}

void MotionRuntime::setExecutorConfig(const MotionExecutorConfig &config) {
  std::lock_guard<std::mutex> lock(mutex_);
  executor_.setConfig(config);
}

void MotionRuntime::setRuntimePhaseLocked(RuntimePhase phase, const std::string &message) {
  RuntimeEvent event;
  event.type = RuntimeEventType::phase_override;
  event.phase = phase;
  event.message = message;
  state_machine_.apply(active_status_, runtime_phase_, event);
}

void MotionRuntime::setActiveSpeedScale(double scale) {
  std::lock_guard<std::mutex> lock(mutex_);
  active_speed_scale_ = clamp_active_speed_scale(scale);
}

bool MotionRuntime::setOwnerLocked(ControlOwner owner, const std::string &reason) {
  OwnerArbiter::TransitionResult transition;
  switch (owner) {
    case ControlOwner::trajectory:
      transition = owner_arbiter_.claimTrajectory(reason);
      break;
    case ControlOwner::effort:
      transition = owner_arbiter_.claimEffort(reason);
      break;
    case ControlOwner::none:
    default:
      transition = owner_arbiter_.clear(reason);
      break;
  }
  RuntimeEvent event;
  event.type = RuntimeEventType::owner_changed;
  event.owner = owner_arbiter_.current();
  event.message = (!transition.accepted && !transition.reason.empty()) ? transition.reason : std::string{};
  state_machine_.apply(active_status_, runtime_phase_, event);
  return transition.accepted;
}

bool MotionRuntime::syncOwnerLocked(BackendInterface &backend,
                                    ControlOwner owner,
                                    const std::string &reason) {
  const bool accepted = setOwnerLocked(owner, reason);
  backend.setControlOwner(active_status_.control_owner);
  return accepted;
}

double MotionRuntime::activeSpeedScale() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_speed_scale_;
}

RuntimePhase MotionRuntime::runtimePhase() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return runtime_phase_;
}

RuntimeContractFacts MotionRuntime::contractFacts() const {
  std::lock_guard<std::mutex> lock(mutex_);
  RuntimeContractFacts facts;
  facts.owner = owner_arbiter_.current();
  facts.runtime_phase = runtime_phase_;
  facts.active_request_count = active_request_id_.empty() ? 0u : 1u;
  facts.active_goal_count = (using_backend_trajectory_ && active_trajectory_goal_.has_value()) ? 1u : 0u;
  facts.backend_quiescent = facts.active_goal_count == 0;
  facts.faulted = runtime_phase_ == RuntimePhase::faulted;
  facts.message = active_status_.message;
  return facts;
}

std::size_t MotionRuntime::activeRequestCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_request_id_.empty() ? 0u : 1u;
}

std::size_t MotionRuntime::activeGoalCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return (using_backend_trajectory_ && active_trajectory_goal_.has_value()) ? 1u : 0u;
}

}  // namespace rokae_xmate3_ros2::runtime
