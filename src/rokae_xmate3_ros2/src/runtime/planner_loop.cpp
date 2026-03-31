#include "runtime/motion_runtime_internal.hpp"

namespace rokae_xmate3_ros2::runtime {

bool MotionRuntime::submit(const MotionRequest &request, std::string &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (request.request_id.empty()) {
    message = "request_id is empty";
    return false;
  }
  if (request.commands.empty()) {
    message = "motion request is empty";
    return false;
  }
  if (!active_request_id_.empty() && !is_terminal_state(active_status_.state)) {
    message = "runtime is busy with another motion request";
    return false;
  }
  if (pending_request_ || queued_plan_) {
    message = "runtime planner queue is busy";
    return false;
  }

  pending_request_ = request;
  active_request_id_ = request.request_id;
  RuntimeEvent planning_event;
  planning_event.type = RuntimeEventType::planning_requested;
  planning_event.request_id = request.request_id;
  planning_event.message = "planning";
  planning_event.total_segments = request.commands.size();
  state_machine_.apply(active_status_, runtime_phase_, planning_event);
  (void)setOwnerLocked(ControlOwner::none, "planning");
  rememberStatus(active_status_);
  planner_cv_.notify_one();
  message.clear();
  return true;
}

void MotionRuntime::stop(const std::string &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_request_id_.empty()) {
    RuntimeEvent reset_event;
    reset_event.type = RuntimeEventType::reset;
    state_machine_.apply(active_status_, runtime_phase_, reset_event);
    return;
  }

  if (using_backend_trajectory_ && backend_ != nullptr) {
    backend_->cancelTrajectoryExecution(message);
  }

  RuntimeEvent stop_event;
  stop_event.type = RuntimeEventType::stopped;
  stop_event.request_id = active_request_id_;
  stop_event.message = message.empty() ? std::string{"stopped"} : message;
  stop_event.total_segments = active_status_.total_segments;
  stop_event.completed_segments = active_status_.completed_segments;
  stop_event.current_segment_index = active_status_.current_segment_index;
  stop_event.execution_backend = (!using_backend_trajectory_ && executor_.hasActivePlan())
                                     ? ExecutionBackend::effort
                                     : active_status_.execution_backend;
  stop_event.terminal_success = false;
  state_machine_.apply(active_status_, runtime_phase_, stop_event);
  (void)setOwnerLocked(ControlOwner::none, stop_event.message);
  rememberStatus(active_status_);
  pending_request_.reset();
  queued_plan_.reset();
  active_trajectory_plan_.reset();
  active_trajectory_goal_.reset();
  using_backend_trajectory_ = false;
  executor_.reset();
}

void MotionRuntime::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (using_backend_trajectory_ && backend_ != nullptr) {
    backend_->cancelTrajectoryExecution("reset");
  }
  pending_request_.reset();
  queued_plan_.reset();
  active_request_id_.clear();
  RuntimeEvent reset_event;
  reset_event.type = RuntimeEventType::reset;
  state_machine_.apply(active_status_, runtime_phase_, reset_event);
  (void)owner_arbiter_.clear("reset");
  executor_.reset();
  active_trajectory_plan_.reset();
  active_trajectory_goal_.reset();
  using_backend_trajectory_ = false;
  status_cv_.notify_all();
}

void MotionRuntime::plannerLoop() {
  while (true) {
    MotionRequest request;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      planner_cv_.wait(lock, [this]() { return shutdown_ || pending_request_.has_value(); });
      if (shutdown_) {
        return;
      }
      request = *pending_request_;
      pending_request_.reset();
    }

    auto plan = planner_.plan(request);

    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_) {
      return;
    }
    if (active_request_id_ != request.request_id) {
      continue;
    }
    if (!plan.valid()) {
      RuntimeEvent fail_event;
      fail_event.type = RuntimeEventType::planning_rejected;
      fail_event.request_id = request.request_id;
      fail_event.message = plan.error_message.empty() ? std::string{"planning failed"} : plan.error_message;
      fail_event.total_segments = active_status_.total_segments;
      fail_event.completed_segments = active_status_.completed_segments;
      fail_event.current_segment_index = active_status_.current_segment_index;
      fail_event.execution_backend = ExecutionBackend::none;
      fail_event.terminal_success = false;
      state_machine_.apply(active_status_, runtime_phase_, fail_event);
      rememberStatus(active_status_);
      continue;
    }

    queued_plan_ = std::move(plan);
    RuntimeEvent queued_event;
    queued_event.type = RuntimeEventType::plan_queued;
    queued_event.request_id = request.request_id;
    queued_event.message = summarize_plan_notes(*queued_plan_);
    queued_event.total_segments = queued_plan_->segments.size();
    queued_event.completed_segments = 0;
    queued_event.current_segment_index = 0;
    state_machine_.apply(active_status_, runtime_phase_, queued_event);
    (void)setOwnerLocked(ControlOwner::none, "queued");
    rememberStatus(active_status_);
  }
}

}  // namespace rokae_xmate3_ros2::runtime
