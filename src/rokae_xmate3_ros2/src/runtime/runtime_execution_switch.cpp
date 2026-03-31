#include "runtime/motion_runtime_internal.hpp"

namespace rokae_xmate3_ros2::runtime {

RuntimeStatus MotionRuntime::tick(BackendInterface &backend, double dt) {
  const auto snapshot = backend.readSnapshot();

  std::lock_guard<std::mutex> lock(mutex_);
  if (active_request_id_.empty()) {
    (void)syncOwnerLocked(backend, ControlOwner::none, "idle");
    if (using_backend_trajectory_) {
      backend.cancelTrajectoryExecution("cleared idle trajectory owner");
      using_backend_trajectory_ = false;
      active_trajectory_plan_.reset();
      active_trajectory_goal_.reset();
    }
    if (executor_.hasActivePlan()) {
      executor_.stop(snapshot);
    }
    backend.clearControl();
    RuntimeEvent reset_event;
    reset_event.type = RuntimeEventType::reset;
    state_machine_.apply(active_status_, runtime_phase_, reset_event);
    (void)owner_arbiter_.clear("idle");
    return active_status_;
  }

  if (active_status_.state == ExecutionState::failed || active_status_.state == ExecutionState::stopped) {
    (void)syncOwnerLocked(
        backend, ControlOwner::none, active_status_.message.empty() ? "terminal" : active_status_.message);
    if (using_backend_trajectory_) {
      backend.cancelTrajectoryExecution(active_status_.message.empty() ? "stopped" : active_status_.message);
      using_backend_trajectory_ = false;
      active_trajectory_plan_.reset();
      active_trajectory_goal_.reset();
    }
    executor_.reset();
    backend.clearControl();
    rememberStatus(active_status_);
    const auto terminal_status = active_status_;
    active_request_id_.clear();
    return terminal_status;
  }

  if (!executor_.hasActivePlan() && queued_plan_) {
    bool started_backend_trajectory = false;
    if (backend.supportsTrajectoryExecution()) {
      auto execution_goal = build_execution_goal(*queued_plan_, active_speed_scale_, nullptr, 0.0);
      std::string trajectory_message;
      if (!execution_goal.points.empty() && backend.startTrajectoryExecution(execution_goal, trajectory_message)) {
        using_backend_trajectory_ = true;
        active_trajectory_plan_ = *queued_plan_;
        active_trajectory_goal_ = std::move(execution_goal);
        dispatched_speed_scale_ = active_speed_scale_;
        RuntimeEvent start_event;
        start_event.type = RuntimeEventType::execution_started;
        start_event.request_id = active_request_id_;
        start_event.message = "executing";
        start_event.total_segments = active_status_.total_segments;
        start_event.completed_segments = active_trajectory_goal_->segment_index_offset;
        start_event.current_segment_index = active_trajectory_goal_->segment_index_offset;
        start_event.execution_backend = ExecutionBackend::jtc;
        state_machine_.apply(active_status_, runtime_phase_, start_event);
        if (!syncOwnerLocked(backend, ControlOwner::trajectory, "jtc execution")) {
          backend.cancelTrajectoryExecution(active_status_.message);
          using_backend_trajectory_ = false;
          active_trajectory_plan_.reset();
          active_trajectory_goal_.reset();
          RuntimeEvent fail_event;
          fail_event.type = RuntimeEventType::failed;
          fail_event.request_id = active_request_id_;
          fail_event.message = active_status_.message;
          fail_event.total_segments = active_status_.total_segments;
          fail_event.completed_segments = active_status_.completed_segments;
          fail_event.current_segment_index = active_status_.current_segment_index;
          fail_event.execution_backend = ExecutionBackend::jtc;
          fail_event.terminal_success = false;
          state_machine_.apply(active_status_, runtime_phase_, fail_event);
          rememberStatus(active_status_);
          const auto terminal_status = active_status_;
          active_request_id_.clear();
          return terminal_status;
        }
        queued_plan_.reset();
        started_backend_trajectory = true;
      } else if (!trajectory_message.empty()) {
        active_status_.message = trajectory_message + "; falling back to effort runtime";
      }
    }

    if (!started_backend_trajectory) {
      executor_.loadPlan(std::move(*queued_plan_));
      queued_plan_.reset();
      RuntimeEvent start_event;
      start_event.type = RuntimeEventType::execution_started;
      start_event.request_id = active_request_id_;
      start_event.message = active_status_.message.empty() ? std::string{"executing"} : active_status_.message;
      start_event.total_segments = active_status_.total_segments;
      start_event.completed_segments = 0;
      start_event.current_segment_index = 0;
      start_event.execution_backend = ExecutionBackend::effort;
      state_machine_.apply(active_status_, runtime_phase_, start_event);
      if (!syncOwnerLocked(backend, ControlOwner::effort, "effort execution")) {
        RuntimeEvent fail_event;
        fail_event.type = RuntimeEventType::failed;
        fail_event.request_id = active_request_id_;
        fail_event.message = active_status_.message.empty() ? std::string{"owner arbitration rejected"} : active_status_.message;
        fail_event.total_segments = active_status_.total_segments;
        fail_event.completed_segments = active_status_.completed_segments;
        fail_event.current_segment_index = active_status_.current_segment_index;
        fail_event.execution_backend = ExecutionBackend::effort;
        fail_event.terminal_success = false;
        state_machine_.apply(active_status_, runtime_phase_, fail_event);
        rememberStatus(active_status_);
        const auto terminal_status = active_status_;
        active_request_id_.clear();
        return terminal_status;
      }
    }
  }

  if (using_backend_trajectory_) {
    if (!syncOwnerLocked(backend, ControlOwner::trajectory, "jtc execution")) {
      backend.cancelTrajectoryExecution(active_status_.message.empty() ? "owner arbitration rejected" :
                                                                      active_status_.message);
      using_backend_trajectory_ = false;
      active_trajectory_plan_.reset();
      active_trajectory_goal_.reset();
      RuntimeEvent fail_event;
      fail_event.type = RuntimeEventType::failed;
      fail_event.request_id = active_request_id_;
      fail_event.message = active_status_.message.empty() ? std::string{"owner arbitration rejected"} : active_status_.message;
      fail_event.total_segments = active_status_.total_segments;
      fail_event.completed_segments = active_status_.completed_segments;
      fail_event.current_segment_index = active_status_.current_segment_index;
      fail_event.execution_backend = ExecutionBackend::jtc;
      fail_event.terminal_success = false;
      state_machine_.apply(active_status_, runtime_phase_, fail_event);
      rememberStatus(active_status_);
      const auto terminal_status = active_status_;
      active_request_id_.clear();
      return terminal_status;
    }
    auto trajectory_state = backend.readTrajectoryExecutionState();
    if (active_trajectory_plan_ && active_trajectory_goal_ && trajectory_state.active &&
        !trajectory_state.completed &&
        std::fabs(active_speed_scale_ - dispatched_speed_scale_) > 1e-6) {
      const double current_original_time =
          active_trajectory_goal_->original_time_offset +
          std::max(trajectory_state.desired_time_from_start, 0.0) * active_trajectory_goal_->original_time_scale;
      const auto start_override = build_start_override(trajectory_state, snapshot);
      auto retimed_goal =
          build_execution_goal(*active_trajectory_plan_, active_speed_scale_, &start_override, current_original_time);
      std::string retime_message;
      if (!retimed_goal.points.empty() && backend.startTrajectoryExecution(retimed_goal, retime_message)) {
        active_trajectory_goal_ = std::move(retimed_goal);
        dispatched_speed_scale_ = active_speed_scale_;
        trajectory_state = backend.readTrajectoryExecutionState();
        RuntimeEvent retimed_event;
        retimed_event.type = RuntimeEventType::trajectory_retimed;
        retimed_event.request_id = active_request_id_;
        retimed_event.message = "retimed";
        state_machine_.apply(active_status_, runtime_phase_, retimed_event);
      } else if (!retime_message.empty()) {
        active_status_.message = retime_message;
      }
    }

    backend.clearControl();
    if (active_trajectory_goal_) {
      const auto progress =
          compute_trajectory_progress(*active_trajectory_goal_, trajectory_state.desired_time_from_start);
      active_status_.completed_segments = progress.completed_segments;
      active_status_.current_segment_index = progress.current_segment_index;
    }
    if (trajectory_state.completed) {
      using_backend_trajectory_ = false;
      const double final_joint_error =
          (active_trajectory_plan_ && !active_trajectory_plan_->segments.empty())
              ? max_joint_error(snapshot.joint_position, active_trajectory_plan_->segments.back().target_joints)
              : 0.0;
      std::string terminal_message = trajectory_state.message.empty()
                                         ? (trajectory_state.succeeded ? std::string{"completed"}
                                                                       : std::string{"trajectory execution failed"})
                                         : trajectory_state.message;
      if (!trajectory_state.succeeded || final_joint_error > kTrajectoryCompletionJointToleranceRad) {
        if (final_joint_error > kTrajectoryCompletionJointToleranceRad) {
          terminal_message += "; final joint error exceeded tolerance";
        }
        RuntimeEvent fail_event;
        fail_event.type = RuntimeEventType::failed;
        fail_event.request_id = active_request_id_;
        fail_event.message = terminal_message;
        fail_event.total_segments = active_status_.total_segments;
        fail_event.completed_segments = active_status_.total_segments;
        fail_event.current_segment_index = active_status_.total_segments == 0 ? 0 : active_status_.total_segments - 1;
        fail_event.execution_backend = ExecutionBackend::jtc;
        fail_event.terminal_success = false;
        state_machine_.apply(active_status_, runtime_phase_, fail_event);
      } else {
        RuntimeEvent complete_event;
        complete_event.type = RuntimeEventType::completed;
        complete_event.request_id = active_request_id_;
        complete_event.message = terminal_message;
        complete_event.total_segments = active_status_.total_segments;
        complete_event.completed_segments = active_status_.total_segments;
        complete_event.current_segment_index = active_status_.total_segments == 0 ? 0 : active_status_.total_segments - 1;
        complete_event.execution_backend = ExecutionBackend::jtc;
        complete_event.terminal_success = true;
        state_machine_.apply(active_status_, runtime_phase_, complete_event);
      }
      (void)syncOwnerLocked(backend, ControlOwner::none, "jtc completed");
      rememberStatus(active_status_);
      const auto terminal_status = active_status_;
      active_trajectory_plan_.reset();
      active_trajectory_goal_.reset();
      active_request_id_.clear();
      return terminal_status;
    }

    active_status_.state = ExecutionState::executing;
    RuntimeEvent progress_event;
    progress_event.type = RuntimeEventType::progress_updated;
    progress_event.request_id = active_request_id_;
    progress_event.message = active_status_.message;
    progress_event.total_segments = active_status_.total_segments;
    progress_event.completed_segments = active_status_.completed_segments;
    progress_event.current_segment_index = active_status_.current_segment_index;
    progress_event.execution_backend = ExecutionBackend::jtc;
    state_machine_.apply(active_status_, runtime_phase_, progress_event);
    (void)setOwnerLocked(ControlOwner::trajectory, "jtc execution");
    if (!trajectory_state.message.empty() && trajectory_state.message != active_status_.message) {
      active_status_.message = trajectory_state.message;
    } else if (active_status_.message.empty()) {
      active_status_.message = "executing";
    }
    rememberStatus(active_status_);
    return active_status_;
  }

  const double trajectory_time_scale = active_speed_scale_;
  const auto step = executor_.tick(snapshot, dt, trajectory_time_scale);
  if (!syncOwnerLocked(backend, ControlOwner::effort, "effort execution")) {
    executor_.stop(snapshot);
    RuntimeEvent fail_event;
    fail_event.type = RuntimeEventType::failed;
    fail_event.request_id = active_request_id_;
    fail_event.message = active_status_.message.empty() ? std::string{"owner arbitration rejected"} : active_status_.message;
    fail_event.total_segments = active_status_.total_segments;
    fail_event.completed_segments = active_status_.completed_segments;
    fail_event.current_segment_index = active_status_.current_segment_index;
    fail_event.execution_backend = ExecutionBackend::effort;
    fail_event.terminal_success = false;
    state_machine_.apply(active_status_, runtime_phase_, fail_event);
    rememberStatus(active_status_);
    const auto terminal_status = active_status_;
    active_request_id_.clear();
    return terminal_status;
  }
  if (step.command.has_effort) {
    backend.applyControl(step.command);
  } else {
    (void)syncOwnerLocked(backend, ControlOwner::none, "effort idle");
    backend.clearControl();
  }

  const bool runtime_driving_execution =
      executor_.hasActivePlan() || step.plan_completed ||
      active_status_.state == ExecutionState::executing ||
      active_status_.state == ExecutionState::settling;
  if (runtime_driving_execution && step.state != ExecutionState::idle) {
    RuntimeEvent progress_event;
    progress_event.type = RuntimeEventType::progress_updated;
    progress_event.request_id = active_request_id_;
    progress_event.message = step.message.empty() ? std::string{to_string(step.state)} : step.message;
    progress_event.total_segments = active_status_.total_segments;
    progress_event.completed_segments = step.completed_segments;
    progress_event.current_segment_index = step.current_segment_index;
    progress_event.execution_backend = ExecutionBackend::effort;
    state_machine_.apply(active_status_, runtime_phase_, progress_event);
    active_status_.state = step.state;
    (void)setOwnerLocked(ControlOwner::effort, "effort execution");
  } else {
    active_status_.current_segment_index = step.current_segment_index;
    active_status_.completed_segments = step.completed_segments;
    active_status_.message = step.message.empty() ? active_status_.message : step.message;
  }

  if (step.plan_completed) {
    RuntimeEvent terminal_event;
    if (step.state == ExecutionState::completed_relaxed) {
      terminal_event.type = RuntimeEventType::completed_relaxed;
    } else {
      terminal_event.type = step.terminal_success ? RuntimeEventType::completed : RuntimeEventType::failed;
    }
    terminal_event.request_id = active_request_id_;
    terminal_event.message = step.message.empty() ? std::string{to_string(step.state)} : step.message;
    terminal_event.total_segments = active_status_.total_segments;
    terminal_event.completed_segments = step.completed_segments;
    terminal_event.current_segment_index = step.current_segment_index;
    terminal_event.execution_backend = ExecutionBackend::effort;
    terminal_event.terminal_success = step.terminal_success;
    state_machine_.apply(active_status_, runtime_phase_, terminal_event);
    (void)syncOwnerLocked(backend, ControlOwner::none, "effort completed");
  } else if (!runtime_driving_execution || step.state == ExecutionState::idle) {
    setRuntimePhaseLocked(RuntimePhase::idle);
    (void)syncOwnerLocked(backend, ControlOwner::none, "effort idle");
  }

  rememberStatus(active_status_);
  const auto returned_status = active_status_;
  if (returned_status.terminal()) {
    active_request_id_.clear();
  }
  return returned_status;
}

}  // namespace rokae_xmate3_ros2::runtime
