#include "runtime/motion_runtime_internal.hpp"

namespace rokae_xmate3_ros2::runtime {

MotionRequest MotionRuntime::normalizeRequestLocked(MotionRequest request) {
  if (request.request_token == 0) {
    request.request_token = next_request_token_++;
  }
  if (request.original_total_segments == 0) {
    request.original_total_segments = request.commands.size();
  }
  request.command_index_offset = std::min(request.command_index_offset, request.original_total_segments);
  return request;
}

std::size_t MotionRuntime::activeCommandOffsetLocked() const {
  return active_request_ ? active_request_->command_index_offset : 0U;
}

std::size_t MotionRuntime::activeTotalSegmentsLocked() const {
  return active_request_ ? active_request_->original_total_segments : active_status_.total_segments;
}

bool MotionRuntime::startPlanningLocked(const MotionRequest &request, const std::string &message) {
  if (pending_request_ || queued_plan_) {
    return false;
  }
  pending_request_ = request;
  active_request_ = request;
  active_request_id_ = request.request_id;
  active_request_token_ = request.request_token;

  RuntimeEvent planning_event;
  planning_event.type = RuntimeEventType::planning_requested;
  planning_event.request_id = request.request_id;
  planning_event.message = message.empty() ? std::string{"moveStart committed; planning"} : message;
  planning_event.total_segments = request.original_total_segments;
  planning_event.completed_segments = request.command_index_offset;
  planning_event.current_segment_index = request.command_index_offset;
  state_machine_.apply(active_status_, runtime_phase_, planning_event);
  (void)setOwnerLocked(ControlOwner::none, "planning");
  rememberStatus(active_status_);
  planner_cv_.notify_one();
  return true;
}

bool MotionRuntime::queue(const MotionRequest &request, std::string &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!queue_initialized_) {
    message = "moveReset() must initialize the NRT queue before queueing";
    return false;
  }
  if (request.request_id.empty()) {
    message = "request_id is empty";
    return false;
  }
  if (request.commands.empty()) {
    message = "motion request is empty";
    return false;
  }
  if (!active_request_id_.empty() && !is_terminal_state(active_status_.state) &&
      active_status_.state != ExecutionState::paused && active_status_.state != ExecutionState::queued) {
    message = "runtime is busy with another motion request";
    return false;
  }
  if (staged_request_ || pending_request_ || queued_plan_ || executor_.hasActivePlan() || using_backend_trajectory_) {
    message = "runtime planner queue is busy";
    return false;
  }

  MotionRequest normalized = normalizeRequestLocked(request);
  staged_request_ = normalized;
  active_request_ = normalized;
  active_request_id_ = normalized.request_id;
  active_request_token_ = normalized.request_token;

  RuntimeEvent queued_event;
  queued_event.type = RuntimeEventType::request_queued;
  queued_event.request_id = normalized.request_id;
  queued_event.message = "queued awaiting moveStart";
  queued_event.total_segments = normalized.original_total_segments;
  queued_event.completed_segments = normalized.command_index_offset;
  queued_event.current_segment_index = normalized.command_index_offset;
  state_machine_.apply(active_status_, runtime_phase_, queued_event);
  (void)setOwnerLocked(ControlOwner::none, "queued awaiting moveStart");
  rememberStatus(active_status_);
  message.clear();
  return true;
}

bool MotionRuntime::submit(const MotionRequest &request, std::string &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!queue_initialized_) {
    message = "moveReset() must initialize the NRT queue before submit";
    return false;
  }
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
  if (staged_request_ || pending_request_ || queued_plan_ || executor_.hasActivePlan() || using_backend_trajectory_) {
    message = "runtime planner queue is busy";
    return false;
  }

  MotionRequest normalized = normalizeRequestLocked(request);
  if (!startPlanningLocked(normalized, "planning")) {
    message = "runtime planner queue is busy";
    return false;
  }
  message.clear();
  return true;
}

bool MotionRuntime::commitQueuedRequest(std::string &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!queue_initialized_) {
    message = "moveStart requires moveReset to initialize the NRT queue";
    return false;
  }
  if (staged_request_) {
    MotionRequest staged = *staged_request_;
    staged_request_.reset();
    if (!startPlanningLocked(staged, "moveStart committed; planning")) {
      staged_request_ = staged;
      message = "runtime planner queue is busy";
      return false;
    }
    message = "move start committed";
    return true;
  }
  if (pending_request_ || queued_plan_ || executor_.hasActivePlan() || using_backend_trajectory_) {
    message = "request already started";
    return true;
  }
  message = "moveStart rejected because no queued NRT request exists";
  return false;
}

MotionRequest MotionRuntime::buildResumeRequestLocked(const RobotSnapshot &snapshot) const {
  MotionRequest resume;
  if (!active_request_) {
    return resume;
  }
  resume = *active_request_;
  const std::size_t offset = std::min(active_status_.current_segment_index, resume.commands.size());
  resume.start_joints.assign(snapshot.joint_position.begin(), snapshot.joint_position.end());
  if (offset < resume.commands.size()) {
    resume.commands.erase(resume.commands.begin(), resume.commands.begin() + static_cast<long>(offset));
  } else {
    resume.commands.clear();
  }
  resume.command_index_offset = offset;
  return resume;
}

RobotSnapshot MotionRuntime::buildSyntheticPauseSnapshotLocked() const {
  RobotSnapshot snapshot;
  snapshot.power_on = true;
  snapshot.drag_mode = false;
  const auto fill_from = [&](const std::optional<MotionRequest> &request) {
    if (!request || request->start_joints.size() < snapshot.joint_position.size()) {
      return false;
    }
    std::copy_n(request->start_joints.begin(), snapshot.joint_position.size(), snapshot.joint_position.begin());
    return true;
  };
  if (fill_from(active_request_)) {
    return snapshot;
  }
  if (fill_from(staged_request_)) {
    return snapshot;
  }
  if (fill_from(pending_request_)) {
    return snapshot;
  }
  return snapshot;
}

void MotionRuntime::pause(const std::string &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto snapshot = buildSyntheticPauseSnapshotLocked();
  if (active_request_id_.empty() && !staged_request_) {
    RuntimeEvent reset_event;
    reset_event.type = RuntimeEventType::reset;
    state_machine_.apply(active_status_, runtime_phase_, reset_event);
    return;
  }

  MotionRequest paused_request;
  bool has_resume_request = false;
  if (active_request_) {
    paused_request = buildResumeRequestLocked(snapshot);
    has_resume_request = !paused_request.request_id.empty() && !paused_request.commands.empty();
  } else if (staged_request_) {
    paused_request = *staged_request_;
    has_resume_request = true;
  } else if (pending_request_) {
    paused_request = *pending_request_;
    has_resume_request = true;
  }

  if (has_resume_request) {
    paused_request = normalizeRequestLocked(paused_request);
    staged_request_ = paused_request;
    active_request_ = paused_request;
    active_request_id_ = paused_request.request_id;
    active_request_token_ = paused_request.request_token;
  } else {
    staged_request_.reset();
  }

  pending_request_.reset();
  queued_plan_.reset();
  active_trajectory_plan_.reset();
  active_trajectory_goal_.reset();
  using_backend_trajectory_ = false;
  executor_.stop(snapshot);

  RuntimeEvent paused_event;
  paused_event.type = RuntimeEventType::paused;
  paused_event.request_id = active_request_id_;
  paused_event.message = message.empty() ? std::string{"paused"} : message;
  paused_event.total_segments = activeTotalSegmentsLocked();
  paused_event.completed_segments = active_status_.completed_segments;
  paused_event.current_segment_index = active_status_.current_segment_index;
  paused_event.execution_backend = active_status_.execution_backend;
  state_machine_.apply(active_status_, runtime_phase_, paused_event);
  (void)setOwnerLocked(ControlOwner::none, paused_event.message);
  rememberStatus(active_status_);
  last_view_update_time_ = std::chrono::steady_clock::now();
}

void MotionRuntime::pause(const RobotSnapshot &snapshot, const std::string &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_request_id_.empty() && !staged_request_) {
    RuntimeEvent reset_event;
    reset_event.type = RuntimeEventType::reset;
    state_machine_.apply(active_status_, runtime_phase_, reset_event);
    return;
  }

  MotionRequest paused_request;
  bool has_resume_request = false;
  if (active_request_) {
    paused_request = buildResumeRequestLocked(snapshot);
    has_resume_request = !paused_request.request_id.empty() && !paused_request.commands.empty();
  } else if (staged_request_) {
    paused_request = *staged_request_;
    has_resume_request = true;
  } else if (pending_request_) {
    paused_request = *pending_request_;
    has_resume_request = true;
  }

  if (has_resume_request) {
    paused_request = normalizeRequestLocked(paused_request);
    staged_request_ = paused_request;
    active_request_ = paused_request;
    active_request_id_ = paused_request.request_id;
    active_request_token_ = paused_request.request_token;
  } else {
    staged_request_.reset();
  }

  if (using_backend_trajectory_ && backend_ != nullptr) {
    backend_->cancelTrajectoryExecution(message);
  }
  pending_request_.reset();
  queued_plan_.reset();
  active_trajectory_plan_.reset();
  active_trajectory_goal_.reset();
  using_backend_trajectory_ = false;
  executor_.stop(snapshot);

  RuntimeEvent paused_event;
  paused_event.type = RuntimeEventType::paused;
  paused_event.request_id = active_request_id_;
  paused_event.message = message.empty() ? std::string{"paused"} : message;
  paused_event.total_segments = activeTotalSegmentsLocked();
  paused_event.completed_segments = active_status_.completed_segments;
  paused_event.current_segment_index = active_status_.current_segment_index;
  paused_event.execution_backend = active_status_.execution_backend;
  state_machine_.apply(active_status_, runtime_phase_, paused_event);
  (void)setOwnerLocked(ControlOwner::none, paused_event.message);
  rememberStatus(active_status_);
  last_view_update_time_ = std::chrono::steady_clock::now();
}

void MotionRuntime::stop(const std::string &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_request_id_.empty() && !staged_request_) {
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
  stop_event.total_segments = activeTotalSegmentsLocked();
  stop_event.completed_segments = active_status_.completed_segments;
  stop_event.current_segment_index = active_status_.current_segment_index;
  stop_event.execution_backend = (!using_backend_trajectory_ && executor_.hasActivePlan())
                                     ? ExecutionBackend::effort
                                     : active_status_.execution_backend;
  stop_event.terminal_success = false;
  state_machine_.apply(active_status_, runtime_phase_, stop_event);
  (void)setOwnerLocked(ControlOwner::none, stop_event.message);
  rememberStatus(active_status_);
  staged_request_.reset();
  pending_request_.reset();
  queued_plan_.reset();
  active_request_.reset();
  active_trajectory_plan_.reset();
  active_trajectory_goal_.reset();
  using_backend_trajectory_ = false;
  executor_.reset();
  last_view_update_time_ = std::chrono::steady_clock::now();
}

void MotionRuntime::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  queue_initialized_ = true;
  if (using_backend_trajectory_ && backend_ != nullptr) {
    backend_->cancelTrajectoryExecution("reset");
  }
  staged_request_.reset();
  pending_request_.reset();
  queued_plan_.reset();
  active_request_.reset();
  active_request_id_.clear();
  active_request_token_ = 0;
  RuntimeEvent reset_event;
  reset_event.type = RuntimeEventType::reset;
  state_machine_.apply(active_status_, runtime_phase_, reset_event);
  (void)owner_arbiter_.clear("reset");
  executor_.reset();
  active_trajectory_plan_.reset();
  active_trajectory_goal_.reset();
  using_backend_trajectory_ = false;
  last_view_update_time_ = std::chrono::steady_clock::now();
  status_cv_.notify_all();
}

void MotionRuntime::clearForModeChange(const std::string &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  queue_initialized_ = false;
  if (using_backend_trajectory_ && backend_ != nullptr) {
    backend_->cancelTrajectoryExecution(message.empty() ? std::string{"motion mode changed"} : message);
  }
  staged_request_.reset();
  pending_request_.reset();
  queued_plan_.reset();
  active_request_.reset();
  active_request_id_.clear();
  active_request_token_ = 0;
  RuntimeEvent reset_event;
  reset_event.type = RuntimeEventType::reset;
  state_machine_.apply(active_status_, runtime_phase_, reset_event);
  (void)owner_arbiter_.clear(message.empty() ? std::string{"motion mode changed"} : message);
  executor_.reset();
  active_trajectory_plan_.reset();
  active_trajectory_goal_.reset();
  using_backend_trajectory_ = false;
  last_view_update_time_ = std::chrono::steady_clock::now();
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
    if (active_request_token_ != request.request_token) {
      continue;
    }
    if (!plan.valid()) {
      RuntimeEvent fail_event;
      fail_event.type = RuntimeEventType::planning_rejected;
      fail_event.request_id = request.request_id;
      fail_event.message = plan.error_message.empty() ? std::string{"planning failed"} : plan.error_message;
      fail_event.total_segments = request.original_total_segments;
      fail_event.completed_segments = request.command_index_offset;
      fail_event.current_segment_index = request.command_index_offset;
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
    queued_event.total_segments = request.original_total_segments;
    queued_event.completed_segments = request.command_index_offset;
    queued_event.current_segment_index = request.command_index_offset;
    state_machine_.apply(active_status_, runtime_phase_, queued_event);
    (void)setOwnerLocked(ControlOwner::none, "queued");
    rememberStatus(active_status_);
  }
}

}  // namespace rokae_xmate3_ros2::runtime
