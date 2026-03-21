#include "runtime/motion_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace rokae_xmate3_ros2::runtime {
namespace {
constexpr std::size_t kMaxCachedStatuses = 16;
constexpr double kExecutionGoalMinDt = 1e-3;
constexpr double kExecutionGoalRetimingEps = 1e-6;
constexpr double kTrajectoryCompletionJointToleranceRad = 0.03;

double clamp_active_speed_scale(double scale) {
  return std::clamp(scale, 0.05, 2.0);
}

bool statusEquivalent(const RuntimeStatus &lhs, const RuntimeStatus &rhs) {
  return lhs.request_id == rhs.request_id && lhs.state == rhs.state && lhs.message == rhs.message &&
         lhs.total_segments == rhs.total_segments && lhs.completed_segments == rhs.completed_segments &&
         lhs.current_segment_index == rhs.current_segment_index &&
         lhs.terminal_success == rhs.terminal_success &&
         lhs.execution_backend == rhs.execution_backend;
}

std::string summarize_plan_notes(const MotionPlan &plan) {
  if (plan.notes.empty()) {
    return "plan ready";
  }
  std::string message = "plan ready";
  for (const auto &note : plan.notes) {
    if (note.empty()) {
      continue;
    }
    message += "; " + note;
  }
  return message;
}

struct FlattenedPlanPoint {
  std::vector<double> position;
  double absolute_time = 0.0;
};

struct TrajectoryProgress {
  std::size_t completed_segments = 0;
  std::size_t current_segment_index = 0;
};

std::vector<double> snapshot_to_vector(const std::array<double, 6> &values) {
  return std::vector<double>(values.begin(), values.end());
}

double max_joint_error(const std::array<double, 6> &actual, const std::vector<double> &target) {
  double error = 0.0;
  for (std::size_t i = 0; i < actual.size() && i < target.size(); ++i) {
    error = std::max(error, std::fabs(actual[i] - target[i]));
  }
  return error;
}

std::vector<FlattenedPlanPoint> flatten_plan(const MotionPlan &plan) {
  std::vector<FlattenedPlanPoint> flattened;
  double cumulative_time = 0.0;
  for (const auto &segment : plan.segments) {
    if (segment.joint_trajectory.empty()) {
      cumulative_time += segment.trajectory_total_time;
      continue;
    }
    for (std::size_t point_index = 0; point_index < segment.joint_trajectory.size(); ++point_index) {
      if (!flattened.empty() && point_index == 0) {
        continue;
      }
      FlattenedPlanPoint point;
      point.position = segment.joint_trajectory[point_index];
      point.absolute_time = cumulative_time + static_cast<double>(point_index) * segment.trajectory_dt;
      flattened.push_back(std::move(point));
    }
    cumulative_time += segment.trajectory_total_time;
  }
  return flattened;
}

TrajectoryExecutionGoal build_execution_goal(const MotionPlan &plan,
                                            double speed_scale,
                                            const std::array<double, 6> *start_override,
                                            double original_time_offset) {
  TrajectoryExecutionGoal goal;
  goal.request_id = plan.request_id;
  goal.total_segments = plan.segments.size();
  goal.original_time_offset = std::max(original_time_offset, 0.0);
  goal.original_time_scale = clamp_active_speed_scale(speed_scale);

  const auto flattened = flatten_plan(plan);
  if (flattened.empty()) {
    return goal;
  }

  std::vector<double> absolute_segment_end_times;
  absolute_segment_end_times.reserve(plan.segments.size());
  double cumulative_time = 0.0;
  for (const auto &segment : plan.segments) {
    cumulative_time += segment.trajectory_total_time;
    absolute_segment_end_times.push_back(cumulative_time);
  }

  while (goal.segment_index_offset < absolute_segment_end_times.size() &&
         absolute_segment_end_times[goal.segment_index_offset] <= goal.original_time_offset + kExecutionGoalRetimingEps) {
    ++goal.segment_index_offset;
  }

  if (start_override != nullptr) {
    goal.points.push_back(TrajectoryExecutionPoint{snapshot_to_vector(*start_override), 0.0});
  }

  for (const auto &point : flattened) {
    if (point.absolute_time + kExecutionGoalRetimingEps < goal.original_time_offset) {
      continue;
    }
    double relative_time = std::max(point.absolute_time - goal.original_time_offset, 0.0) / goal.original_time_scale;
    if (!goal.points.empty() &&
        relative_time <= goal.points.back().time_from_start + kExecutionGoalRetimingEps) {
      relative_time = goal.points.back().time_from_start + kExecutionGoalMinDt;
    }
    goal.points.push_back(TrajectoryExecutionPoint{point.position, relative_time});
  }

  if (goal.points.empty()) {
    goal.points.push_back(TrajectoryExecutionPoint{flattened.back().position, 0.0});
  }
  if (goal.points.size() == 1) {
    auto hold_point = goal.points.back();
    hold_point.time_from_start = std::max(kExecutionGoalMinDt, hold_point.time_from_start + kExecutionGoalMinDt);
    goal.points.push_back(std::move(hold_point));
  }

  for (std::size_t index = goal.segment_index_offset; index < absolute_segment_end_times.size(); ++index) {
    double relative_time =
        std::max(absolute_segment_end_times[index] - goal.original_time_offset, 0.0) / goal.original_time_scale;
    if (!goal.segment_end_times.empty() &&
        relative_time <= goal.segment_end_times.back() + kExecutionGoalRetimingEps) {
      relative_time = goal.segment_end_times.back() + kExecutionGoalMinDt;
    }
    goal.segment_end_times.push_back(relative_time);
  }

  const double final_required_time = goal.segment_end_times.empty() ? goal.points.back().time_from_start
                                                                    : goal.segment_end_times.back();
  if (goal.points.back().time_from_start + kExecutionGoalRetimingEps < final_required_time) {
    auto final_point = goal.points.back();
    final_point.time_from_start = final_required_time;
    goal.points.push_back(std::move(final_point));
  }

  return goal;
}

TrajectoryProgress compute_trajectory_progress(const TrajectoryExecutionGoal &goal,
                                               double desired_time_from_start) {
  TrajectoryProgress progress;
  progress.completed_segments = goal.segment_index_offset;
  progress.current_segment_index = goal.segment_index_offset;
  for (double segment_end_time : goal.segment_end_times) {
    if (desired_time_from_start + kExecutionGoalRetimingEps >= segment_end_time) {
      ++progress.completed_segments;
      ++progress.current_segment_index;
      continue;
    }
    break;
  }
  if (goal.total_segments > 0) {
    progress.current_segment_index =
        std::min(progress.current_segment_index, goal.total_segments - 1);
    progress.completed_segments =
        std::min(progress.completed_segments, goal.total_segments);
  }
  return progress;
}
}  // namespace

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

void MotionRuntime::setActiveSpeedScale(double scale) {
  std::lock_guard<std::mutex> lock(mutex_);
  active_speed_scale_ = clamp_active_speed_scale(scale);
}

double MotionRuntime::activeSpeedScale() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_speed_scale_;
}

void MotionRuntime::rememberStatus(RuntimeStatus &status) {
  if (status.request_id.empty()) {
    return;
  }

  const auto it = status_cache_.find(status.request_id);
  if (it != status_cache_.end() && statusEquivalent(it->second, status)) {
    status.revision = it->second.revision;
    return;
  }

  status.revision = next_status_revision_++;
  status_cache_[status.request_id] = status;
  if (std::find(status_order_.begin(), status_order_.end(), status.request_id) == status_order_.end()) {
    status_order_.push_back(status.request_id);
  }
  while (status_order_.size() > kMaxCachedStatuses) {
    status_cache_.erase(status_order_.front());
    status_order_.pop_front();
  }
  status_cv_.notify_all();
}

RuntimeView MotionRuntime::buildViewLocked(const std::string *request_id) const {
  RuntimeView view;
  RuntimeStatus effective_status;
  bool found_status = false;

  if (request_id != nullptr && !request_id->empty()) {
    const auto it = status_cache_.find(*request_id);
    if (it != status_cache_.end()) {
      effective_status = it->second;
      found_status = true;
    } else if (active_status_.request_id == *request_id) {
      effective_status = active_status_;
      found_status = true;
    } else {
      effective_status = RuntimeStatus{*request_id, ExecutionState::idle, "unknown request", 0, 0, 0, false, 0};
    }
    view.has_request = found_status;
  } else if (!active_request_id_.empty()) {
    effective_status = active_status_;
    found_status = true;
    view.has_request = true;
  } else if (!active_status_.request_id.empty()) {
    effective_status = active_status_;
    found_status = true;
    view.has_request = !active_status_.terminal();
  } else if (!status_order_.empty()) {
    const auto it = status_cache_.find(status_order_.back());
    if (it != status_cache_.end()) {
      effective_status = it->second;
      found_status = true;
    }
  }

  if (!found_status) {
    effective_status = RuntimeStatus{};
  }

  const bool request_slot_busy =
      pending_request_.has_value() || queued_plan_.has_value() || executor_.hasActivePlan() ||
      using_backend_trajectory_ || active_trajectory_plan_.has_value();

  view.status = effective_status;
  view.active_motion = is_active_state(effective_status.state);
  view.terminal = effective_status.terminal();
  view.can_accept_request = (not request_slot_busy) &&
                            (active_request_id_.empty() || is_terminal_state(active_status_.state));
  return view;
}

bool MotionRuntime::canAcceptRequest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool request_slot_busy =
      pending_request_.has_value() || queued_plan_.has_value() || executor_.hasActivePlan() ||
      using_backend_trajectory_ || active_trajectory_plan_.has_value();
  return (not request_slot_busy) && (active_request_id_.empty() || is_terminal_state(active_status_.state));
}

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
  active_status_ = RuntimeStatus{
      request.request_id,
      ExecutionState::planning,
      "planning",
      request.commands.size(),
      0,
      0,
      false,
  };
  active_status_.execution_backend = ExecutionBackend::none;
  rememberStatus(active_status_);
  planner_cv_.notify_one();
  message.clear();
  return true;
}

void MotionRuntime::stop(const std::string &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_request_id_.empty()) {
    active_status_ = RuntimeStatus{};
    return;
  }

  if (using_backend_trajectory_ && backend_ != nullptr) {
    backend_->cancelTrajectoryExecution(message);
  }

  active_status_.state = ExecutionState::stopped;
  active_status_.message = message;
  active_status_.terminal_success = false;
  if (!using_backend_trajectory_ && executor_.hasActivePlan()) {
    active_status_.execution_backend = ExecutionBackend::effort;
  }
  rememberStatus(active_status_);
  pending_request_.reset();
  queued_plan_.reset();
  active_trajectory_plan_.reset();
  active_trajectory_goal_.reset();
  using_backend_trajectory_ = false;
  executor_.stop(RobotSnapshot{});
}

void MotionRuntime::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (using_backend_trajectory_ && backend_ != nullptr) {
    backend_->cancelTrajectoryExecution("reset");
  }
  pending_request_.reset();
  queued_plan_.reset();
  active_request_id_.clear();
  active_status_ = RuntimeStatus{};
  executor_.reset();
  active_trajectory_plan_.reset();
  active_trajectory_goal_.reset();
  using_backend_trajectory_ = false;
  status_cv_.notify_all();
}

RuntimeStatus MotionRuntime::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_status_;
}

RuntimeStatus MotionRuntime::status(const std::string &request_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = status_cache_.find(request_id);
  if (it != status_cache_.end()) {
    return it->second;
  }
  if (active_status_.request_id == request_id) {
    return active_status_;
  }
  return RuntimeStatus{request_id, ExecutionState::idle, "unknown request", 0, 0, 0, false, 0};
}

RuntimeView MotionRuntime::view() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return buildViewLocked(nullptr);
}

RuntimeView MotionRuntime::view(const std::string &request_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return buildViewLocked(&request_id);
}

RuntimeStatus MotionRuntime::waitForUpdate(const std::string &request_id,
                                           std::uint64_t last_revision,
                                           std::chrono::milliseconds timeout) const {
  std::unique_lock<std::mutex> lock(mutex_);
  const auto lookup_status = [this, &request_id]() -> RuntimeStatus {
    const auto it = status_cache_.find(request_id);
    if (it != status_cache_.end()) {
      return it->second;
    }
    if (active_status_.request_id == request_id) {
      return active_status_;
    }
    return RuntimeStatus{request_id, ExecutionState::idle, "unknown request", 0, 0, 0, false, 0};
  };

  const auto status_changed = [this, &lookup_status, last_revision]() {
    return shutdown_ || lookup_status().revision > last_revision;
  };

  if (timeout.count() > 0) {
    status_cv_.wait_for(lock, timeout, status_changed);
  }
  return lookup_status();
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
      active_status_.state = ExecutionState::failed;
      active_status_.message = plan.error_message.empty() ? "planning failed" : plan.error_message;
      active_status_.terminal_success = false;
      rememberStatus(active_status_);
      continue;
    }

    queued_plan_ = std::move(plan);
    active_status_.state = ExecutionState::queued;
    active_status_.message = summarize_plan_notes(*queued_plan_);
    active_status_.total_segments = queued_plan_->segments.size();
    active_status_.completed_segments = 0;
    active_status_.current_segment_index = 0;
    active_status_.execution_backend = ExecutionBackend::none;
    rememberStatus(active_status_);
  }
}

RuntimeStatus MotionRuntime::tick(BackendInterface &backend, double dt) {
  const auto snapshot = backend.readSnapshot();

  std::lock_guard<std::mutex> lock(mutex_);
  if (active_request_id_.empty()) {
    const auto step = executor_.tick(snapshot, dt, 1.0);
    if (step.command.has_effort) {
      backend.applyControl(step.command);
    } else {
      backend.clearControl();
    }
    active_status_ = RuntimeStatus{};
    return active_status_;
  }

  if (active_status_.state == ExecutionState::failed || active_status_.state == ExecutionState::stopped) {
    if (using_backend_trajectory_) {
      backend.cancelTrajectoryExecution(active_status_.message.empty() ? "stopped" : active_status_.message);
      using_backend_trajectory_ = false;
      active_trajectory_plan_.reset();
      active_trajectory_goal_.reset();
    }
    executor_.stop(snapshot);
    const auto step = executor_.tick(snapshot, dt, 1.0);
    if (step.command.has_effort) {
      backend.applyControl(step.command);
    } else {
      backend.clearControl();
    }
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
        active_status_.state = ExecutionState::executing;
        active_status_.message = "executing";
        active_status_.current_segment_index = active_trajectory_goal_->segment_index_offset;
        active_status_.completed_segments = active_trajectory_goal_->segment_index_offset;
        active_status_.execution_backend = ExecutionBackend::jtc;
        queued_plan_.reset();
        started_backend_trajectory = true;
      } else if (!trajectory_message.empty()) {
        active_status_.message = trajectory_message + "; falling back to effort runtime";
      }
    }

    if (!started_backend_trajectory) {
      executor_.loadPlan(std::move(*queued_plan_));
      queued_plan_.reset();
      active_status_.state = ExecutionState::executing;
      active_status_.message = active_status_.message.empty() ? "executing" : active_status_.message;
      active_status_.current_segment_index = 0;
      active_status_.completed_segments = 0;
      active_status_.execution_backend = ExecutionBackend::effort;
    }
  }

  if (using_backend_trajectory_) {
    auto trajectory_state = backend.readTrajectoryExecutionState();
    if (active_trajectory_plan_ && active_trajectory_goal_ && trajectory_state.active &&
        !trajectory_state.completed &&
        std::fabs(active_speed_scale_ - dispatched_speed_scale_) > 1e-6) {
      const double current_original_time =
          active_trajectory_goal_->original_time_offset +
          std::max(trajectory_state.desired_time_from_start, 0.0) * active_trajectory_goal_->original_time_scale;
      auto retimed_goal =
          build_execution_goal(*active_trajectory_plan_, active_speed_scale_, &snapshot.joint_position, current_original_time);
      std::string retime_message;
      if (!retimed_goal.points.empty() && backend.startTrajectoryExecution(retimed_goal, retime_message)) {
        active_trajectory_goal_ = std::move(retimed_goal);
        dispatched_speed_scale_ = active_speed_scale_;
        trajectory_state = backend.readTrajectoryExecutionState();
        active_status_.message = "retimed";
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
    active_status_.execution_backend = ExecutionBackend::jtc;

    if (trajectory_state.completed) {
      using_backend_trajectory_ = false;
      const double final_joint_error =
          (active_trajectory_plan_ && !active_trajectory_plan_->segments.empty())
              ? max_joint_error(snapshot.joint_position, active_trajectory_plan_->segments.back().target_joints)
              : 0.0;
      active_status_.completed_segments = active_status_.total_segments;
      active_status_.current_segment_index =
          active_status_.total_segments == 0 ? 0 : active_status_.total_segments - 1;
      active_status_.message = trajectory_state.message.empty()
                                   ? (trajectory_state.succeeded ? "completed" : "trajectory execution failed")
                                   : trajectory_state.message;
      if (trajectory_state.succeeded && final_joint_error <= kTrajectoryCompletionJointToleranceRad) {
        active_status_.state = ExecutionState::completed;
        active_status_.terminal_success = true;
      } else {
        active_status_.state = ExecutionState::failed;
        active_status_.terminal_success = false;
        if (final_joint_error > kTrajectoryCompletionJointToleranceRad) {
          active_status_.message += "; final joint error exceeded tolerance";
        }
      }
      rememberStatus(active_status_);
      const auto terminal_status = active_status_;
      active_trajectory_plan_.reset();
      active_trajectory_goal_.reset();
      active_request_id_.clear();
      return terminal_status;
    }

    active_status_.state = ExecutionState::executing;
    active_status_.execution_backend = ExecutionBackend::jtc;
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
  if (step.command.has_effort) {
    backend.applyControl(step.command);
  } else {
    backend.clearControl();
  }

  const bool runtime_driving_execution =
      executor_.hasActivePlan() || step.plan_completed ||
      active_status_.state == ExecutionState::executing ||
      active_status_.state == ExecutionState::settling;
  if (runtime_driving_execution && step.state != ExecutionState::idle) {
    active_status_.state = step.state;
    active_status_.execution_backend = ExecutionBackend::effort;
  }
  active_status_.current_segment_index = step.current_segment_index;
  active_status_.completed_segments = step.completed_segments;
  active_status_.message = step.message.empty() ? to_string(active_status_.state) : step.message;

  if (step.plan_completed) {
    active_status_.state = step.state;
    active_status_.message = step.message.empty() ? to_string(step.state) : step.message;
    active_status_.terminal_success = step.terminal_success;
    active_status_.execution_backend = ExecutionBackend::effort;
  }

  rememberStatus(active_status_);
  const auto returned_status = active_status_;
  if (returned_status.terminal()) {
    active_request_id_.clear();
  }
  return returned_status;
}

}  // namespace rokae_xmate3_ros2::runtime
