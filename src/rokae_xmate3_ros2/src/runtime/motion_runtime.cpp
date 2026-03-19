#include "runtime/motion_runtime.hpp"

#include <algorithm>
#include <utility>

namespace rokae_xmate3_ros2::runtime {
namespace {
constexpr std::size_t kMaxCachedStatuses = 16;

bool statusEquivalent(const RuntimeStatus &lhs, const RuntimeStatus &rhs) {
  return lhs.request_id == rhs.request_id && lhs.state == rhs.state && lhs.message == rhs.message &&
         lhs.total_segments == rhs.total_segments && lhs.completed_segments == rhs.completed_segments &&
         lhs.current_segment_index == rhs.current_segment_index &&
         lhs.terminal_success == rhs.terminal_success;
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

  const bool request_slot_busy = pending_request_.has_value() || queued_plan_.has_value() || executor_.hasActivePlan();

  view.status = effective_status;
  view.active_motion = is_active_state(effective_status.state);
  view.terminal = effective_status.terminal();
  view.can_accept_request = (not request_slot_busy) &&
                            (active_request_id_.empty() || is_terminal_state(active_status_.state));
  return view;
}

bool MotionRuntime::canAcceptRequest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool request_slot_busy = pending_request_.has_value() || queued_plan_.has_value() || executor_.hasActivePlan();
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

  active_status_.state = ExecutionState::stopped;
  active_status_.message = message;
  active_status_.terminal_success = false;
  rememberStatus(active_status_);
  pending_request_.reset();
  queued_plan_.reset();
}

void MotionRuntime::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  pending_request_.reset();
  queued_plan_.reset();
  active_request_id_.clear();
  active_status_ = RuntimeStatus{};
  executor_.reset();
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
    active_status_.message = "plan ready";
    active_status_.total_segments = queued_plan_->segments.size();
    active_status_.completed_segments = 0;
    active_status_.current_segment_index = 0;
    rememberStatus(active_status_);
  }
}

RuntimeStatus MotionRuntime::tick(BackendInterface &backend, double dt) {
  const auto snapshot = backend.readSnapshot();

  std::lock_guard<std::mutex> lock(mutex_);
  if (active_request_id_.empty()) {
    const auto step = executor_.tick(snapshot, dt);
    if (step.command.has_effort) {
      backend.applyControl(step.command);
    } else {
      backend.clearControl();
    }
    active_status_ = RuntimeStatus{};
    return active_status_;
  }

  if (active_status_.state == ExecutionState::failed || active_status_.state == ExecutionState::stopped) {
    executor_.stop(snapshot);
    const auto step = executor_.tick(snapshot, dt);
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
    executor_.loadPlan(std::move(*queued_plan_));
    queued_plan_.reset();
    active_status_.state = ExecutionState::executing;
    active_status_.message = "executing";
    active_status_.current_segment_index = 0;
    active_status_.completed_segments = 0;
  }

  const auto step = executor_.tick(snapshot, dt);
  if (step.command.has_effort) {
    backend.applyControl(step.command);
  } else {
    backend.clearControl();
  }

  const bool runtime_driving_execution =
      executor_.hasActivePlan() || step.plan_completed ||
      active_status_.state == ExecutionState::executing ||
      active_status_.state == ExecutionState::settling;
  if ((runtime_driving_execution && step.state != ExecutionState::idle) ||
      step.state == ExecutionState::completed) {
    active_status_.state = step.state;
  }
  active_status_.current_segment_index = step.current_segment_index;
  active_status_.completed_segments = step.completed_segments;
  active_status_.message = step.message.empty() ? to_string(active_status_.state) : step.message;

  if (step.plan_completed) {
    active_status_.state = ExecutionState::completed;
    active_status_.message = step.message.empty() ? "completed" : step.message;
    active_status_.terminal_success = true;
  }

  rememberStatus(active_status_);
  const auto returned_status = active_status_;
  if (returned_status.terminal()) {
    active_request_id_.clear();
  }
  return returned_status;
}

}  // namespace rokae_xmate3_ros2::runtime
