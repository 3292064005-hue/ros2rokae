#include "runtime/motion_runtime_internal.hpp"

namespace rokae_xmate3_ros2::runtime {

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
      effective_status.runtime_phase = runtime_phase_;
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
    effective_status.runtime_phase = runtime_phase_;
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
  RuntimeStatus status{request_id, ExecutionState::idle, "unknown request", 0, 0, 0, false, 0};
  status.runtime_phase = runtime_phase_;
  return status;
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
    RuntimeStatus status{request_id, ExecutionState::idle, "unknown request", 0, 0, 0, false, 0};
    status.runtime_phase = runtime_phase_;
    return status;
  };

  const auto status_changed = [this, &lookup_status, last_revision]() {
    return shutdown_ || lookup_status().revision > last_revision;
  };

  if (timeout.count() > 0) {
    status_cv_.wait_for(lock, timeout, status_changed);
  }
  return lookup_status();
}

}  // namespace rokae_xmate3_ros2::runtime
