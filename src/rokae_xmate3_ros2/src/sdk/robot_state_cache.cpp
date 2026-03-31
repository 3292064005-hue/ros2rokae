#include "robot_internal.hpp"

namespace rokae::ros2 {

int xMateRobot::getStateDataArray6(const std::string& fieldName, std::array<double, 6>& data) {
    std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
    auto it = impl_->state_cache_.find(fieldName);
    if (it == impl_->state_cache_.end()) {
        return -1;
    }
    try {
        data = std::any_cast<std::array<double, 6>>(it->second);
        return 0;
    } catch (const std::bad_any_cast&) {
        return -1;
    }
}

void xMateRobot::setEventWatcher(rokae::Event eventType, const rokae::EventCallback& callback, std::error_code& ec) {
    std::lock_guard<std::mutex> lock(impl_->event_mutex_);
    if (eventType == rokae::Event::moveExecution) {
        impl_->move_event_watcher_ = callback;
    } else {
        impl_->safety_event_watcher_ = callback;
    }
    ec.clear();
}

rokae::EventInfo xMateRobot::queryEventInfo(rokae::Event eventType, std::error_code& ec) {
    std::lock_guard<std::mutex> lock(impl_->event_mutex_);
    ec.clear();
    return eventType == rokae::Event::moveExecution ? impl_->last_move_event_ : impl_->last_safety_event_;
}

void xMateRobot::setMaxCacheSize(int number, std::error_code& ec) {
    if (number <= 0) {
        ec = std::make_error_code(std::errc::invalid_argument);
    } else {
        impl_->max_cache_size_ = number;
        ec.clear();
    }
}

std::error_code xMateRobot::lastErrorCode() noexcept {
    return impl_->last_error_code_;
}

} // namespace rokae::ros2
