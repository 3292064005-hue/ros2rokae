#include "robot_internal.hpp"

namespace rokae::ros2 {
namespace {

template <typename T>
int readStateCacheValue(std::mutex &mutex,
                        const std::unordered_map<std::string, std::any> &state_cache,
                        const std::string &fieldName,
                        T &data) {
    std::lock_guard<std::mutex> lock(mutex);
    const auto it = state_cache.find(fieldName);
    if (it == state_cache.end()) {
        return -1;
    }
    try {
        data = std::any_cast<T>(it->second);
        return 0;
    } catch (const std::bad_any_cast&) {
        return -1;
    }
}

}  // namespace

bool xMateRobot::Impl::refreshRuntimeStateSnapshot(std::error_code& ec,
                                                  bool force,
                                                  std::chrono::milliseconds max_age) {
    if (!connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_cache_mutex_);
        if (!force && runtime_state_snapshot_valid_) {
            const auto age = std::chrono::steady_clock::now() - runtime_state_snapshot_stamp_;
            if (age <= max_age) {
                ec.clear();
                return true;
            }
        }
    }

    if (!wait_for_service(xmate3_internal_get_runtime_state_snapshot_client_, ec)) {
        return false;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetRuntimeStateSnapshot::Request>();
    auto future = xmate3_internal_get_runtime_state_snapshot_client_->async_send_request(request);
    if (wait_for_future(future, std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }

    const auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_cache_mutex_);
        runtime_state_snapshot_ = *result;
        runtime_state_snapshot_stamp_ = std::chrono::steady_clock::now();
        runtime_state_snapshot_valid_ = true;
    }
    ec.clear();
    return true;
}

void xMateRobot::Impl::clearRuntimeStateSnapshotCache() noexcept {
    std::lock_guard<std::mutex> lock(state_cache_mutex_);
    runtime_state_snapshot_valid_ = false;
    runtime_state_snapshot_stamp_ = std::chrono::steady_clock::time_point{};
    runtime_state_snapshot_ = rokae_xmate3_ros2::srv::GetRuntimeStateSnapshot::Response();
}

int xMateRobot::getStateDataArray6(const std::string& fieldName, std::array<double, 6>& data) {
    return readStateCacheValue(impl_->state_cache_mutex_, impl_->state_cache_, fieldName, data);
}

int xMateRobot::getStateDataArray3(const std::string& fieldName, std::array<double, 3>& data) {
    return readStateCacheValue(impl_->state_cache_mutex_, impl_->state_cache_, fieldName, data);
}

int xMateRobot::getStateDataMatrix16(const std::string& fieldName, std::array<double, 16>& data) {
    return readStateCacheValue(impl_->state_cache_mutex_, impl_->state_cache_, fieldName, data);
}

int xMateRobot::getStateDataScalarDouble(const std::string& fieldName, double& data) {
    return readStateCacheValue(impl_->state_cache_mutex_, impl_->state_cache_, fieldName, data);
}

int xMateRobot::getStateDataBool(const std::string& fieldName, bool& data) {
    if (fieldName == rokae::RtCompatFields::sampleFresh) {
        std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
        if (impl_->rt_state_last_update_time_ == std::chrono::steady_clock::time_point{}) {
            return -1;
        }
        const auto allowed_age = std::max(std::chrono::duration_cast<std::chrono::milliseconds>(impl_->state_interval_) * 2,
                                          std::chrono::milliseconds(8));
        data = (std::chrono::steady_clock::now() - impl_->rt_state_last_update_time_) <= allowed_age;
        return 0;
    }
    return readStateCacheValue(impl_->state_cache_mutex_, impl_->state_cache_, fieldName, data);
}

void xMateRobot::setEventWatcher(rokae::Event eventType, const rokae::EventCallback& callback, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    std::lock_guard<std::mutex> lock(impl_->event_mutex_);
    if (eventType == rokae::Event::moveExecution) {
        impl_->move_event_watcher_ = callback;
    } else {
        impl_->safety_event_watcher_ = callback;
    }
    ec.clear();
}

rokae::EventInfo xMateRobot::queryEventInfo(rokae::Event eventType, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    std::lock_guard<std::mutex> lock(impl_->event_mutex_);
    ec.clear();
    return eventType == rokae::Event::moveExecution ? impl_->last_move_event_ : impl_->last_safety_event_;
}

void xMateRobot::setMaxCacheSize(int number, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
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
