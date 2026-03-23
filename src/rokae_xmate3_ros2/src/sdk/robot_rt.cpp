#include "robot_internal.hpp"

namespace rokae::ros2 {

void xMateRobot::setRtControlMode(rokae::RtControllerMode mode, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }
    if (!impl_->wait_for_service(impl_->xmate3_rt_set_control_mode_client_, ec)) {
        return;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetRtControlMode::Request>();
    request->mode = static_cast<int32_t>(mode);
    auto future = impl_->xmate3_rt_set_control_mode_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置实时控制模式失败: %s", result->error_msg.c_str());
        return;
    }
    ec.clear();
}

bool xMateRobot::getRtJointData(std::array<double, 6>& position,
                                std::array<double, 6>& velocity,
                                std::array<double, 6>& torque,
                                std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_rt_get_joint_data_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetRtJointData::Request>();
    auto future = impl_->xmate3_rt_get_joint_data_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }
    for (size_t i = 0; i < 6; ++i) {
        position[i] = result->joint_position[i];
        velocity[i] = result->joint_velocity[i];
        torque[i] = result->joint_torque[i];
    }
    ec.clear();
    return true;
}

void xMateRobot::startReceiveRobotState(std::chrono::steady_clock::duration interval,
                                        const std::vector<std::string>& fields) {
    std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
    impl_->state_fields_ = fields;
    impl_->state_interval_ = interval;
    impl_->state_cache_.clear();
    impl_->has_previous_state_ = false;
}

void xMateRobot::stopReceiveRobotState() noexcept {
    std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
    impl_->state_fields_.clear();
    impl_->state_cache_.clear();
    impl_->has_previous_state_ = false;
}

unsigned xMateRobot::updateRobotState(std::chrono::steady_clock::duration timeout) {
    (void)timeout;
    std::error_code ec;
    std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
    if (impl_->state_fields_.empty()) {
        return 0;
    }
    auto joints = jointPos(ec);
    if (!ec) {
        auto joint_vel = jointVel(ec);
        auto joint_tau = jointTorque(ec);
        for (const auto& field : impl_->state_fields_) {
            if (field == rokae::RtSupportedFields::jointPos_m) {
                impl_->state_cache_[field] = joints;
            } else if (field == rokae::RtSupportedFields::jointVel_m) {
                impl_->state_cache_[field] = joint_vel;
            } else if (field == rokae::RtSupportedFields::tau_m) {
                impl_->state_cache_[field] = joint_tau;
            }
        }
    }
    return static_cast<unsigned>(impl_->state_cache_.size());
}

std::weak_ptr<void> xMateRobot::getRtMotionController() {
    if (!impl_->rt_controller_) {
        impl_->rt_controller_ = std::make_shared<int>(0);
    }
    return impl_->rt_controller_;
}

}  // namespace rokae::ros2
