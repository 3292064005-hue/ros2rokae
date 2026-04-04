#include "robot_internal.hpp"

#include "runtime/rt_subscription_plan.hpp"

namespace rokae::ros2 {
namespace {

bool is_valid_rt_mode(rokae::RtControllerMode mode) {
    switch (mode) {
        case rokae::RtControllerMode::jointPosition:
        case rokae::RtControllerMode::cartesianPosition:
        case rokae::RtControllerMode::jointImpedance:
        case rokae::RtControllerMode::cartesianImpedance:
        case rokae::RtControllerMode::torque:
            return true;
        default:
            return false;
    }
}

std::array<double, 6> posture_to_array(const rokae::CartesianPosition &pose) {
    return {pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz};
}

std::array<double, 6> diff_array6(const std::array<double, 6> &lhs,
                                  const std::array<double, 6> &rhs,
                                  double dt_sec) {
    std::array<double, 6> out{};
    if (dt_sec <= 0.0) {
        return out;
    }
    for (size_t i = 0; i < out.size(); ++i) {
        out[i] = (lhs[i] - rhs[i]) / dt_sec;
    }
    return out;
}

}  // namespace

/**
 * @brief Select the active realtime control mode for the compatibility RT lane.
 * @param mode Requested RT controller mode.
 * @param ec Output error code.
 * @throws None.
 * @note Boundary behavior: unsupported enum values are rejected before any transport call so wrapper
 *       code cannot silently coerce them into another RT mode.
 */
void xMateRobot::setRtControlMode(rokae::RtControllerMode mode, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }
    if (!is_valid_rt_mode(mode)) {
        ec = rokae::make_error_code(rokae::SdkError::invalid_argument);
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
    auto _last_error_scope = track_last_error(impl_, ec);
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

/**
 * @brief Arm RT state reception using a strict subscription plan.
 * @param interval Requested state interval.
 * @param fields Requested RT fields.
 * @details 1 ms requests are treated as strict in-loop RT subscriptions and only
 *          accept controller-native or locally-derived RT fields. Slower requests
 *          are armed as polled state streams and may include simulation-grade
 *          service-backed fields. Unsupported or downgraded plans are rejected
 *          instead of being silently normalized to a smaller field set.
 */
void xMateRobot::startReceiveRobotState(std::chrono::steady_clock::duration interval,
                                        const std::vector<std::string>& fields) {
    std::error_code ec;
    if (!impl_->connected_) {
        impl_->remember_last_error(std::make_error_code(std::errc::not_connected));
        RCLCPP_ERROR(impl_->node_->get_logger(), "RT state subscription requires an active connection");
        return;
    }
    {
        std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
        if (!impl_->state_fields_.empty()) {
            impl_->remember_last_error(std::make_error_code(std::errc::operation_in_progress));
            RCLCPP_ERROR(impl_->node_->get_logger(),
                         "RT state subscription is already active; call stopReceiveRobotState() before re-arming");
            return;
        }
    }
    const auto interval_ms = std::chrono::duration<double, std::milli>(interval).count();
    const bool use_state_data_in_loop = std::fabs(interval_ms - 1.0) <= 1e-6;
    const auto plan = rokae_xmate3_ros2::runtime::buildRtSubscriptionPlan(fields, interval, use_state_data_in_loop);
    {
        std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
        impl_->state_interval_ = interval;
        impl_->state_cache_.clear();
        impl_->has_previous_state_ = false;
        impl_->has_previous_pose_ = false;
        impl_->rt_state_plan_rejected_ = !plan.ok;
        if (!plan.ok) {
            impl_->state_fields_.clear();
            impl_->rt_state_plan_summary_ = std::string{"strict_rejected; "} + plan.summary();
            ec = std::make_error_code(std::errc::operation_not_supported);
            impl_->remember_last_error(ec);
            RCLCPP_ERROR(impl_->node_->get_logger(),
                         "RT state subscription rejected because the requested plan is not controller-grade: %s",
                         impl_->rt_state_plan_summary_.c_str());
            return;
        }
        impl_->state_fields_ = plan.accepted_fields;
        impl_->rt_state_plan_summary_ = std::string{"strict_ok; "} + plan.summary();
        impl_->remember_last_error({});
        RCLCPP_INFO(impl_->node_->get_logger(), "RT state subscription armed: %s", impl_->rt_state_plan_summary_.c_str());
    }

    const auto first_frame = updateRobotState(std::chrono::seconds(3));
    if (first_frame == 0U) {
        std::lock_guard<std::mutex> relock(impl_->state_cache_mutex_);
        impl_->state_fields_.clear();
        impl_->state_cache_.clear();
        impl_->has_previous_state_ = false;
        impl_->has_previous_pose_ = false;
        impl_->rt_state_plan_rejected_ = false;
        impl_->rt_state_plan_summary_ = "inactive_after_first_frame_failure";
        impl_->remember_last_error(std::make_error_code(std::errc::timed_out));
        RCLCPP_ERROR(impl_->node_->get_logger(),
                     "RT state subscription failed to receive the first frame within the 3s startup window");
    }
}

void xMateRobot::stopReceiveRobotState() noexcept {
    std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
    impl_->state_fields_.clear();
    impl_->state_cache_.clear();
    impl_->has_previous_state_ = false;
    impl_->has_previous_pose_ = false;
    impl_->rt_state_plan_rejected_ = false;
    impl_->rt_state_plan_summary_ = "inactive";
    impl_->remember_last_error({});
}

unsigned xMateRobot::updateRobotState(std::chrono::steady_clock::duration timeout) {
    std::vector<std::string> state_fields;
    std::chrono::steady_clock::duration configured_interval{std::chrono::milliseconds(1)};
    {
        std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
        if (impl_->rt_state_plan_rejected_) {
            RCLCPP_ERROR(impl_->node_->get_logger(),
                         "RT state update refused because the subscription plan was previously rejected: %s",
                         impl_->rt_state_plan_summary_.c_str());
            impl_->remember_last_error(std::make_error_code(std::errc::operation_not_supported));
            return 0;
        }
        if (impl_->state_fields_.empty()) {
            impl_->remember_last_error({});
            return 0;
        }
        state_fields = impl_->state_fields_;
        configured_interval = impl_->state_interval_;
    }

    const auto request_started = std::chrono::steady_clock::now();
    const bool enforce_timeout = timeout > std::chrono::steady_clock::duration::zero();
    const auto deadline = request_started + timeout;
    const auto remaining_time = [&]() {
        if (!enforce_timeout) {
            return std::chrono::seconds(30);
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            return std::chrono::nanoseconds::zero();
        }
        return std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now);
    };
    const auto timeout_expired = [&]() { return enforce_timeout && remaining_time() <= std::chrono::nanoseconds::zero(); };

    std::error_code ec;
    std::array<double, 6> joints{};
    std::array<double, 6> joint_vel{};
    std::array<double, 6> joint_tau{};

    const auto fetch_rt_joint_data = [&]() -> bool {
        if (!impl_->connected_ || !impl_->xmate3_rt_get_joint_data_client_) {
            ec = std::make_error_code(std::errc::not_connected);
            return false;
        }
        if (!impl_->wait_for_service(impl_->xmate3_rt_get_joint_data_client_, ec, remaining_time())) {
            return false;
        }
        auto request = std::make_shared<rokae_xmate3_ros2::srv::GetRtJointData::Request>();
        auto future = impl_->xmate3_rt_get_joint_data_client_->async_send_request(request);
        if (impl_->wait_for_future(future, remaining_time()) != rclcpp::FutureReturnCode::SUCCESS) {
            ec = std::make_error_code(std::errc::timed_out);
            return false;
        }
        const auto result = future.get();
        if (!result->success) {
            ec = std::make_error_code(result->error_code == 2002 ? std::errc::timed_out : std::errc::operation_not_permitted);
            return false;
        }
        for (size_t i = 0; i < 6; ++i) {
            joints[i] = result->joint_position[i];
            joint_vel[i] = result->joint_velocity[i];
            joint_tau[i] = result->joint_torque[i];
        }
        ec.clear();
        return true;
    };

    if (!fetch_rt_joint_data()) {
        if (timeout_expired()) {
            impl_->remember_last_error(ec ? ec : std::make_error_code(std::errc::timed_out));
            return 0;
        }
        impl_->remember_last_error(ec ? ec : std::make_error_code(std::errc::io_error));
        RCLCPP_ERROR(impl_->node_->get_logger(),
                     "RT joint-state update failed under strict RT semantics; no NRT fallback will be used");
        return 0;
    }

    const auto requires_pose_fields = std::any_of(state_fields.begin(), state_fields.end(), [](const std::string& field) {
        return field == rokae::RtSupportedFields::tcpPoseAbc_m ||
               field == rokae::RtSupportedFields::tcpPose_m ||
               field == rokae::RtSupportedFields::tcpPose_c ||
               field == rokae::RtSupportedFields::tcpVel_c ||
               field == rokae::RtSupportedFields::tcpAcc_c;
    });
    const auto requires_end_torque_fields = std::any_of(state_fields.begin(), state_fields.end(), [](const std::string& field) {
        return field == rokae::RtSupportedFields::tauExt_inBase ||
               field == rokae::RtSupportedFields::tauExt_inStiff;
    });

    std::array<double, 6> pose_abc{};
    std::array<double, 16> pose_matrix{};
    std::array<double, 6> end_torque{};
    if (requires_pose_fields) {
        auto pose_request = std::make_shared<rokae_xmate3_ros2::srv::GetCartPosture::Request>();
        pose_request->coordinate_type = static_cast<int32_t>(rokae::CoordinateType::flangeInBase);
        if (!impl_->wait_for_service(impl_->xmate3_robot_get_cart_posture_client_, ec, remaining_time())) {
            impl_->remember_last_error(ec ? ec : std::make_error_code(std::errc::timed_out));
            return 0;
        }
        auto pose_future = impl_->xmate3_robot_get_cart_posture_client_->async_send_request(pose_request);
        if (impl_->wait_for_future(pose_future, remaining_time()) != rclcpp::FutureReturnCode::SUCCESS) {
            impl_->remember_last_error(std::make_error_code(std::errc::timed_out));
            return 0;
        }
        const auto pose_result = pose_future.get();
        if (!pose_result->success) {
            impl_->remember_last_error(std::make_error_code(std::errc::operation_not_permitted));
            return 0;
        }
        rokae::CartesianPosition pose{pose_result->x, pose_result->y, pose_result->z,
                                      pose_result->rx, pose_result->ry, pose_result->rz};
        pose_abc = posture_to_array(pose);
        pose_matrix = rokae::Utils::postureToMatrix(pose);
    }

    if (requires_end_torque_fields) {
        impl_->ensureDynamicsClients();
        if (!impl_->wait_for_service(impl_->xmate3_cobot_get_end_torque_client_, ec, remaining_time())) {
            impl_->remember_last_error(ec ? ec : std::make_error_code(std::errc::timed_out));
            return 0;
        }
        auto end_torque_request = std::make_shared<rokae_xmate3_ros2::srv::GetEndEffectorTorque::Request>();
        auto end_torque_future = impl_->xmate3_cobot_get_end_torque_client_->async_send_request(end_torque_request);
        if (impl_->wait_for_future(end_torque_future, remaining_time()) != rclcpp::FutureReturnCode::SUCCESS) {
            impl_->remember_last_error(std::make_error_code(std::errc::timed_out));
            return 0;
        }
        const auto end_torque_result = end_torque_future.get();
        if (!end_torque_result->success) {
            impl_->remember_last_error(std::make_error_code(std::errc::operation_not_permitted));
            return 0;
        }
        end_torque = end_torque_result->end_torque;
    }

    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
    const double interval_dt = std::max(1e-6, std::chrono::duration<double>(configured_interval).count());
    const double derived_dt = impl_->has_previous_state_
                                ? std::max(1e-6, std::chrono::duration<double>(now - impl_->previous_state_time_).count())
                                : interval_dt;
    if (impl_->has_previous_state_) {
        impl_->rt_state_max_gap_ms_ = std::max(impl_->rt_state_max_gap_ms_, derived_dt * 1000.0);
        if (derived_dt > interval_dt * 2.0) {
            ++impl_->rt_state_late_cycle_count_;
        }
    }

    std::array<double, 6> joint_acc{};
    std::array<double, 6> tau_vel{};
    std::array<double, 6> tcp_vel{};
    std::array<double, 6> tcp_acc{};
    if (impl_->has_previous_state_) {
        for (size_t i = 0; i < 6; ++i) {
            joint_acc[i] = (joint_vel[i] - impl_->previous_joint_velocity_[i]) / derived_dt;
            tau_vel[i] = (joint_tau[i] - impl_->previous_joint_torque_[i]) / derived_dt;
        }
    }
    if (requires_pose_fields && impl_->has_previous_pose_) {
        tcp_vel = diff_array6(pose_abc, impl_->previous_pose_abc_, derived_dt);
        tcp_acc = diff_array6(tcp_vel, impl_->previous_pose_velocity_, derived_dt);
    }

    impl_->previous_joint_velocity_ = joint_vel;
    impl_->previous_joint_torque_ = joint_tau;
    if (requires_pose_fields) {
        impl_->previous_pose_abc_ = pose_abc;
        impl_->previous_pose_velocity_ = tcp_vel;
        impl_->has_previous_pose_ = true;
    }
    impl_->previous_state_time_ = now;
    impl_->rt_state_last_update_time_ = now;
    impl_->has_previous_state_ = true;
    impl_->state_cache_.clear();

    const std::array<double, 6> zero6{};
    const double sample_period_s = derived_dt;
    const auto allowed_age = std::max(std::chrono::duration_cast<std::chrono::milliseconds>(impl_->state_interval_) * 2,
                                      std::chrono::milliseconds(8));
    const bool sample_fresh = (std::chrono::steady_clock::now() - now) <= allowed_age;
    for (const auto &field : state_fields) {
        if (field == rokae::RtSupportedFields::jointPos_m || field == rokae::RtSupportedFields::jointPos_c ||
            field == rokae::RtSupportedFields::theta_m) {
            impl_->state_cache_[field] = joints;
        } else if (field == rokae::RtSupportedFields::jointVel_m || field == rokae::RtSupportedFields::jointVel_c ||
                   field == rokae::RtSupportedFields::thetaVel_m) {
            impl_->state_cache_[field] = joint_vel;
        } else if (field == rokae::RtSupportedFields::jointAcc_c) {
            impl_->state_cache_[field] = joint_acc;
        } else if (field == rokae::RtSupportedFields::tau_m || field == rokae::RtSupportedFields::tau_c ||
                   field == rokae::RtSupportedFields::tauFiltered_m || field == rokae::RtSupportedFields::motorTau ||
                   field == rokae::RtSupportedFields::motorTauFiltered) {
            impl_->state_cache_[field] = joint_tau;
        } else if (field == rokae::RtSupportedFields::tcpPoseAbc_m) {
            impl_->state_cache_[field] = pose_abc;
        } else if (field == rokae::RtSupportedFields::tcpPose_m || field == rokae::RtSupportedFields::tcpPose_c) {
            impl_->state_cache_[field] = pose_matrix;
        } else if (field == rokae::RtSupportedFields::tcpVel_c) {
            impl_->state_cache_[field] = tcp_vel;
        } else if (field == rokae::RtSupportedFields::tcpAcc_c) {
            impl_->state_cache_[field] = tcp_acc;
        } else if (field == rokae::RtSupportedFields::tauVel_c) {
            impl_->state_cache_[field] = tau_vel;
        } else if (field == rokae::RtSupportedFields::tauExt_inBase || field == rokae::RtSupportedFields::tauExt_inStiff) {
            impl_->state_cache_[field] = end_torque;
        } else if (field == rokae::RtCompatFields::samplePeriod_s) {
            impl_->state_cache_[field] = sample_period_s;
        } else if (field == rokae::RtCompatFields::sampleFresh) {
            impl_->state_cache_[field] = sample_fresh;
        } else {
            impl_->state_cache_[field] = zero6;
        }
    }
    impl_->remember_last_error({});
    return static_cast<unsigned>(impl_->state_cache_.size());
}

/**
 * @brief Return the shared RT controller weak handle for the xMate6 compatibility lane.
 * @return Weak reference to the opaque RT controller instance.
 * @throws No exception.
 * @note The controller object is cached on the robot instance and is not destroyed just because the caller
 *       switches back to NRT or disconnects. Subsequent use after those lifecycle transitions may still fail
 *       through the normal RT exception / error paths, matching the official SDK ownership model.
 */
std::weak_ptr<void> xMateRobot::getRtMotionController() {
    if (!impl_->rt_controller_) {
        impl_->rt_controller_ = std::make_shared<int>(0);
    }
    return impl_->rt_controller_;
}

}  // namespace rokae::ros2
