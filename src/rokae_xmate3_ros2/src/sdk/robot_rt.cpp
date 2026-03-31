#include "robot_internal.hpp"

#include "runtime/rt_subscription_plan.hpp"

namespace rokae::ros2 {
namespace {

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
    const auto plan = rokae_xmate3_ros2::runtime::buildRtSubscriptionPlan(fields, interval, true);
    std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
    impl_->state_fields_ = plan.accepted_fields;
    impl_->state_interval_ = interval;
    impl_->state_cache_.clear();
    impl_->has_previous_state_ = false;
    impl_->has_previous_pose_ = false;
    impl_->rt_state_plan_summary_ = plan.summary();
    if (!plan.ok) {
        RCLCPP_WARN(impl_->node_->get_logger(), "RT state subscription plan downgraded: %s", impl_->rt_state_plan_summary_.c_str());
    } else {
        RCLCPP_INFO(impl_->node_->get_logger(), "RT state subscription armed: %s", impl_->rt_state_plan_summary_.c_str());
    }
}

void xMateRobot::stopReceiveRobotState() noexcept {
    std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
    impl_->state_fields_.clear();
    impl_->state_cache_.clear();
    impl_->has_previous_state_ = false;
    impl_->has_previous_pose_ = false;
    impl_->rt_state_plan_summary_ = "inactive";
}

unsigned xMateRobot::updateRobotState(std::chrono::steady_clock::duration timeout) {
    (void)timeout;
    std::error_code ec;
    std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
    if (impl_->state_fields_.empty()) {
        return 0;
    }

    std::array<double, 6> joints{};
    std::array<double, 6> joint_vel{};
    std::array<double, 6> joint_tau{};
    if (!getRtJointData(joints, joint_vel, joint_tau, ec)) {
        ec.clear();
        joints = jointPos(ec);
        if (ec) {
            return 0;
        }
        joint_vel = jointVel(ec);
        if (ec) {
            return 0;
        }
        joint_tau = jointTorques(ec);
        if (ec) {
            return 0;
        }
    }

    const auto pose = cartPosture(rokae::CoordinateType::flangeInBase, ec);
    if (ec) {
        return 0;
    }
    const auto pose_abc = posture_to_array(pose);
    const auto pose_matrix = rokae::Utils::postureToMatrix(pose);
    const auto end_torque = getEndEffectorTorque(ec);
    if (ec) {
        return 0;
    }

    const auto now = std::chrono::steady_clock::now();
    const double configured_dt = std::max(1e-6, std::chrono::duration<double>(impl_->state_interval_).count());
    const double derived_dt = impl_->has_previous_state_
                                ? std::max(1e-6, std::chrono::duration<double>(now - impl_->previous_state_time_).count())
                                : configured_dt;
    if (impl_->has_previous_state_) {
        impl_->rt_state_max_gap_ms_ = std::max(impl_->rt_state_max_gap_ms_, derived_dt * 1000.0);
        if (derived_dt > configured_dt * 2.0) {
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
    if (impl_->has_previous_pose_) {
        tcp_vel = diff_array6(pose_abc, impl_->previous_pose_abc_, derived_dt);
        tcp_acc = diff_array6(tcp_vel, impl_->previous_pose_velocity_, derived_dt);
    }

    impl_->previous_joint_velocity_ = joint_vel;
    impl_->previous_joint_torque_ = joint_tau;
    impl_->previous_pose_abc_ = pose_abc;
    impl_->previous_pose_velocity_ = tcp_vel;
    impl_->previous_state_time_ = now;
    impl_->has_previous_state_ = true;
    impl_->has_previous_pose_ = true;
    impl_->state_cache_.clear();

    const std::array<double, 6> zero6{};
    for (const auto &field : impl_->state_fields_) {
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
        } else if (field == rokae::RtSupportedFields::elbow_m || field == rokae::RtSupportedFields::elbow_c ||
                   field == rokae::RtSupportedFields::elbowVel_c || field == rokae::RtSupportedFields::elbowAcc_c) {
            impl_->state_cache_[field] = 0.0;
        } else if (field == rokae::RtSupportedFields::tauVel_c) {
            impl_->state_cache_[field] = tau_vel;
        } else if (field == rokae::RtSupportedFields::tauExt_inBase || field == rokae::RtSupportedFields::tauExt_inStiff) {
            impl_->state_cache_[field] = end_torque;
        } else {
            impl_->state_cache_[field] = zero6;
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
