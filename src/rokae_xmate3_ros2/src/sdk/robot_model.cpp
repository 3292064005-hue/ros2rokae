#include <cfloat>
#include "robot_internal.hpp"

namespace rokae::ros2 {

void xMateRobot::setAvoidSingularity(bool enable, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    (void)enable;
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }
    ec = rokae::make_error_code(rokae::SdkError::not_implemented);
}

bool xMateRobot::getAvoidSingularity(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    ec = rokae::make_error_code(rokae::SdkError::not_implemented);
    return false;
}

std::array<double, 6> xMateRobot::getEndEffectorTorque(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    impl_->ensureDynamicsClients();
    std::array<double, 6> wrench{};
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return wrench;
    }
    if (!impl_->wait_for_service(impl_->xmate3_cobot_get_end_torque_client_, ec)) {
        return wrench;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetEndEffectorTorque::Request>();
    auto future = impl_->xmate3_cobot_get_end_torque_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return wrench;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return wrench;
    }
    for (size_t i = 0; i < 6; ++i) {
        wrench[i] = result->end_torque[i];
    }
    ec.clear();
    return wrench;
}

std::array<double, 6> xMateRobot::getEndTorque(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    return getEndEffectorTorque(ec);
}

void xMateRobot::getEndTorque(rokae::FrameType ref_type,
                              std::array<double, 6>& joint_torque_measured,
                              std::array<double, 6>& external_torque_measured,
                              std::array<double, 3>& cart_torque,
                              std::array<double, 3>& cart_force,
                              std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    impl_->ensureDynamicsClients();
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }
    if (impl_->xmate3_cobot_get_end_wrench_client_ &&
        impl_->wait_for_service(impl_->xmate3_cobot_get_end_wrench_client_, ec)) {
        auto request = std::make_shared<rokae_xmate3_ros2::srv::GetEndWrench::Request>();
        request->ref_type = static_cast<int>(ref_type);
        auto future = impl_->xmate3_cobot_get_end_wrench_client_->async_send_request(request);
        if (impl_->wait_for_future(future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future.get();
            if (result->success) {
                for (size_t i = 0; i < 6; ++i) {
                    joint_torque_measured[i] = result->joint_torque_measured[i];
                    external_torque_measured[i] = result->external_joint_torque[i];
                }
                cart_torque = {result->cart_torque[0], result->cart_torque[1], result->cart_torque[2]};
                cart_force = {result->cart_force[0], result->cart_force[1], result->cart_force[2]};
                ec.clear();
                return;
            }
        }
    }
    joint_torque_measured = jointTorques(ec);
    if (ec) {
        return;
    }
    const auto wrench = getEndEffectorTorque(ec);
    if (ec) {
        return;
    }
    external_torque_measured.fill(0.0);
    cart_force = {wrench[0], wrench[1], wrench[2]};
    cart_torque = {wrench[3], wrench[4], wrench[5]};
    ec.clear();
}

bool xMateRobot::calcJointTorque(const std::array<double, 6>& joint_pos,
                                 const std::array<double, 6>& joint_vel,
                                 const std::array<double, 6>& joint_acc,
                                 const std::array<double, 6>& external_force,
                                 std::array<double, 6>& joint_torque,
                                 std::array<double, 6>& gravity_torque,
                                 std::array<double, 6>& coriolis_torque,
                                 std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    impl_->ensureDynamicsClients();
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_dyn_calc_joint_torque_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::CalcJointTorque::Request>();
    request->joint_pos = joint_pos;
    request->joint_vel = joint_vel;
    request->joint_acc = joint_acc;
    request->external_force = external_force;
    auto future = impl_->xmate3_dyn_calc_joint_torque_client_->async_send_request(request);
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
        joint_torque[i] = result->joint_torque[i];
        gravity_torque[i] = result->gravity_torque[i];
        coriolis_torque[i] = result->coriolis_torque[i];
    }
    ec.clear();
    return true;
}

bool xMateRobot::generateSTrajectory(const std::array<double, 6>& start_joint_pos,
                                     const std::array<double, 6>& target_joint_pos,
                                     std::vector<std::array<double, 6>>& trajectory_points,
                                     double& total_time,
                                     std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    impl_->ensureDynamicsClients();
    trajectory_points.clear();
    total_time = 0.0;
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_dyn_generate_s_trajectory_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::GenerateSTrajectory::Request>();
    request->start_joint_pos = start_joint_pos;
    request->target_joint_pos = target_joint_pos;
    request->max_velocity = 1.0;
    request->max_acceleration = 0.5;
    request->blend_radius = 0.01;
    request->is_cartesian = false;
    auto future = impl_->xmate3_dyn_generate_s_trajectory_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }
    total_time = result->total_time;
    trajectory_points.reserve(result->trajectory_points.size());
    for (const auto& point : result->trajectory_points) {
        trajectory_points.push_back(point.pos);
    }
    ec.clear();
    return true;
}

bool xMateRobot::mapCartesianToJointTorque(const std::array<double, 6>& cart_force,
                                           const std::array<double, 6>& joint_pos,
                                           std::array<double, 6>& joint_torque,
                                           std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    impl_->ensureDynamicsClients();
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_dyn_map_cartesian_to_joint_torque_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::MapCartesianToJointTorque::Request>();
    request->cart_force = cart_force;
    request->joint_pos = joint_pos;
    auto future = impl_->xmate3_dyn_map_cartesian_to_joint_torque_client_->async_send_request(request);
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
        joint_torque[i] = result->joint_torque[i];
    }
    ec.clear();
    return true;
}

std::shared_ptr<void> xMateRobot::model() {
    if (!impl_->model_) {
        impl_->model_ = std::make_shared<rokae::ros2::XMateModel>(*this);
    }
    return impl_->model_;
}

bool xMateRobot::getSoftLimit(std::array<double[2], 6>& limits, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    std::array<std::array<double, 2>, 6> tmp{};
    bool enabled = getSoftLimit(tmp, ec);
    if (!ec) {
        for (size_t i = 0; i < 6; ++i) {
            limits[i][0] = tmp[i][0];
            limits[i][1] = tmp[i][1];
        }
    }
    return enabled;
}

void xMateRobot::setSoftLimit(bool enable, std::error_code& ec, const std::array<double[2], 6>& limits) {
    auto _last_error_scope = track_last_error(impl_, ec);
    std::array<std::array<double, 2>, 6> tmp{};
    bool use_existing_limits = enable;
    for (size_t i = 0; i < 6; ++i) {
        tmp[i][0] = limits[i][0];
        tmp[i][1] = limits[i][1];
        const bool axis_uses_sentinel = (limits[i][0] == DBL_MAX && limits[i][1] == DBL_MAX);
        use_existing_limits = use_existing_limits && axis_uses_sentinel;
    }
    if (use_existing_limits) {
        std::array<std::array<double, 2>, 6> current_limits{};
        const bool current_enabled = getSoftLimit(current_limits, ec);
        if (ec) {
            return;
        }
        (void)current_enabled;
        tmp = current_limits;
    }
    setSoftLimit(enable, ec, tmp);
}

rokae::FrameCalibrationResult xMateRobot::calibrateFrame(rokae::FrameType type,
                                                          const std::vector<std::array<double, 6>>& points,
                                                          bool is_held,
                                                          std::error_code& ec,
                                                          const std::array<double, 3>& base_aux) {
    auto _last_error_scope = track_last_error(impl_, ec);
    (void)type;
    (void)points;
    (void)is_held;
    (void)base_aux;
    ec = std::make_error_code(std::errc::function_not_supported);
    return rokae::FrameCalibrationResult{};
}

} // namespace rokae::ros2
