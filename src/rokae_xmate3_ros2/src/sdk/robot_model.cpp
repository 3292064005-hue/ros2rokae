#include "robot_internal.hpp"

namespace rokae::ros2 {

void xMateRobot::setAvoidSingularity(bool enable, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }
    if (!impl_->wait_for_service(impl_->xmate3_cobot_set_avoid_singularity_client_, ec)) {
        return;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetAvoidSingularity::Request>();
    request->enable = enable;
    auto future = impl_->xmate3_cobot_set_avoid_singularity_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return;
    }
    ec.clear();
}

bool xMateRobot::getAvoidSingularity(std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_cobot_get_avoid_singularity_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetAvoidSingularity::Request>();
    auto future = impl_->xmate3_cobot_get_avoid_singularity_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }
    ec.clear();
    return result->enabled;
}

std::array<double, 6> xMateRobot::getEndTorque(std::error_code& ec) {
    std::array<double, 6> wrench{};
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return wrench;
    }
    if (!impl_->wait_for_service(impl_->xmate3_cobot_get_end_torque_client_, ec)) {
        return wrench;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetEndTorque::Request>();
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

bool xMateRobot::calcJointTorque(const std::array<double, 6>& joint_pos,
                                 const std::array<double, 6>& joint_vel,
                                 const std::array<double, 6>& joint_acc,
                                 const std::array<double, 6>& external_force,
                                 std::array<double, 6>& joint_torque,
                                 std::array<double, 6>& gravity_torque,
                                 std::array<double, 6>& coriolis_torque,
                                 std::error_code& ec) {
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

// 模型和实时控制器访问（占位实现）
std::shared_ptr<void> xMateRobot::model() {
    if (!impl_->model_) {
        impl_->model_ = std::make_shared<rokae::ros2::XMateModel>(*this);
    }
    return impl_->model_;
}

// 软限位SDK兼容版本
bool xMateRobot::getSoftLimit(std::array<double[2], 6>& limits, std::error_code& ec) {
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
    std::array<std::array<double, 2>, 6> tmp{};
    for (size_t i = 0; i < 6; ++i) {
        tmp[i][0] = limits[i][0];
        tmp[i][1] = limits[i][1];
    }
    setSoftLimit(enable, ec, tmp);
}

// 坐标系标定 - 简化实现
rokae::FrameCalibrationResult xMateRobot::calibrateFrame(rokae::FrameType type,
                                                          const std::vector<std::array<double, 6>>& points,
                                                          bool is_held,
                                                          std::error_code& ec,
                                                          const std::array<double, 3>& base_aux) {
    rokae::FrameCalibrationResult result{};
    (void)is_held;
    (void)base_aux;

    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return result;
    }
    if (points.size() < 3) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return result;
    }

    Eigen::Vector3d mean_translation = Eigen::Vector3d::Zero();
    Eigen::Vector4d quaternion_accumulator = Eigen::Vector4d::Zero();
    for (const auto &point : points) {
        mean_translation += Eigen::Vector3d(point[0], point[1], point[2]);
        const auto quat = rokae_xmate3_ros2::runtime::pose_utils::rpyToQuaternion(point[3], point[4], point[5]);
        quaternion_accumulator += Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
    }
    mean_translation /= static_cast<double>(points.size());
    if (quaternion_accumulator.norm() < 1e-9) {
        quaternion_accumulator = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    }
    quaternion_accumulator.normalize();
    const Eigen::Quaterniond average_quaternion(
        quaternion_accumulator[0], quaternion_accumulator[1], quaternion_accumulator[2], quaternion_accumulator[3]);
    Eigen::Isometry3d calibrated_transform = Eigen::Isometry3d::Identity();
    calibrated_transform.translation() = mean_translation;
    calibrated_transform.linear() = average_quaternion.toRotationMatrix();
    const auto calibrated_pose = rokae_xmate3_ros2::runtime::pose_utils::isometryToPose(calibrated_transform);
    std::array<double, 6> calibrated_frame{};
    for (size_t i = 0; i < calibrated_frame.size() && i < calibrated_pose.size(); ++i) {
        calibrated_frame[i] = calibrated_pose[i];
    }
    result.frame = rokae::Frame(calibrated_frame);

    double translation_error = 0.0;
    double orientation_error = 0.0;
    for (const auto &point : points) {
        const Eigen::Vector3d translation(point[0], point[1], point[2]);
        translation_error += (translation - mean_translation).norm();
        orientation_error += rokae_xmate3_ros2::runtime::pose_utils::angularDistance(
            std::vector<double>(point.begin(), point.end()), calibrated_pose);
    }
    translation_error /= static_cast<double>(points.size());
    orientation_error /= static_cast<double>(points.size());

    result.success = true;
    result.errors = {translation_error, orientation_error, 0.0};
    ec.clear();

    // 更新缓存
    {
        std::lock_guard<std::mutex> lock(impl_->state_mutex_);
        if (type == rokae::FrameType::tool) {
            impl_->toolset_cache_.end = result.frame;
        } else if (type == rokae::FrameType::wobj || type == rokae::FrameType::base) {
            impl_->toolset_cache_.ref = result.frame;
        }
    }

    return result;
}


} // namespace rokae::ros2
