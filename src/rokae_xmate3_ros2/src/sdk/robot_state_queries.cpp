#include "robot_internal.hpp"

namespace rokae::ros2 {

std::array<double, 6> xMateRobot::jointPos(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    std::array<double, 6> pos = {0.0};
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return pos;
    }

    impl_->pump_callbacks();
    {
        std::lock_guard<std::mutex> lock(impl_->state_mutex_);
        if (impl_->last_joint_state_.position.size() >= pos.size()) {
            std::copy_n(impl_->last_joint_state_.position.begin(), pos.size(), pos.begin());
            ec.clear();
            return pos;
        }
    }

    if (impl_->refreshRuntimeStateSnapshot(ec)) {
        std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
        pos = impl_->runtime_state_snapshot_.joint_position;
        ec.clear();
        return pos;
    }

    if (!impl_->wait_for_service(impl_->xmate3_robot_get_joint_pos_client_, ec)) {
        return pos;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetJointPos::Request>();
    auto future = impl_->xmate3_robot_get_joint_pos_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return pos;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return pos;
    }
    std::copy(result->joint_positions.begin(), result->joint_positions.end(), pos.begin());
    ec.clear();
    return pos;
}


// 获取关节速度

std::array<double, 6> xMateRobot::jointVel(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    std::array<double, 6> vel = {0.0};
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return vel;
    }

    impl_->pump_callbacks();
    {
        std::lock_guard<std::mutex> lock(impl_->state_mutex_);
        if (impl_->last_joint_state_.velocity.size() >= vel.size()) {
            std::copy_n(impl_->last_joint_state_.velocity.begin(), vel.size(), vel.begin());
            ec.clear();
            return vel;
        }
    }

    if (impl_->refreshRuntimeStateSnapshot(ec)) {
        std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
        vel = impl_->runtime_state_snapshot_.joint_velocity;
        ec.clear();
        return vel;
    }

    if (!impl_->wait_for_service(impl_->xmate3_robot_get_joint_vel_client_, ec)) {
        return vel;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetJointVel::Request>();
    auto future = impl_->xmate3_robot_get_joint_vel_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return vel;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return vel;
    }

    std::copy(result->joint_vel.begin(), result->joint_vel.end(), vel.begin());
    ec.clear();
    return vel;
}

// 获取关节扭矩

std::array<double, 6> xMateRobot::jointTorques(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    std::array<double, 6> torque = {0.0};
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return torque;
    }

    impl_->pump_callbacks();
    {
        std::lock_guard<std::mutex> lock(impl_->state_mutex_);
        if (impl_->last_joint_state_.effort.size() >= torque.size()) {
            std::copy_n(impl_->last_joint_state_.effort.begin(), torque.size(), torque.begin());
            ec.clear();
            return torque;
        }
    }

    if (impl_->refreshRuntimeStateSnapshot(ec)) {
        std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
        torque = impl_->runtime_state_snapshot_.joint_torque;
        ec.clear();
        return torque;
    }

    if (!impl_->wait_for_service(impl_->xmate3_robot_get_joint_torque_client_, ec)) {
        return torque;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetJointTorques::Request>();
    auto future = impl_->xmate3_robot_get_joint_torque_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return torque;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return torque;
    }

    std::copy(result->joint_torque.begin(), result->joint_torque.end(), torque.begin());
    ec.clear();
    return torque;
}

std::array<double, 6> xMateRobot::jointTorque(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    return jointTorques(ec);
}

// 获取末端位姿

std::array<double, 6> xMateRobot::posture(rokae::CoordinateType ct, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    std::array<double, 6> pose = {0.0};
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return pose;
    }

    rokae::JointPosition joints(6, 0.0);
    const auto cached_joints = jointPos(ec);
    if (ec) {
        return pose;
    }
    joints.joints.assign(cached_joints.begin(), cached_joints.end());
    const auto fk_pose = calcFk(joints, ec);
    if (ec) {
        return pose;
    }

    pose = {fk_pose.x, fk_pose.y, fk_pose.z, fk_pose.rx, fk_pose.ry, fk_pose.rz};
    if (ct == rokae::CoordinateType::endInRef) {
        const auto active_toolset = toolset(ec);
        if (ec) {
            return pose;
        }
        const std::vector<double> flange_pose(pose.begin(), pose.end());
        const std::vector<double> tool_pose(active_toolset.tool_pose.begin(), active_toolset.tool_pose.end());
        const std::vector<double> wobj_pose(active_toolset.wobj_pose.begin(), active_toolset.wobj_pose.end());
        const auto transformed =
            rokae_xmate3_ros2::runtime::pose_utils::convertFlangeInBaseToEndInRef(flange_pose, tool_pose, wobj_pose);
        std::copy_n(transformed.begin(), std::min<std::size_t>(transformed.size(), pose.size()), pose.begin());
    }
    ec.clear();
    return pose;
}

// 获取笛卡尔位姿结构体

rokae::CartesianPosition xMateRobot::cartPosture(rokae::CoordinateType ct, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    rokae::CartesianPosition cart_pose;
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return cart_pose;
    }

    const auto pose = posture(ct, ec);
    if (ec) {
        return cart_pose;
    }

    cart_pose.x = pose[0];
    cart_pose.y = pose[1];
    cart_pose.z = pose[2];
    cart_pose.rx = pose[3];
    cart_pose.ry = pose[4];
    cart_pose.rz = pose[5];
    ec.clear();
    return cart_pose;
}

// 获取基坐标系

std::array<double, 6> xMateRobot::baseFrame(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    std::array<double, 6> frame = {0.0};
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return frame;
    }

    if (impl_->refreshRuntimeStateSnapshot(ec)) {
        std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
        frame = impl_->runtime_state_snapshot_.base_frame;
        ec.clear();
        return frame;
    }

    if (!impl_->wait_for_service(impl_->xmate3_robot_get_base_frame_client_, ec)) {
        return frame;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetBaseFrame::Request>();
    auto future = impl_->xmate3_robot_get_base_frame_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return frame;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return frame;
    }

    std::copy(result->base_frame.begin(), result->base_frame.end(), frame.begin());
    ec.clear();
    return frame;
}

// 逆解计算

} // namespace rokae::ros2
