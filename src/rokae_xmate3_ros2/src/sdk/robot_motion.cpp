#include "robot_internal.hpp"

namespace rokae::ros2 {
namespace {

constexpr size_t kMoveAppendMaxCount = 100;
constexpr size_t kExecuteCommandMaxCount = 1000;

}

void xMateRobot::setMotionControlMode(rokae::MotionControlMode mode, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_motion_set_control_mode_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetMotionControlMode::Request>();
    switch (mode) {
        case rokae::MotionControlMode::Idle:
        case rokae::MotionControlMode::NrtCommand:
            request->mode = 0;
            break;
        case rokae::MotionControlMode::RtCommand:
            request->mode = 1;
            break;
        case rokae::MotionControlMode::NrtRLTask:
            request->mode = 2;
            break;
        default:
            request->mode = 0;
            break;
    }

    auto future = impl_->xmate3_motion_set_control_mode_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置运动控制模式失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "运动控制模式设置成功");
}

// 重置运动缓存
void xMateRobot::moveReset(std::error_code& ec) {
    // 首先清理本地缓存
    impl_->clearCache();

    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_motion_reset_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::MoveReset::Request>();
    auto future = impl_->xmate3_motion_reset_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "重置运动缓存失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "运动缓存重置成功");
}

// 开始运动
void xMateRobot::moveStart(std::error_code& ec) {
    // 首先发送缓存命令
    if (!impl_->flushCachedCommands(ec)) {
        // 失败时ec已经设置
        return;
    }

    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_motion_start_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::MoveStart::Request>();
    auto future = impl_->xmate3_motion_start_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "启动运动失败: %s", result->message.c_str());
        return;
    }

    impl_->pump_callbacks();
    if (impl_->checkMoveAppendFailure(ec)) {
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "机器人运动已启动");
}

// 停止运动
void xMateRobot::stop(std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_motion_stop_client_, ec)) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(impl_->action_mutex_);
        impl_->suppress_next_stopped_move_append_result_ = true;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::Stop::Request>();
    auto future = impl_->xmate3_motion_stop_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "停止运动失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "机器人运动已停止");
}

// 设置默认运动速度
void xMateRobot::setDefaultSpeed(int speed, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_motion_set_default_speed_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetDefaultSpeed::Request>();
    request->speed = speed;

    auto future = impl_->xmate3_motion_set_default_speed_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置默认速度失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "默认运动速度设置为: %d mm/s", speed);
}

// 设置默认转弯区
void xMateRobot::setDefaultZone(int zone, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_motion_set_default_zone_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetDefaultZone::Request>();
    request->zone = zone;

    auto future = impl_->xmate3_motion_set_default_zone_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置默认转弯区失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "默认转弯区设置为: %d mm", zone);
}

// 设置笛卡尔点位Conf检查
void xMateRobot::setDefaultConfOpt(bool forced, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_motion_set_default_conf_opt_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetDefaultConfOpt::Request>();
    request->forced = forced;

    auto future = impl_->xmate3_motion_set_default_conf_opt_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置Conf检查模式失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "笛卡尔点位Conf检查设置为: %s", forced ? "严格遵循" : "不严格遵循");
}

// 在线调整运动速率
void xMateRobot::adjustSpeedOnline(double scale, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_motion_adjust_speed_online_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::AdjustSpeedOnline::Request>();
    request->scale = scale;

    auto future = impl_->xmate3_motion_adjust_speed_online_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "在线调整速度失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "运动速率在线调整为: %.2f", scale);
}

void xMateRobot::startJog(rokae::JogOpt::Space space,
                          double rate,
                          double step,
                          unsigned int index,
                          bool direction,
                          std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }
    if (index >= 6) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return;
    }
    if (!(rate > 0.0 && rate <= 1.0)) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return;
    }
    double sdk_step = step;
    if (space == rokae::JogOpt::Space::jointSpace) {
        sdk_step = rokae::Utils::degToRad(step);
    } else {
        sdk_step = step / 1000.0;
    }
    const double signed_step = direction ? std::abs(sdk_step) : -std::abs(sdk_step);
    const int cmd_speed = std::clamp(static_cast<int>(std::lround(rate * 100.0)), 1, 100);
    moveReset(ec);
    if (ec) {
        return;
    }

    if (space == rokae::JogOpt::Space::jointSpace) {
        auto joints = jointPos(ec);
        if (ec) {
            return;
        }
        rokae::MoveAbsJCommand cmd;
        cmd.target.joints.assign(joints.begin(), joints.end());
        cmd.target.joints[index] += signed_step;
        cmd.speed = cmd_speed;
        cmd.zone = 0;
        moveAbsJ(cmd, ec);
    } else {
        auto pose = cartPosture(rokae::CoordinateType::flangeInBase, ec);
        if (ec) {
            return;
        }
        switch (index) {
            case 0: pose.x += signed_step; break;
            case 1: pose.y += signed_step; break;
            case 2: pose.z += signed_step; break;
            case 3: pose.rx += signed_step; break;
            case 4: pose.ry += signed_step; break;
            case 5: pose.rz += signed_step; break;
            default: break;
        }
        rokae::MoveLCommand cmd;
        cmd.target = pose;
        cmd.speed = cmd_speed;
        cmd.zone = 0;
        moveL(cmd, ec);
    }

    if (ec) {
        return;
    }
    moveStart(ec);
}

void xMateRobot::startJog(rokae::JogOpt::Space space,
                          unsigned int index,
                          bool direction,
                          double step,
                          std::error_code& ec) {
    startJog(space, 0.1, step, index, direction, ec);
}

void xMateRobot::moveAppend(const std::vector<rokae::MoveAbsJCommand>& cmds, std::string& cmdID, std::error_code& ec) {
    if (cmds.empty() || cmds.size() > kMoveAppendMaxCount) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return;
    }
    cmdID = std::to_string(impl_->next_cmd_id_++);
    for (const auto& cmd : cmds) {
        moveAbsJ(cmd, ec);
        if (ec) return;
    }
    ec.clear();
}

void xMateRobot::moveAppend(const std::vector<rokae::MoveJCommand>& cmds, std::string& cmdID, std::error_code& ec) {
    if (cmds.empty() || cmds.size() > kMoveAppendMaxCount) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return;
    }
    cmdID = std::to_string(impl_->next_cmd_id_++);
    for (const auto& cmd : cmds) {
        moveJ(cmd, ec);
        if (ec) return;
    }
    ec.clear();
}

void xMateRobot::moveAppend(const std::vector<rokae::MoveLCommand>& cmds, std::string& cmdID, std::error_code& ec) {
    if (cmds.empty() || cmds.size() > kMoveAppendMaxCount) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return;
    }
    cmdID = std::to_string(impl_->next_cmd_id_++);
    for (const auto& cmd : cmds) {
        moveL(cmd, ec);
        if (ec) return;
    }
    ec.clear();
}

void xMateRobot::moveAppend(const std::vector<rokae::MoveCCommand>& cmds, std::string& cmdID, std::error_code& ec) {
    if (cmds.empty() || cmds.size() > kMoveAppendMaxCount) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return;
    }
    cmdID = std::to_string(impl_->next_cmd_id_++);
    for (const auto& cmd : cmds) {
        moveC(cmd, ec);
        if (ec) return;
    }
    ec.clear();
}

void xMateRobot::moveAppend(const std::vector<rokae::MoveCFCommand>& cmds, std::string& cmdID, std::error_code& ec) {
    if (cmds.empty() || cmds.size() > kMoveAppendMaxCount) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return;
    }
    cmdID = std::to_string(impl_->next_cmd_id_++);
    for (const auto& cmd : cmds) {
        moveCF(cmd, ec);
        if (ec) return;
    }
    ec.clear();
}

void xMateRobot::moveAppend(const std::vector<rokae::MoveSPCommand>& cmds, std::string& cmdID, std::error_code& ec) {
    if (cmds.empty() || cmds.size() > kMoveAppendMaxCount) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return;
    }
    cmdID = std::to_string(impl_->next_cmd_id_++);
    for (const auto& cmd : cmds) {
        moveSP(cmd, ec);
        if (ec) return;
    }
    ec.clear();
}

void xMateRobot::executeCommand(const std::vector<rokae::MoveAbsJCommand>& cmds, std::error_code& ec) {
    if (cmds.empty() || cmds.size() > kExecuteCommandMaxCount) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return;
    }
    std::string cmd_id;
    moveReset(ec);
    if (ec) return;
    moveAppend(cmds, cmd_id, ec);
    if (ec) return;
    moveStart(ec);
}

void xMateRobot::executeCommand(const std::vector<rokae::MoveJCommand>& cmds, std::error_code& ec) {
    if (cmds.empty() || cmds.size() > kExecuteCommandMaxCount) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return;
    }
    std::string cmd_id;
    moveReset(ec);
    if (ec) return;
    moveAppend(cmds, cmd_id, ec);
    if (ec) return;
    moveStart(ec);
}

void xMateRobot::executeCommand(const std::vector<rokae::MoveLCommand>& cmds, std::error_code& ec) {
    if (cmds.empty() || cmds.size() > kExecuteCommandMaxCount) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return;
    }
    std::string cmd_id;
    moveReset(ec);
    if (ec) return;
    moveAppend(cmds, cmd_id, ec);
    if (ec) return;
    moveStart(ec);
}


} // namespace rokae::ros2
