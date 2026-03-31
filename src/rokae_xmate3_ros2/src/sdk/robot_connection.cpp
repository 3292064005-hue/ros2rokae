#include "robot_internal.hpp"

namespace rokae::ros2 {

void xMateRobot::connectToRobot(std::error_code& ec) {
    if (impl_->connected_) {
        ec = std::make_error_code(std::errc::already_connected);
        RCLCPP_WARN(impl_->node_->get_logger(), "机器人已连接，无需重复连接");
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_robot_connect_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::Connect::Request>();
    request->remote_ip = impl_->remote_ip_;
    request->local_ip = impl_->local_ip_;

    auto future = impl_->xmate3_robot_connect_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        RCLCPP_ERROR(impl_->node_->get_logger(), "机器人连接请求发送失败");
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "机器人连接失败: %s", result->message.c_str());
        return;
    }

    impl_->connected_ = true;
    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "机器人连接成功, 远程IP: %s", impl_->remote_ip_.c_str());
}

// 断开机器人连接

void xMateRobot::disconnectFromRobot(std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        RCLCPP_WARN(impl_->node_->get_logger(), "机器人未连接，无需断开");
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_robot_disconnect_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::Disconnect::Request>();
    auto future = impl_->xmate3_robot_disconnect_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        RCLCPP_ERROR(impl_->node_->get_logger(), "断开连接请求发送失败");
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "断开连接失败: %s", result->message.c_str());
        return;
    }

    impl_->connected_ = false;
    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "机器人断开连接成功");
}

// 获取机器人信息

rokae::Info xMateRobot::robotInfo(std::error_code& ec) {
    rokae::Info info;
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return info;
    }

    if (!impl_->wait_for_service(impl_->xmate3_robot_get_info_client_, ec)) {
        return info;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetInfo::Request>();
    auto future = impl_->xmate3_robot_get_info_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return info;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return info;
    }

    // 映射返回值
    info.model = result->model;
    info.serial_number = result->serial_number;
    info.version = result->firmware_version;
    info.sdk_version = result->sdk_version;
    info.type = result->robot_type;
    info.joint_num = result->joint_num;
    ec.clear();
    return info;
}

// 获取上电状态

rokae::PowerState xMateRobot::powerState(std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return rokae::PowerState::unknown;
    }
    if (!impl_->wait_for_service(impl_->xmate3_robot_get_power_state_client_, ec)) {
        return rokae::PowerState::unknown;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetPowerState::Request>();
    auto future = impl_->xmate3_robot_get_power_state_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return rokae::PowerState::unknown;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return rokae::PowerState::unknown;
    }
    ec.clear();
    switch (result->state.state) {
        case rokae_xmate3_ros2::msg::PowerState::ON:
            return rokae::PowerState::on;
        case rokae_xmate3_ros2::msg::PowerState::OFF:
            return rokae::PowerState::off;
        case rokae_xmate3_ros2::msg::PowerState::ESTOP:
            return rokae::PowerState::estop;
        case rokae_xmate3_ros2::msg::PowerState::GSTOP:
            return rokae::PowerState::gstop;
        default:
            return rokae::PowerState::unknown;
    }
}


// 设置上电状态

void xMateRobot::setPowerState(bool on, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_robot_set_power_state_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetPowerState::Request>();
    request->on = on;

    auto future = impl_->xmate3_robot_set_power_state_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置电源状态失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "机器人电源状态设置为: %s", on ? "上电" : "下电");
}

// 获取操作模式

rokae::OperateMode xMateRobot::operateMode(std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return rokae::OperateMode::unknown;
    }
    if (!impl_->wait_for_service(impl_->xmate3_robot_get_operate_mode_client_, ec)) {
        return rokae::OperateMode::unknown;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetOperateMode::Request>();
    auto future = impl_->xmate3_robot_get_operate_mode_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return rokae::OperateMode::unknown;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return rokae::OperateMode::unknown;
    }
    ec.clear();
    // 核心修正：从msg中取出uint8_t原始值再强转
    return static_cast<rokae::OperateMode>(result->mode.mode);
}


// 设置操作模式

void xMateRobot::setOperateMode(rokae::OperateMode mode, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_robot_set_operate_mode_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetOperateMode::Request>();
    request->mode = static_cast<uint8_t>(mode);

    auto future = impl_->xmate3_robot_set_operate_mode_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置操作模式失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "机器人操作模式设置为: %s", 
        mode == rokae::OperateMode::automatic ? "自动" : "手动");
}

// 获取运行状态

rokae::OperationState xMateRobot::operationState(std::error_code& ec) {
    impl_->pump_callbacks();
    if (impl_->checkMoveAppendFailure(ec)) {
        return rokae::OperationState::unknown;
    }
    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return rokae::OperationState::unknown;
    }
    ec.clear();
    switch (impl_->last_operation_state_.state) {
        case rokae_xmate3_ros2::msg::OperationState::IDLE:
            return rokae::OperationState::idle;
        case rokae_xmate3_ros2::msg::OperationState::JOG:
            return rokae::OperationState::jog;
        case rokae_xmate3_ros2::msg::OperationState::RT_CONTROLLING:
            return rokae::OperationState::rtControlling;
        case rokae_xmate3_ros2::msg::OperationState::DRAG:
            return rokae::OperationState::drag;
        case rokae_xmate3_ros2::msg::OperationState::RL_PROGRAM:
            return rokae::OperationState::rlProgram;
        case rokae_xmate3_ros2::msg::OperationState::MOVING:
            return rokae::OperationState::moving;
        case rokae_xmate3_ros2::msg::OperationState::JOGGING:
            return rokae::OperationState::jogging;
        default:
            return rokae::OperationState::unknown;
    }
}

// 查询控制器日志

std::vector<rokae::LogInfo> xMateRobot::queryControllerLog(unsigned int count, std::error_code& ec) {
    std::vector<rokae::LogInfo> logs;
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return logs;
    }
    if (!impl_->wait_for_service(impl_->xmate3_robot_query_log_client_, ec)) {
        return logs;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::QueryControllerLog::Request>();
    request->count = count;
    auto future = impl_->xmate3_robot_query_log_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return logs;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return logs;
    }
    // 核心修正：遍历日志msg数组，不是string数组
    for (const auto& log_msg : result->logs) {
        rokae::LogInfo log;
        log.timestamp = log_msg.timestamp;
        log.content = log_msg.content;
        log.repair = log_msg.repair;
        log.level = log_msg.level;
        logs.push_back(log);
    }
    ec.clear();
    return logs;
}


// 清除伺服报警

void xMateRobot::clearServoAlarm(std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_robot_clear_servo_alarm_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::ClearServoAlarm::Request>();
    auto future = impl_->xmate3_robot_clear_servo_alarm_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "清除伺服报警失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "伺服报警清除成功");
}

} // namespace rokae::ros2
