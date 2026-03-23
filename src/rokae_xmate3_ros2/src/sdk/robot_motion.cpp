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

// ==================== IO接口实现 ====================
bool xMateRobot::getDI(unsigned int board, unsigned int port, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }

    if (!impl_->wait_for_service(impl_->xmate3_io_get_di_client_, ec)) {
        return false;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetDI::Request>();
    request->board = board;
    request->port = port;

    auto future = impl_->xmate3_io_get_di_client_->async_send_request(request);
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
    return result->state;
}

bool xMateRobot::getDO(unsigned int board, unsigned int port, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }

    if (!impl_->wait_for_service(impl_->xmate3_io_get_do_client_, ec)) {
        return false;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetDO::Request>();
    request->board = board;
    request->port = port;

    auto future = impl_->xmate3_io_get_do_client_->async_send_request(request);
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
    return result->state;
}

void xMateRobot::setDI(unsigned int board, unsigned int port, bool state, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_io_set_di_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetDI::Request>();
    request->board = board;
    request->port = port;
    request->state = state;

    auto future = impl_->xmate3_io_set_di_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置DI信号失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
}

void xMateRobot::setDO(unsigned int board, unsigned int port, bool state, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_io_set_do_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetDO::Request>();
    request->board = board;
    request->port = port;
    request->state = state;

    auto future = impl_->xmate3_io_set_do_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置DO信号失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "DO信号设置成功, board:%d, port:%d, state:%d", board, port, state);
}

double xMateRobot::getAI(unsigned int board, unsigned int port, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return 0.0;
    }

    if (!impl_->wait_for_service(impl_->xmate3_io_get_ai_client_, ec)) {
        return 0.0;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetAI::Request>();
    request->board = board;
    request->port = port;

    auto future = impl_->xmate3_io_get_ai_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return 0.0;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return 0.0;
    }

    ec.clear();
    return result->value;
}

void xMateRobot::setAO(unsigned int board, unsigned int port, double value, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_io_set_ao_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetAO::Request>();
    request->board = board;
    request->port = port;
    request->value = value;

    auto future = impl_->xmate3_io_set_ao_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置AO信号失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
}

void xMateRobot::setSimulationMode(bool state, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_io_set_simulation_mode_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetSimulationMode::Request>();
    request->state = state;

    auto future = impl_->xmate3_io_set_simulation_mode_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置仿真模式失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "输入仿真模式设置为: %s", state ? "开启" : "关闭");
}

// ==================== 拖动与路径录制接口实现 ====================
void xMateRobot::enableDrag(rokae::DragParameter::Space space, rokae::DragParameter::Type type, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_enable_drag_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::EnableDrag::Request>();
    request->space = static_cast<uint8_t>(space);
    request->type = static_cast<uint8_t>(type);

    auto future = impl_->xmate3_cobot_enable_drag_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "开启拖动失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "机器人拖动模式已开启");
}

void xMateRobot::disableDrag(std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_disable_drag_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::DisableDrag::Request>();
    auto future = impl_->xmate3_cobot_disable_drag_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "关闭拖动失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "机器人拖动模式已关闭");
}

void xMateRobot::startRecordPath(std::chrono::seconds duration, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_start_record_path_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::StartRecordPath::Request>();
    request->duration = duration.count();

    auto future = impl_->xmate3_cobot_start_record_path_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "开始路径录制失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "路径录制已开始, 时长: %ld秒", duration.count());
}

void xMateRobot::stopRecordPath(std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_stop_record_path_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::StopRecordPath::Request>();
    auto future = impl_->xmate3_cobot_stop_record_path_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "停止路径录制失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "路径录制已停止");
}

void xMateRobot::cancelRecordPath(std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_cancel_record_path_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::CancelRecordPath::Request>();
    auto future = impl_->xmate3_cobot_cancel_record_path_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "取消路径录制失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "路径录制已取消");
}

void xMateRobot::saveRecordPath(const std::string& name, std::error_code& ec, const std::string& saveAs) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_save_record_path_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SaveRecordPath::Request>();
    request->name = name;
    request->save_as = saveAs;

    auto future = impl_->xmate3_cobot_save_record_path_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "保存路径失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "路径保存成功, 名称: %s", name.c_str());
}

void xMateRobot::replayPath(const std::string& name, double rate, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_replay_path_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::ReplayPath::Request>();
    request->name = name;
    request->rate = rate;

    auto future = impl_->xmate3_cobot_replay_path_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "路径回放失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "路径回放已开始, 名称: %s, 速率: %.2f", name.c_str(), rate);
}

void xMateRobot::removePath(const std::string& name, std::error_code& ec, bool removeAll) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_remove_path_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::RemovePath::Request>();
    request->name = name;
    request->remove_all = removeAll;

    auto future = impl_->xmate3_cobot_remove_path_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "删除路径失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "路径删除成功");
}

std::vector<std::string> xMateRobot::queryPathLists(std::error_code& ec) {
    std::vector<std::string> path_list;
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return path_list;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_query_path_lists_client_, ec)) {
        return path_list;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::QueryPathLists::Request>();
    auto future = impl_->xmate3_cobot_query_path_lists_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return path_list;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return path_list;
    }

    path_list = result->path_names;
    ec.clear();
    return path_list;
}

// cache helpers (implemented on Impl)
void xMateRobot::Impl::cacheCommand(const rokae_xmate3_ros2::action::MoveAppend::Goal &goal) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    // 把新命令合并到单个缓存goal中，而不是创建多个goal
    for (const auto& cmd : goal.absj_cmds) {
        cached_goal_.absj_cmds.push_back(cmd);
    }
    for (const auto& cmd : goal.j_cmds) {
        cached_goal_.j_cmds.push_back(cmd);
    }
    for (const auto& cmd : goal.l_cmds) {
        cached_goal_.l_cmds.push_back(cmd);
    }
    for (const auto& cmd : goal.c_cmds) {
        cached_goal_.c_cmds.push_back(cmd);
    }
    for (const auto& cmd : goal.cf_cmds) {
        cached_goal_.cf_cmds.push_back(cmd);
    }
    for (const auto& cmd : goal.sp_cmds) {
        cached_goal_.sp_cmds.push_back(cmd);
    }
}

void xMateRobot::Impl::clearCache() {
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        cached_goal_ = rokae_xmate3_ros2::action::MoveAppend::Goal();
        active_goal_handles_.clear();
    }
    resetMoveAppendState();
}

bool xMateRobot::Impl::flushCachedCommands(std::error_code &ec) {
    rokae_xmate3_ros2::action::MoveAppend::Goal goal_to_send;
    size_t total_cmds = 0;
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        // 检查是否有缓存的命令
        total_cmds = cached_goal_.absj_cmds.size() + cached_goal_.j_cmds.size() +
                     cached_goal_.l_cmds.size() + cached_goal_.c_cmds.size() +
                     cached_goal_.cf_cmds.size() + cached_goal_.sp_cmds.size();
        if (total_cmds == 0) {
            ec.clear();
            return true;
        }
        // 取出缓存的goal并清空
        goal_to_send = std::move(cached_goal_);
        cached_goal_ = rokae_xmate3_ros2::action::MoveAppend::Goal();
    }
    if (!move_append_action_client_) {
        ec = std::make_error_code(std::errc::not_supported);
        RCLCPP_ERROR(node_->get_logger(), "MoveAppend action client not initialized");
        return false;
    }
    // ensure the action server is ready before dispatching any cached goals
    if (!move_append_action_client_->wait_for_action_server(1s)) {
        RCLCPP_WARN(node_->get_logger(), "MoveAppend action server not ready yet, waiting up to 10 seconds...");
        if (!move_append_action_client_->wait_for_action_server(10s)) {
            ec = std::make_error_code(std::errc::host_unreachable);
            RCLCPP_ERROR(node_->get_logger(), "MoveAppend action server not available after timeout");
            return false;
        }
    }

    RCLCPP_INFO(node_->get_logger(), "已发送 %zu 条运动指令", total_cmds);
    resetMoveAppendState();

    rclcpp_action::Client<rokae_xmate3_ros2::action::MoveAppend>::SendGoalOptions send_goal_options;
    send_goal_options.result_callback = [this](const auto &wrapped_result) {
        handleMoveAppendResult(wrapped_result);
    };

    // 发送goal（异步，但等待goal被接受）
    auto send_future = move_append_action_client_->async_send_goal(goal_to_send, send_goal_options);

    // 等待goal被接受
    if (wait_for_future(send_future, 5s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::timed_out);
        RCLCPP_ERROR(node_->get_logger(), "Failed to send goal to action server");
        return false;
    }

    auto goal_handle = send_future.get();
    if (!goal_handle) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by action server");
        return false;
    }

    // 保存goal handle的引用，确保action不被取消
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        active_goal_handles_.push_back(goal_handle);
    }

    // 【关键】等待直到operation state变为MOVING，确保运动真正开始
    const bool has_complex_cartesian =
        !goal_to_send.c_cmds.empty() || !goal_to_send.cf_cmds.empty() || !goal_to_send.sp_cmds.empty();
    const auto start_timeout = has_complex_cartesian ? std::chrono::seconds(15) : std::chrono::seconds(5);
    RCLCPP_INFO(node_->get_logger(), "等待运动开始...");
    auto start_wait = std::chrono::steady_clock::now();
    bool motion_started = false;
    bool action_completed_during_wait = false;
    while (std::chrono::steady_clock::now() - start_wait < start_timeout) {
        pump_callbacks();
        if (checkMoveAppendFailure(ec)) {
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (last_operation_state_.state == rokae_xmate3_ros2::msg::OperationState::MOVING) {
                motion_started = true;
                break;
            }
        }
        {
            std::lock_guard<std::mutex> lock(action_mutex_);
            action_completed_during_wait =
                move_append_result_ready_ &&
                move_append_result_code_ == rclcpp_action::ResultCode::SUCCEEDED &&
                move_append_result_success_;
        }
        if (action_completed_during_wait) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (motion_started) {
        RCLCPP_INFO(node_->get_logger(), "运动已开始!");
    } else {
        if (checkMoveAppendFailure(ec)) {
            return false;
        }
        if (action_completed_during_wait) {
            RCLCPP_INFO(node_->get_logger(), "运动在等待窗口内已完成，跳过开始检测");
        } else {
            RCLCPP_WARN(node_->get_logger(), "等待运动开始超时，继续执行...");
        }
    }

    ec.clear();
    return true;
}

// helper to convert application commands into action goal parts
static rokae_xmate3_ros2::msg::CartesianPosition toMsg(const rokae::CartesianPosition &pose) {
    rokae_xmate3_ros2::msg::CartesianPosition m;
    m.x = pose.x;
    m.y = pose.y;
    m.z = pose.z;
    m.rx = pose.rx;
    m.ry = pose.ry;
    m.rz = pose.rz;
    m.elbow = pose.elbow;
    m.has_elbow = pose.hasElbow;
    m.conf_data.assign(pose.confData.begin(), pose.confData.end());
    m.external = pose.external;
    return m;
}

static int32_t offsetTypeToMsg(rokae::CartesianPosition::Offset::Type type) {
    switch (type) {
        case rokae::CartesianPosition::Offset::offs:
            return 1;
        case rokae::CartesianPosition::Offset::relTool:
            return 2;
        case rokae::CartesianPosition::Offset::none:
        default:
            return 0;
    }
}

static std::array<double, 6> frameToPose(const rokae::Frame &frame) {
    return {frame.trans[0], frame.trans[1], frame.trans[2],
            frame.rpy[0], frame.rpy[1], frame.rpy[2]};
}

static rokae_xmate3_ros2::msg::MoveAbsJCommand toMsg(const rokae::MoveAbsJCommand &cmd) {
    rokae_xmate3_ros2::msg::MoveAbsJCommand m;
    m.target.joints = cmd.target.joints;
    m.target.external = cmd.target.external;
    m.speed = cmd.speed;
    m.zone = cmd.zone;
    return m;
}
static rokae_xmate3_ros2::msg::MoveJCommand toMsg(const rokae::MoveJCommand &cmd) {
    rokae_xmate3_ros2::msg::MoveJCommand m;
    m.target = toMsg(cmd.target);
    m.speed = cmd.speed;
    m.zone = cmd.zone;
    m.offset_type = offsetTypeToMsg(cmd.offset.type);
    m.offset_pose = frameToPose(cmd.offset.frame);
    return m;
}
static rokae_xmate3_ros2::msg::MoveLCommand toMsg(const rokae::MoveLCommand &cmd) {
    rokae_xmate3_ros2::msg::MoveLCommand m;
    m.target = toMsg(cmd.target);
    m.speed = cmd.speed;
    m.zone = cmd.zone;
    m.offset_type = offsetTypeToMsg(cmd.offset.type);
    m.offset_pose = frameToPose(cmd.offset.frame);
    return m;
}
static rokae_xmate3_ros2::msg::MoveCCommand toMsg(const rokae::MoveCCommand &cmd) {
    rokae_xmate3_ros2::msg::MoveCCommand m;
    m.target = toMsg(cmd.target);
    m.aux = toMsg(cmd.aux);
    m.speed = cmd.speed;
    m.zone = cmd.zone;
    m.target_offset_type = offsetTypeToMsg(cmd.targetOffset.type);
    m.target_offset_pose = frameToPose(cmd.targetOffset.frame);
    m.aux_offset_type = offsetTypeToMsg(cmd.auxOffset.type);
    m.aux_offset_pose = frameToPose(cmd.auxOffset.frame);
    return m;
}
static rokae_xmate3_ros2::msg::MoveCFCommand toMsg(const rokae::MoveCFCommand &cmd) {
    rokae_xmate3_ros2::msg::MoveCFCommand m;
    m.target = toMsg(cmd.target);
    m.aux = toMsg(cmd.aux);
    m.speed = cmd.speed;
    m.zone = cmd.zone;
    m.angle = cmd.angle;
    m.target_offset_type = offsetTypeToMsg(cmd.targetOffset.type);
    m.target_offset_pose = frameToPose(cmd.targetOffset.frame);
    m.aux_offset_type = offsetTypeToMsg(cmd.auxOffset.type);
    m.aux_offset_pose = frameToPose(cmd.auxOffset.frame);
    return m;
}
static rokae_xmate3_ros2::msg::MoveSPCommand toMsg(const rokae::MoveSPCommand &cmd) {
    rokae_xmate3_ros2::msg::MoveSPCommand m;
    m.target = toMsg(cmd.target);
    m.speed = cmd.speed;
    m.zone = cmd.zone;
    m.radius = cmd.radius;
    m.radius_step = cmd.radius_step;
    m.angle = cmd.angle;
    m.direction = cmd.direction;
    m.offset_type = offsetTypeToMsg(rokae::CartesianPosition::Offset::none);
    m.offset_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    return m;
}

void xMateRobot::moveAbsJ(const rokae::MoveAbsJCommand& cmd, std::error_code& ec) {
    rokae_xmate3_ros2::action::MoveAppend::Goal goal;
    goal.absj_cmds.push_back(toMsg(cmd));
    impl_->cacheCommand(goal);
    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "MoveAbsJ指令已缓存");
}
void xMateRobot::moveJ(const rokae::MoveJCommand& cmd, std::error_code& ec) {
    rokae_xmate3_ros2::action::MoveAppend::Goal goal;
    goal.j_cmds.push_back(toMsg(cmd));
    impl_->cacheCommand(goal);
    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "MoveJ指令已缓存");
}
void xMateRobot::moveL(const rokae::MoveLCommand& cmd, std::error_code& ec) {
    rokae_xmate3_ros2::action::MoveAppend::Goal goal;
    goal.l_cmds.push_back(toMsg(cmd));
    impl_->cacheCommand(goal);
    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "MoveL指令已缓存");
}
void xMateRobot::moveC(const rokae::MoveCCommand& cmd, std::error_code& ec) {
    rokae_xmate3_ros2::action::MoveAppend::Goal goal;
    goal.c_cmds.push_back(toMsg(cmd));
    impl_->cacheCommand(goal);
    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "MoveC指令已缓存");
}
void xMateRobot::moveCF(const rokae::MoveCFCommand& cmd, std::error_code& ec) {
    rokae_xmate3_ros2::action::MoveAppend::Goal goal;
    goal.cf_cmds.push_back(toMsg(cmd));
    impl_->cacheCommand(goal);
    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "MoveCF指令已缓存");
}
void xMateRobot::moveSP(const rokae::MoveSPCommand& cmd, std::error_code& ec) {
    rokae_xmate3_ros2::action::MoveAppend::Goal goal;
    goal.sp_cmds.push_back(toMsg(cmd));
    impl_->cacheCommand(goal);
    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "MoveSP指令已缓存");
}

} // namespace rokae::ros2
