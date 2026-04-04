#include "robot_internal.hpp"

namespace rokae::ros2 {
namespace {

constexpr size_t kMoveAppendMaxCount = 100;
constexpr size_t kExecuteCommandMaxCount = 1000;

bool try_snapshot_motion_mode(xMateRobot::Impl &impl, std::error_code &ec, rokae::MotionControlMode &mode);


/**
 * @brief Validate install-facing NRT queue preconditions before accepting queued commands.
 * @param impl Backend implementation.
 * @param command_count Number of commands requested for this append call.
 * @param max_count Maximum allowed batch size for the API entry point.
 * @param command_family Stable family tag used to reject mixed-type queue batches.
 * @param ec Output error code.
 * @return true when the queued append may proceed.
 * @throws None.
 * @note Boundary behavior: the xMate6 public lane requires moveReset() before queued NRT
 *       appends and rejects mixed command families to preserve a single deterministic request lane.
 */
bool validate_nrt_queue_preconditions(xMateRobot::Impl &impl,
                                      std::size_t command_count,
                                      std::size_t max_count,
                                      const char *command_family,
                                      std::error_code &ec) {
    if (!impl.connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    rokae::MotionControlMode current_mode = rokae::MotionControlMode::Idle;
    if (!try_snapshot_motion_mode(impl, ec, current_mode)) {
        if (ec) {
            return false;
        }
    } else if (current_mode != rokae::MotionControlMode::NrtCommand) {
        ec = rokae::make_error_code(rokae::SdkError::control_mode_mismatch);
        return false;
    }
    if (command_count == 0 || command_count > max_count) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return false;
    }
    {
        std::lock_guard<std::mutex> snapshot_lock(impl.state_cache_mutex_);
        if (impl.runtime_state_snapshot_valid_) {
            if (!impl.runtime_state_snapshot_.power_on) {
                ec = rokae::make_error_code(rokae::SdkError::motion_not_ready);
                return false;
            }
            if (impl.runtime_state_snapshot_.drag_mode) {
                ec = rokae::make_error_code(rokae::SdkError::invalid_operation);
                return false;
            }
        }
    }
    if (!impl.nrt_queue_initialized_) {
        ec = rokae::make_error_code(rokae::SdkError::invalid_operation);
        return false;
    }
    if (!impl.current_cached_command_family_.empty() && impl.current_cached_command_family_ != command_family) {
        ec = rokae::make_error_code(rokae::SdkError::invalid_operation);
        return false;
    }
    impl.current_cached_command_family_ = command_family;
    ec.clear();
    return true;
}

bool map_public_motion_mode(rokae::MotionControlMode mode, int32_t &request_mode) {
    switch (mode) {
        case rokae::MotionControlMode::NrtCommand:
            request_mode = 0;
            return true;
        case rokae::MotionControlMode::RtCommand:
            request_mode = 1;
            return true;
        case rokae::MotionControlMode::NrtRLTask:
            request_mode = 2;
            return true;
        case rokae::MotionControlMode::Idle:
        default:
            return false;
    }
}

/**
 * @brief Read the current motion-control mode from the aggregated runtime snapshot.
 * @param impl Backend implementation.
 * @param ec Output error code updated by snapshot refresh.
 * @param mode Output mapped public SDK mode.
 * @return true when the snapshot is available and authoritative.
 * @throws None.
 */
bool try_snapshot_motion_mode(xMateRobot::Impl &impl, std::error_code &ec, rokae::MotionControlMode &mode) {
    if (!impl.refreshRuntimeStateSnapshot(ec)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(impl.state_cache_mutex_);
    switch (impl.runtime_state_snapshot_.motion_mode) {
        case 0:
            mode = rokae::MotionControlMode::NrtCommand;
            break;
        case 1:
            mode = rokae::MotionControlMode::RtCommand;
            break;
        case 2:
            mode = rokae::MotionControlMode::NrtRLTask;
            break;
        default:
            mode = rokae::MotionControlMode::Idle;
            break;
    }
    ec.clear();
    return true;
}

}

/**
 * @brief Set the high-level motion-control mode for the xMate6 compatibility lane.
 * @param mode Target public SDK motion-control mode.
 * @param ec Output error code.
 * @throws None.
 * @note Boundary behavior: if the aggregated runtime snapshot already reports the requested mode,
 *       this function succeeds without sending an additional mode-change request.
 */
void xMateRobot::setMotionControlMode(rokae::MotionControlMode mode, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    rokae::MotionControlMode current_mode = rokae::MotionControlMode::Idle;
    if (try_snapshot_motion_mode(*impl_, ec, current_mode) && current_mode == mode) {
        ec.clear();
        RCLCPP_INFO(impl_->node_->get_logger(), "运动控制模式已是目标值，setMotionControlMode 按幂等成功处理");
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_motion_set_control_mode_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetMotionControlMode::Request>();
    if (!map_public_motion_mode(mode, request->mode)) {
        ec = rokae::make_error_code(rokae::SdkError::invalid_argument);
        RCLCPP_ERROR(impl_->node_->get_logger(), "非法运动控制模式值: %u", static_cast<unsigned>(mode));
        return;
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

    if (mode != rokae::MotionControlMode::NrtCommand) {
        impl_->resetNrtQueueState();
        impl_->clearCache();
    }
    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "运动控制模式设置成功");
}

// 重置运动缓存
/**
 * @brief Reset the queued non-realtime motion pipeline.
 * @param ec Output error code.
 * @throws None.
 * @note Boundary behavior: local cached commands are always cleared first for queue hygiene, but the
 *       public NRT queue is marked initialized only after the remote reset call succeeds. A disconnected
 *       session therefore still reports `not_connected` and does not arm later appends implicitly.
 */
void xMateRobot::moveReset(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    impl_->clearCache();
    impl_->resetNrtQueueState();

    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    rokae::MotionControlMode current_mode = rokae::MotionControlMode::Idle;
    if (try_snapshot_motion_mode(*impl_, ec, current_mode) && current_mode != rokae::MotionControlMode::NrtCommand) {
        ec = rokae::make_error_code(rokae::SdkError::control_mode_mismatch);
        return;
    }
    {
        std::lock_guard<std::mutex> snapshot_lock(impl_->state_cache_mutex_);
        if (impl_->runtime_state_snapshot_valid_ && impl_->runtime_state_snapshot_.drag_mode) {
            ec = rokae::make_error_code(rokae::SdkError::invalid_operation);
            return;
        }
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

    impl_->markNrtQueueInitialized();
    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "运动缓存重置成功");
}

// 开始运动
/**
 * @brief Start or resume queued non-realtime motion after flushing cached commands.
 * @param ec Output error code.
 * @throws None.
 * @note Boundary behavior: connection and public-mode validation happen before cached commands are
 *       flushed so a disconnected session cannot consume local queue state as a side effect. When the
 *       cached commands were already flushed to the backend request lane, retries reuse that remote
 *       request knowledge instead of incorrectly demanding another local append.
 */
void xMateRobot::moveStart(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    rokae::MotionControlMode current_mode = rokae::MotionControlMode::Idle;
    if (!try_snapshot_motion_mode(*impl_, ec, current_mode)) {
        if (ec) {
            RCLCPP_ERROR(impl_->node_->get_logger(),
                         "moveStart 无法在缺少 authoritative runtime snapshot 的情况下继续执行");
            return;
        }
    } else if (current_mode != rokae::MotionControlMode::NrtCommand) {
        ec = rokae::make_error_code(rokae::SdkError::motion_not_ready);
        RCLCPP_ERROR(impl_->node_->get_logger(), "moveStart 仅支持在 NrtCommand 运动模式下启动，当前模式=%u",
                     static_cast<unsigned>(current_mode));
        return;
    }

    {
        std::lock_guard<std::mutex> snapshot_lock(impl_->state_cache_mutex_);
        if (impl_->runtime_state_snapshot_valid_) {
            if (!impl_->runtime_state_snapshot_.power_on) {
                ec = rokae::make_error_code(rokae::SdkError::motion_not_ready);
                return;
            }
            if (impl_->runtime_state_snapshot_.drag_mode) {
                ec = rokae::make_error_code(rokae::SdkError::invalid_operation);
                return;
            }
        }
    }

    if (!impl_->nrt_queue_initialized_ && !impl_->nrt_queue_remote_request_known_) {
        ec = rokae::make_error_code(rokae::SdkError::invalid_operation);
        RCLCPP_ERROR(impl_->node_->get_logger(), "moveStart requires a successful moveReset() before the first queued NRT request");
        return;
    }

    const bool has_local_cached_commands = impl_->nrt_queue_has_cached_commands_;
    if (has_local_cached_commands) {
        if (!impl_->flushCachedCommands(ec)) {
            return;
        }
    } else if (!impl_->nrt_queue_remote_request_known_) {
        ec = rokae::make_error_code(rokae::SdkError::trajectory_empty);
        RCLCPP_ERROR(impl_->node_->get_logger(), "moveStart rejected because no queued NRT request is known locally or remotely");
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
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_motion_stop_client_, ec)) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(impl_->action_mutex_);
        impl_->suppress_next_stopped_move_append_result_ = !impl_->active_goal_handles_.empty();
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
    auto _last_error_scope = track_last_error(impl_, ec);
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
    auto _last_error_scope = track_last_error(impl_, ec);
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
    auto _last_error_scope = track_last_error(impl_, ec);
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
    auto _last_error_scope = track_last_error(impl_, ec);
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
    auto _last_error_scope = track_last_error(impl_, ec);
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
    auto _last_error_scope = track_last_error(impl_, ec);
    startJog(space, 0.1, step, index, direction, ec);
}

void xMateRobot::moveAppend(const std::vector<rokae::MoveAbsJCommand>& cmds, std::string& cmdID, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!validate_nrt_queue_preconditions(*impl_, cmds.size(), kMoveAppendMaxCount, "move_absj", ec)) {
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
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!validate_nrt_queue_preconditions(*impl_, cmds.size(), kMoveAppendMaxCount, "move_j", ec)) {
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
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!validate_nrt_queue_preconditions(*impl_, cmds.size(), kMoveAppendMaxCount, "move_l", ec)) {
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
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!validate_nrt_queue_preconditions(*impl_, cmds.size(), kMoveAppendMaxCount, "move_c", ec)) {
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
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!validate_nrt_queue_preconditions(*impl_, cmds.size(), kMoveAppendMaxCount, "move_cf", ec)) {
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
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!validate_nrt_queue_preconditions(*impl_, cmds.size(), kMoveAppendMaxCount, "move_sp", ec)) {
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
    auto _last_error_scope = track_last_error(impl_, ec);
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
    auto _last_error_scope = track_last_error(impl_, ec);
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
    auto _last_error_scope = track_last_error(impl_, ec);
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

void xMateRobot::executeCommand(const std::vector<rokae::MoveCCommand>& cmds, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
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
