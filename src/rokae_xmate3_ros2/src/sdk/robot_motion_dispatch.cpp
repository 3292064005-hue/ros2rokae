#include "robot_internal.hpp"

namespace rokae::ros2 {

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
