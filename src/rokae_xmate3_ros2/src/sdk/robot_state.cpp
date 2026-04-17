#include "robot_internal.hpp"

namespace rokae::ros2 {

rokae::JointPosition xMateRobot::calcIk(const rokae::CartesianPosition& posture, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    rokae::JointPosition joints(6);
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return joints;
    }
    if (!impl_->wait_for_service(impl_->xmate3_robot_calc_ik_client_, ec)) {
        return joints;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::CalcIk::Request>();
    // 核心修正：按数组赋值，不是单独的x/y/z/rx/ry/rz
    request->target_posture[0] = posture.x;
    request->target_posture[1] = posture.y;
    request->target_posture[2] = posture.z;
    request->target_posture[3] = posture.rx;
    request->target_posture[4] = posture.ry;
    request->target_posture[5] = posture.rz;
    request->elbow = posture.elbow;
    request->has_elbow = posture.hasElbow;
    request->conf_data.assign(posture.confData.begin(), posture.confData.end());
    request->external = posture.external;

    auto future = impl_->xmate3_robot_calc_ik_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return joints;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "逆解计算失败: %s", result->message.c_str());
        return joints;
    }
    // 核心修正：按数组读取关节角度，不是单独的j1-j6
    joints.joints[0] = result->joint_positions[0];
    joints.joints[1] = result->joint_positions[1];
    joints.joints[2] = result->joint_positions[2];
    joints.joints[3] = result->joint_positions[3];
    joints.joints[4] = result->joint_positions[4];
    joints.joints[5] = result->joint_positions[5];
    ec.clear();
    return joints;
}


// 正解计算

rokae::CartesianPosition xMateRobot::calcFk(const rokae::JointPosition& joints, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    rokae::CartesianPosition posture;
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return posture;
    }
    if (!impl_->wait_for_service(impl_->xmate3_robot_calc_fk_client_, ec)) {
        return posture;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::CalcFk::Request>();
    // 核心修正：按数组赋值关节角度，不是单独的j1-j6
    request->joint_positions[0] = joints.joints[0];
    request->joint_positions[1] = joints.joints[1];
    request->joint_positions[2] = joints.joints[2];
    request->joint_positions[3] = joints.joints[3];
    request->joint_positions[4] = joints.joints[4];
    request->joint_positions[5] = joints.joints[5];
    
    auto future = impl_->xmate3_robot_calc_fk_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return posture;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "正解计算失败: %s", result->message.c_str());
        return posture;
    }
    // 核心修正：按数组读取位姿，不是单独的x/y/z/rx/ry/rz
    posture.x = result->posture[0];
    posture.y = result->posture[1];
    posture.z = result->posture[2];
    posture.rx = result->posture[3];
    posture.ry = result->posture[4];
    posture.rz = result->posture[5];
    ec.clear();
    return posture;
}


// 获取工具工件组

rokae::Toolset xMateRobot::toolset(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    rokae::Toolset toolset;
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return toolset;
    }

    if (impl_->refreshRuntimeStateSnapshot(ec)) {
        std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
        const auto &snapshot = impl_->runtime_state_snapshot_;
        toolset.tool_name = snapshot.tool_name;
        toolset.wobj_name = snapshot.wobj_name;
        const auto tool_count = std::min<size_t>(toolset.tool_pose.size(), snapshot.tool_pose.size());
        const auto wobj_count = std::min<size_t>(toolset.wobj_pose.size(), snapshot.wobj_pose.size());
        std::copy_n(snapshot.tool_pose.begin(), tool_count, toolset.tool_pose.begin());
        std::copy_n(snapshot.wobj_pose.begin(), wobj_count, toolset.wobj_pose.begin());
        toolset.load.mass = snapshot.tool_mass;
        toolset.load.cog = snapshot.tool_com;
    } else {
        impl_->ensureToolingClients();

        if (!impl_->wait_for_service(impl_->xmate3_robot_get_toolset_client_, ec)) {
            return toolset;
        }

        auto request = std::make_shared<rokae_xmate3_ros2::srv::GetToolset::Request>();
        auto future = impl_->xmate3_robot_get_toolset_client_->async_send_request(request);
        if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
            ec = std::make_error_code(std::errc::io_error);
            return toolset;
        }

        auto result = future.get();
        if (!result->success) {
            ec = std::make_error_code(std::errc::operation_not_permitted);
            return toolset;
        }

        toolset.tool_name = result->tool_name;
        toolset.wobj_name = result->wobj_name;
        const auto tool_count = std::min<size_t>(toolset.tool_pose.size(), result->tool_pose.size());
        const auto wobj_count = std::min<size_t>(toolset.wobj_pose.size(), result->wobj_pose.size());
        std::copy_n(result->tool_pose.begin(), tool_count, toolset.tool_pose.begin());
        std::copy_n(result->wobj_pose.begin(), wobj_count, toolset.wobj_pose.begin());
    }
    toolset.end = rokae::Frame(toolset.tool_pose);
    toolset.ref = rokae::Frame(toolset.wobj_pose);
    {
        std::lock_guard<std::mutex> lock(impl_->state_mutex_);
        impl_->toolset_cache_ = toolset;
    }
    ec.clear();
    return toolset;
}

// 设置工具工件组

void xMateRobot::setToolset(const rokae::Toolset& toolset, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    impl_->ensureToolingClients();

    if (!impl_->wait_for_service(impl_->xmate3_robot_set_toolset_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetToolset::Request>();
    request->tool_name = toolset.tool_name;
    request->wobj_name = toolset.wobj_name;
    request->tool_pose.assign(toolset.tool_pose.begin(), toolset.tool_pose.end());
    request->wobj_pose.assign(toolset.wobj_pose.begin(), toolset.wobj_pose.end());

    auto future = impl_->xmate3_robot_set_toolset_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    const auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置工具工件组失败: %s", result->message.c_str());
        return;
    }

    {
        std::lock_guard<std::mutex> lock(impl_->state_mutex_);
        impl_->toolset_cache_ = toolset;
    }
    impl_->clearRuntimeStateSnapshotCache();
    ec.clear();
}

void xMateRobot::setToolset(const std::string& toolName, const std::string& wobjName, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    impl_->ensureToolingClients();

    if (!impl_->wait_for_service(impl_->xmate3_robot_set_toolset_by_name_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetToolsetByName::Request>();
    request->tool_name = toolName;
    request->wobj_name = wobjName;

    auto future = impl_->xmate3_robot_set_toolset_by_name_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "按名称设置工具工件组失败: %s", result->message.c_str());
        return;
    }

    {
        std::error_code refresh_ec;
        auto refreshed = toolset(refresh_ec);
        if (!refresh_ec) {
            std::lock_guard<std::mutex> lock(impl_->state_mutex_);
            impl_->toolset_cache_ = refreshed;
        }
    }
    impl_->clearRuntimeStateSnapshotCache();
    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "工具工件组设置成功, 工具: %s, 工件: %s", toolName.c_str(), wobjName.c_str());
}

// 开启碰撞检测

void xMateRobot::enableCollisionDetection(const std::array<double, 6>& sensitivity, rokae::StopLevel behaviour, double fallback, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    impl_->ensureSafetyClients();

    if (!impl_->wait_for_service(impl_->xmate3_robot_enable_collision_detection_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::EnableCollisionDetection::Request>();
    request->sensitivity.assign(sensitivity.begin(), sensitivity.end());
    request->behaviour = static_cast<uint8_t>(behaviour);
    request->fallback = fallback;

    auto future = impl_->xmate3_robot_enable_collision_detection_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "开启碰撞检测失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "碰撞检测开启成功");
}

// 关闭碰撞检测

void xMateRobot::disableCollisionDetection(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    impl_->ensureSafetyClients();

    if (!impl_->wait_for_service(impl_->xmate3_robot_disable_collision_detection_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::DisableCollisionDetection::Request>();
    auto future = impl_->xmate3_robot_disable_collision_detection_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "关闭碰撞检测失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "碰撞检测关闭成功");
}

// 获取软限位

bool xMateRobot::getSoftLimit(std::array<std::array<double,2>,6>& limits, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }

    impl_->ensureSafetyClients();

    if (!impl_->wait_for_service(impl_->xmate3_robot_get_soft_limit_client_, ec)) {
        return false;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetSoftLimit::Request>();
    auto future = impl_->xmate3_robot_get_soft_limit_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }

    // 关键修改：一维数组转二维数组逻辑（适配ROS2不支持多维数组的限制）
    for (int i = 0; i < 6; ++i) {
        limits[i][0] = result->limits[i * 2];    // 第i个关节下限（一维索引：i*2）
        limits[i][1] = result->limits[i * 2 + 1];// 第i个关节上限（一维索引：i*2+1）
    }
    ec.clear();
    return result->enable;
}

// 设置软限位

void xMateRobot::setSoftLimit(bool enable, std::error_code& ec, const std::array<std::array<double,2>,6>& limits) {
    auto _last_error_scope = track_last_error(impl_, ec);
    // 1. 前置校验：机器人连接状态
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        RCLCPP_WARN(impl_->node_->get_logger(), "设置软限位失败：机器人未连接");
        return;
    }

    std::array<std::array<double, 2>, 6> effective_limits = limits;
    bool use_existing_limits = enable;
    for (int i = 0; i < 6; ++i) {
        const bool axis_uses_sentinel = (limits[i][0] == DBL_MAX && limits[i][1] == DBL_MAX);
        use_existing_limits = use_existing_limits && axis_uses_sentinel;
    }
    if (use_existing_limits) {
        const bool current_enabled = getSoftLimit(effective_limits, ec);
        (void)current_enabled;
        if (ec) {
            return;
        }
    }

    // 2. 前置校验：仅在启用软限位时要求显式提供有效区间。
    //    关闭软限位沿用控制侧语义，不要求调用方提供有效 limits。
    if (enable) {
        for (int i = 0; i < 6; ++i) {
            if (effective_limits[i][0] >= effective_limits[i][1]) {
                ec = std::make_error_code(std::errc::invalid_argument);
                RCLCPP_ERROR(impl_->node_->get_logger(),
                    "设置软限位失败：第%d关节限位值无效（下限%.3f ≥ 上限%.3f）",
                    i + 1, effective_limits[i][0], effective_limits[i][1]);
                return;
            }
        }
    }

    // 3. 等待服务可用
    impl_->ensureSafetyClients();

    if (!impl_->wait_for_service(impl_->xmate3_robot_set_soft_limit_client_, ec)) {
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置软限位失败：SetSoftLimit服务不可用");
        return;
    }

    // 4. 构造请求：二维数组转一维数组（适配ROS2的float64[12]）
    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetSoftLimit::Request>();
    request->enable = enable;
    for (int i = 0; i < 6; ++i) {
        // 关键修改：一维数组索引计算（i*2=下限，i*2+1=上限）
        request->limits[i * 2] = effective_limits[i][0];    // 第i+1关节下限
        request->limits[i * 2 + 1] = effective_limits[i][1];// 第i+1关节上限
    }

    // 5. 发送服务请求并等待响应
    auto future = impl_->xmate3_robot_set_soft_limit_client_->async_send_request(request);
    auto spin_result = impl_->wait_for_future(future);
    if (spin_result != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        RCLCPP_ERROR(impl_->node_->get_logger(), 
            "设置软限位失败：服务调用超时/中断（spin结果：%d）", static_cast<int>(spin_result));
        return;
    }

    // 6. 处理服务响应
    auto result = future.get();
    if (!result->success) {
        // 优化错误码：invalid_state 比 operation_not_permitted 更贴合"限位设置失败"
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), 
            "设置软限位失败（启用状态：%s）：%s", 
            enable ? "开启" : "关闭", result->message.c_str());
        return;
    }

    // 7. 操作成功
    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), 
        "软限位设置成功，启用状态：%s", 
        enable ? "开启" : "关闭");
}

// 设置运动控制模式

} // namespace rokae::ros2
