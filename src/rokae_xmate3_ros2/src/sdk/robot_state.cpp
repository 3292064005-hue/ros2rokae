#include "robot_internal.hpp"

namespace rokae::ros2 {

int xMateRobot::getStateDataArray6(const std::string& fieldName, std::array<double, 6>& data) {
    std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
    auto it = impl_->state_cache_.find(fieldName);
    if (it == impl_->state_cache_.end()) {
        return -1;
    }
    try {
        data = std::any_cast<std::array<double, 6>>(it->second);
        return 0;
    } catch (const std::bad_any_cast&) {
        return -1;
    }
}

void xMateRobot::setEventWatcher(rokae::Event eventType, const rokae::EventCallback& callback, std::error_code& ec) {
    std::lock_guard<std::mutex> lock(impl_->event_mutex_);
    if (eventType == rokae::Event::moveExecution) {
        impl_->move_event_watcher_ = callback;
    } else {
        impl_->safety_event_watcher_ = callback;
    }
    ec.clear();
}

rokae::EventInfo xMateRobot::queryEventInfo(rokae::Event eventType, std::error_code& ec) {
    std::lock_guard<std::mutex> lock(impl_->event_mutex_);
    ec.clear();
    return eventType == rokae::Event::moveExecution ? impl_->last_move_event_ : impl_->last_safety_event_;
}

void xMateRobot::setMaxCacheSize(int number, std::error_code& ec) {
    if (number <= 0) {
        ec = std::make_error_code(std::errc::invalid_argument);
    } else {
        impl_->max_cache_size_ = number;
        ec.clear();
    }
}

std::error_code xMateRobot::lastErrorCode() noexcept {
    return impl_->last_error_code_;
}

std::array<double, 6> xMateRobot::jointPos(std::error_code& ec) {
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
    return jointTorques(ec);
}

// 获取末端位姿
std::array<double, 6> xMateRobot::posture(rokae::CoordinateType ct, std::error_code& ec) {
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
    std::array<double, 6> frame = {0.0};
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
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
rokae::JointPosition xMateRobot::calcIk(const rokae::CartesianPosition& posture, std::error_code& ec) {
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
    rokae::Toolset toolset;
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return toolset;
    }

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
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_robot_set_toolset_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetToolset::Request>();
    request->tool_name = toolset.tool_name;
    request->wobj_name = toolset.wobj_name;
    const std::array<double, 6> tool_pose = {toolset.end.x, toolset.end.y, toolset.end.z, toolset.end.rx, toolset.end.ry, toolset.end.rz};
    const std::array<double, 6> wobj_pose = {toolset.ref.x, toolset.ref.y, toolset.ref.z, toolset.ref.rx, toolset.ref.ry, toolset.ref.rz};
    request->tool_pose.assign(tool_pose.begin(), tool_pose.end());
    request->wobj_pose.assign(wobj_pose.begin(), wobj_pose.end());

    auto future = impl_->xmate3_robot_set_toolset_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置工具工件组失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    {
        std::lock_guard<std::mutex> lock(impl_->state_mutex_);
        impl_->toolset_cache_ = toolset;
    }
    RCLCPP_INFO(impl_->node_->get_logger(), "工具工件组设置成功");
}

// 按名称设置工具工件组
void xMateRobot::setToolset(const std::string& toolName, const std::string& wobjName, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

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

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "工具工件组设置成功, 工具: %s, 工件: %s", toolName.c_str(), wobjName.c_str());
}

// 开启碰撞检测
void xMateRobot::enableCollisionDetection(const std::array<double, 6>& sensitivity, rokae::StopLevel behaviour, double fallback, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

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
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

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
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }

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
    // 1. 前置校验：机器人连接状态
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        RCLCPP_WARN(impl_->node_->get_logger(), "设置软限位失败：机器人未连接");
        return;
    }

    // 2. 前置校验：限位值合理性（下限必须小于上限）
    for (int i = 0; i < 6; ++i) {
        if (limits[i][0] >= limits[i][1]) {
            ec = std::make_error_code(std::errc::invalid_argument);
            RCLCPP_ERROR(impl_->node_->get_logger(), 
                "设置软限位失败：第%d关节限位值无效（下限%.3f ≥ 上限%.3f）", 
                i+1, limits[i][0], limits[i][1]);
            return;
        }
    }

    // 3. 等待服务可用
    if (!impl_->wait_for_service(impl_->xmate3_robot_set_soft_limit_client_, ec)) {
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置软限位失败：SetSoftLimit服务不可用");
        return;
    }

    // 4. 构造请求：二维数组转一维数组（适配ROS2的float64[12]）
    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetSoftLimit::Request>();
    request->enable = enable;
    for (int i = 0; i < 6; ++i) {
        // 关键修改：一维数组索引计算（i*2=下限，i*2+1=上限）
        request->limits[i * 2] = limits[i][0];    // 第i+1关节下限
        request->limits[i * 2 + 1] = limits[i][1];// 第i+1关节上限
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
