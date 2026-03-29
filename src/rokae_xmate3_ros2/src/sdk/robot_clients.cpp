#include "robot_internal.hpp"

namespace rokae::ros2 {

void xMateRobot::Impl::init_node() {
    RCLCPP_INFO(node_->get_logger(), "xMate3 ROS2节点初始化完成, 节点名: %s", node_->get_name());
}

void xMateRobot::Impl::init_clients() {
    // 基础连接与信息 - 使用与Gazebo插件一致的路径 /xmate3/cobot/
    xmate3_robot_connect_client_ = node_->create_client<rokae_xmate3_ros2::srv::Connect>("/xmate3/cobot/connect");
    xmate3_robot_disconnect_client_ = node_->create_client<rokae_xmate3_ros2::srv::Disconnect>("/xmate3/cobot/disconnect");
    xmate3_robot_get_info_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetInfo>("/xmate3/cobot/get_info");
    xmate3_robot_get_power_state_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetPowerState>("/xmate3/cobot/get_power_state");
    xmate3_robot_set_power_state_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetPowerState>("/xmate3/cobot/set_power_state");
    xmate3_robot_get_operate_mode_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetOperateMode>("/xmate3/cobot/get_operate_mode");
    xmate3_robot_set_operate_mode_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetOperateMode>("/xmate3/cobot/set_operate_mode");
    xmate3_robot_query_log_client_ = node_->create_client<rokae_xmate3_ros2::srv::QueryControllerLog>("/xmate3/cobot/query_controller_log");
    xmate3_robot_clear_servo_alarm_client_ = node_->create_client<rokae_xmate3_ros2::srv::ClearServoAlarm>("/xmate3/cobot/clear_servo_alarm");

    // 关节与位姿
    xmate3_robot_get_joint_pos_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetJointPos>("/xmate3/cobot/get_joint_pos");
    xmate3_robot_get_joint_vel_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetJointVel>("/xmate3/cobot/get_joint_vel");
    xmate3_robot_get_joint_torque_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetJointTorques>("/xmate3/cobot/get_joint_torque");
    xmate3_robot_get_posture_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetPosture>("/xmate3/cobot/get_posture");
    xmate3_robot_get_cart_posture_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetCartPosture>("/xmate3/cobot/get_cart_posture");
    xmate3_robot_get_base_frame_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetBaseFrame>("/xmate3/cobot/get_base_frame");
    xmate3_robot_calc_ik_client_ = node_->create_client<rokae_xmate3_ros2::srv::CalcIk>("/xmate3/cobot/calc_ik");
    xmate3_robot_calc_fk_client_ = node_->create_client<rokae_xmate3_ros2::srv::CalcFk>("/xmate3/cobot/calc_fk");

    // 工具与坐标系
    xmate3_robot_get_toolset_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetToolset>("/xmate3/cobot/get_toolset");
    xmate3_robot_set_toolset_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetToolset>("/xmate3/cobot/set_toolset");
    xmate3_robot_set_toolset_by_name_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetToolsetByName>("/xmate3/cobot/set_toolset_by_name");

    // 安全相关
    xmate3_robot_enable_collision_detection_client_ = node_->create_client<rokae_xmate3_ros2::srv::EnableCollisionDetection>("/xmate3/cobot/enable_collision_detection");
    xmate3_robot_disable_collision_detection_client_ = node_->create_client<rokae_xmate3_ros2::srv::DisableCollisionDetection>("/xmate3/cobot/disable_collision_detection");
    xmate3_robot_get_soft_limit_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetSoftLimit>("/xmate3/cobot/get_soft_limit");
    xmate3_robot_set_soft_limit_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetSoftLimit>("/xmate3/cobot/set_soft_limit");

    // 非实时运动控制（统一到 cobot 命名空间）
    xmate3_motion_set_control_mode_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetMotionControlMode>("/xmate3/cobot/set_motion_control_mode");
    xmate3_motion_reset_client_ = node_->create_client<rokae_xmate3_ros2::srv::MoveReset>("/xmate3/cobot/move_reset");
    xmate3_motion_start_client_ = node_->create_client<rokae_xmate3_ros2::srv::MoveStart>("/xmate3/cobot/move_start");
    xmate3_motion_stop_client_ = node_->create_client<rokae_xmate3_ros2::srv::Stop>("/xmate3/cobot/stop");
    xmate3_motion_set_default_speed_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetDefaultSpeed>("/xmate3/cobot/set_default_speed");
    xmate3_motion_set_default_zone_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetDefaultZone>("/xmate3/cobot/set_default_zone");
    xmate3_motion_set_default_conf_opt_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetDefaultConfOpt>("/xmate3/cobot/set_default_conf_opt");
    xmate3_motion_adjust_speed_online_client_ = node_->create_client<rokae_xmate3_ros2::srv::AdjustSpeedOnline>("/xmate3/cobot/adjust_speed_online");

    // 实时控制/高级数据
    xmate3_rt_set_control_mode_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetRtControlMode>("/xmate3/cobot/set_rt_control_mode");
    xmate3_rt_get_joint_data_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetRtJointData>("/xmate3/cobot/get_rt_joint_data");
    xmate3_comm_send_custom_data_client_ = node_->create_client<rokae_xmate3_ros2::srv::SendCustomData>("/xmate3/cobot/send_custom_data");
    xmate3_comm_register_data_callback_client_ = node_->create_client<rokae_xmate3_ros2::srv::RegisterDataCallback>("/xmate3/cobot/register_data_callback");
    xmate3_comm_read_register_client_ = node_->create_client<rokae_xmate3_ros2::srv::ReadRegister>("/xmate3/cobot/read_register");
    xmate3_comm_write_register_client_ = node_->create_client<rokae_xmate3_ros2::srv::WriteRegister>("/xmate3/cobot/write_register");

    // IO控制
    xmate3_io_get_di_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetDI>("/xmate3/io/get_di");
    xmate3_io_get_do_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetDO>("/xmate3/io/get_do");
    xmate3_io_set_di_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetDI>("/xmate3/io/set_di");
    xmate3_io_set_do_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetDO>("/xmate3/io/set_do");
    xmate3_io_get_ai_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetAI>("/xmate3/io/get_ai");
    xmate3_io_set_ao_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetAO>("/xmate3/io/set_ao");

    xmate3_io_set_simulation_mode_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetSimulationMode>("/xmate3/io/set_simulation_mode");
    xmate3_rl_load_project_client_ = node_->create_client<rokae_xmate3_ros2::srv::LoadRLProject>("/xmate3/cobot/load_rl_project");
    xmate3_rl_start_project_client_ = node_->create_client<rokae_xmate3_ros2::srv::StartRLProject>("/xmate3/cobot/start_rl_project");
    xmate3_rl_stop_project_client_ = node_->create_client<rokae_xmate3_ros2::srv::StopRLProject>("/xmate3/cobot/stop_rl_project");
    xmate3_cobot_set_avoid_singularity_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetAvoidSingularity>("/xmate3/cobot/set_avoid_singularity");
    xmate3_cobot_get_avoid_singularity_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetAvoidSingularity>("/xmate3/cobot/get_avoid_singularity");
    xmate3_cobot_get_end_torque_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetEndEffectorTorque>("/xmate3/cobot/get_end_torque");
    xmate3_dyn_calc_joint_torque_client_ = node_->create_client<rokae_xmate3_ros2::srv::CalcJointTorque>("/xmate3/cobot/calc_joint_torque");
    xmate3_dyn_generate_s_trajectory_client_ = node_->create_client<rokae_xmate3_ros2::srv::GenerateSTrajectory>("/xmate3/cobot/generate_s_trajectory");
    xmate3_dyn_map_cartesian_to_joint_torque_client_ = node_->create_client<rokae_xmate3_ros2::srv::MapCartesianToJointTorque>("/xmate3/cobot/map_cartesian_to_joint_torque");

    // 拖动与路径录制
    xmate3_cobot_enable_drag_client_ = node_->create_client<rokae_xmate3_ros2::srv::EnableDrag>("/xmate3/cobot/enable_drag");
    
    // MoveAppend action 客户端，用于缓存非实时运动指令
    move_append_action_client_ = rclcpp_action::create_client<rokae_xmate3_ros2::action::MoveAppend>(
        node_, "/xmate3/cobot/move_append");
    xmate3_cobot_disable_drag_client_ = node_->create_client<rokae_xmate3_ros2::srv::DisableDrag>("/xmate3/cobot/disable_drag");
    xmate3_cobot_start_record_path_client_ = node_->create_client<rokae_xmate3_ros2::srv::StartRecordPath>("/xmate3/cobot/start_record_path");
    xmate3_cobot_stop_record_path_client_ = node_->create_client<rokae_xmate3_ros2::srv::StopRecordPath>("/xmate3/cobot/stop_record_path");
    xmate3_cobot_cancel_record_path_client_ = node_->create_client<rokae_xmate3_ros2::srv::CancelRecordPath>("/xmate3/cobot/cancel_record_path");
    xmate3_cobot_save_record_path_client_ = node_->create_client<rokae_xmate3_ros2::srv::SaveRecordPath>("/xmate3/cobot/save_record_path");
    xmate3_cobot_replay_path_client_ = node_->create_client<rokae_xmate3_ros2::srv::ReplayPath>("/xmate3/cobot/replay_path");
    xmate3_cobot_remove_path_client_ = node_->create_client<rokae_xmate3_ros2::srv::RemovePath>("/xmate3/cobot/remove_path");
    xmate3_cobot_query_path_lists_client_ = node_->create_client<rokae_xmate3_ros2::srv::QueryPathLists>("/xmate3/cobot/query_path_lists");

    RCLCPP_INFO(node_->get_logger(), "所有ROS2服务客户端初始化完成");
}

void xMateRobot::Impl::init_subscribers() {
    // 订阅关节状态（加锁保证线程安全）
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/xmate3/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_joint_state_ = *msg;
        });

    // 订阅机器人运行状态
    operation_state_sub_ = node_->create_subscription<rokae_xmate3_ros2::msg::OperationState>(
        "/xmate3/cobot/operation_state", 10,
        [this](const rokae_xmate3_ros2::msg::OperationState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_operation_state_ = *msg;
        });

    RCLCPP_INFO(node_->get_logger(), "所有ROS2话题订阅器初始化完成");
}

void xMateRobot::Impl::start_executor() {
    // Keep the ROS2 wrapper on explicit spin calls.
    // This avoids the node being attached to multiple executors when examples mix
    // service/action waits and state polling from different threads.
}

void xMateRobot::Impl::stop_executor() {
    if (executor_) {
        executor_->cancel();
    }
    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }
    if (executor_ && node_) {
        executor_->remove_node(node_);
    }
    executor_.reset();
}

xMateRobot::Impl::~Impl() {
    stop_executor();
}

// Impl构造函数实现
xMateRobot::Impl::Impl(const std::string& node_name) {
    // 首先初始化ROS2（如果还没有初始化）
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    // 然后创建节点
    node_ = rclcpp::Node::make_shared(node_name);
    init_node();
    init_clients();
    init_subscribers();
    start_executor();
}

xMateRobot::Impl::Impl(const std::string& remote_ip, const std::string& local_ip) {
    remote_ip_ = remote_ip;
    local_ip_ = local_ip;
    // 首先初始化ROS2（如果还没有初始化）
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    // 然后创建节点
    node_ = rclcpp::Node::make_shared("xmate3_robot");
    init_node();
    init_clients();
    init_subscribers();
    start_executor();
}

// 通用服务等待工具函数
bool xMateRobot::Impl::wait_for_service(rclcpp::ClientBase::SharedPtr client, std::error_code& ec, int timeout_s) {
    if (!client->wait_for_service(std::chrono::seconds(timeout_s))) {
        ec = std::make_error_code(std::errc::host_unreachable);
        RCLCPP_ERROR(node_->get_logger(), "服务 %s 等待超时", client->get_service_name());
        return false;
    }
    ec.clear();
    return true;
}

void xMateRobot::Impl::pump_callbacks() {
    if (!node_ || !rclcpp::ok()) {
        return;
    }
    std::lock_guard<std::mutex> lock(ros_call_mutex_);
    rclcpp::spin_some(node_);
}

void xMateRobot::Impl::resetMoveAppendState() {
    std::lock_guard<std::mutex> lock(action_mutex_);
    move_append_result_ready_ = false;
    move_append_result_code_ = rclcpp_action::ResultCode::UNKNOWN;
    move_append_result_success_ = true;
    move_append_result_message_.clear();
}

void xMateRobot::Impl::handleMoveAppendResult(
    const rclcpp_action::ClientGoalHandle<rokae_xmate3_ros2::action::MoveAppend>::WrappedResult &result) {
    {
        std::lock_guard<std::mutex> lock(action_mutex_);
        const auto result_message = result.result ? result.result->message : std::string();
        const bool expected_stop_result =
            suppress_next_stopped_move_append_result_ &&
            (result_message == "stop service called" || result.code == rclcpp_action::ResultCode::CANCELED);
        if (expected_stop_result) {
            suppress_next_stopped_move_append_result_ = false;
            move_append_result_ready_ = false;
            move_append_result_code_ = rclcpp_action::ResultCode::UNKNOWN;
            move_append_result_success_ = true;
            move_append_result_message_.clear();
        } else {
            suppress_next_stopped_move_append_result_ = false;
            move_append_result_ready_ = true;
            move_append_result_code_ = result.code;
            move_append_result_success_ = result.result ? result.result->success : false;
            move_append_result_message_ = result_message;
            if (move_append_result_message_.empty()) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::ABORTED:
                        move_append_result_message_ = "MoveAppend action aborted";
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        move_append_result_message_ = "MoveAppend action canceled";
                        break;
                    default:
                        break;
                }
            }
        }
    }
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        active_goal_handles_.clear();
    }
}

bool xMateRobot::Impl::checkMoveAppendFailure(std::error_code &ec) {
    std::string message;
    rclcpp_action::ResultCode result_code = rclcpp_action::ResultCode::UNKNOWN;
    bool result_success = true;
    {
        std::lock_guard<std::mutex> lock(action_mutex_);
        if (!move_append_result_ready_) {
            return false;
        }
        result_code = move_append_result_code_;
        result_success = move_append_result_success_;
        message = move_append_result_message_;
    }

    if (result_code == rclcpp_action::ResultCode::SUCCEEDED && result_success) {
        return false;
    }

    if (message.empty()) {
        message = "MoveAppend action failed";
    }
    ec = std::make_error_code(
        result_code == rclcpp_action::ResultCode::CANCELED ? std::errc::operation_canceled
                                                            : std::errc::operation_not_permitted);
    RCLCPP_ERROR(node_->get_logger(), "MoveAppend执行失败: %s", message.c_str());
    return true;
}

// ==================== xMateRobot对外接口实现 ====================
// 构造函数
xMateRobot::xMateRobot(const std::string& node_name)
    : impl_(std::make_shared<Impl>(node_name)) {}

xMateRobot::xMateRobot(const std::string& remote_ip, const std::string& local_ip)
    : impl_(std::make_shared<Impl>(remote_ip, local_ip)) {}

// 析构函数
xMateRobot::~xMateRobot() {
    std::error_code ec;
    if (impl_->connected_) {
        disconnectFromRobot(ec);
    }
    if (impl_) {
        impl_->stop_executor();
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

// 连接机器人
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

std::string xMateRobot::sendCustomData(const std::string& topic,
                                       const std::string& payload,
                                       std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return {};
    }
    if (!impl_->wait_for_service(impl_->xmate3_comm_send_custom_data_client_, ec)) {
        return {};
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::SendCustomData::Request>();
    request->data_topic = topic;
    request->custom_data = payload;
    auto future = impl_->xmate3_comm_send_custom_data_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return {};
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "发送自定义数据失败: %s", result->error_msg.c_str());
        return {};
    }
    ec.clear();
    return result->response_data;
}

bool xMateRobot::registerDataCallback(const std::string& data_topic,
                                      const std::string& callback_id,
                                      std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_comm_register_data_callback_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::RegisterDataCallback::Request>();
    request->data_topic = data_topic;
    request->callback_id = callback_id;
    auto future = impl_->xmate3_comm_register_data_callback_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "注册数据回调失败: %s", result->error_msg.c_str());
        return false;
    }
    ec.clear();
    return true;
}

std::string xMateRobot::readRegister(const std::string& key, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return {};
    }
    if (!impl_->wait_for_service(impl_->xmate3_comm_read_register_client_, ec)) {
        return {};
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::ReadRegister::Request>();
    request->key = key;
    auto future = impl_->xmate3_comm_read_register_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return {};
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return {};
    }
    ec.clear();
    return result->value;
}

void xMateRobot::writeRegister(const std::string& key, const std::string& value, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }
    if (!impl_->wait_for_service(impl_->xmate3_comm_write_register_client_, ec)) {
        return;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::WriteRegister::Request>();
    request->key = key;
    request->value = value;
    auto future = impl_->xmate3_comm_write_register_client_->async_send_request(request);
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

bool xMateRobot::loadRLProject(const std::string& project_path, std::string& project_name, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_rl_load_project_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::LoadRLProject::Request>();
    request->project_path = project_path;
    auto future = impl_->xmate3_rl_load_project_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "加载RL工程失败: %s", result->error_msg.c_str());
        return false;
    }
    project_name = result->project_name;
    ec.clear();
    return true;
}

bool xMateRobot::startRLProject(const std::string& project_id, int& current_episode, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_rl_start_project_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::StartRLProject::Request>();
    request->project_id = project_id;
    auto future = impl_->xmate3_rl_start_project_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }
    current_episode = result->current_episode;
    ec.clear();
    return true;
}

bool xMateRobot::stopRLProject(const std::string& project_id, int& finished_episode, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_rl_stop_project_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::StopRLProject::Request>();
    request->project_id = project_id;
    auto future = impl_->xmate3_rl_stop_project_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }
    finished_episode = result->finished_episode;
    ec.clear();
    return true;
}


} // namespace rokae::ros2
