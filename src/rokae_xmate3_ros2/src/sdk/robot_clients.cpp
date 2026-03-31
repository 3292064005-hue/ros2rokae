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
    xmate3_internal_get_profile_capabilities_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetProfileCapabilities>("/xmate3/internal/get_profile_capabilities");
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
    xmate3_comm_read_register_ex_client_ = node_->create_client<rokae_xmate3_ros2::srv::ReadRegisterEx>("/xmate3/cobot/read_register_ex");
    xmate3_comm_write_register_client_ = node_->create_client<rokae_xmate3_ros2::srv::WriteRegister>("/xmate3/cobot/write_register");
    xmate3_comm_write_register_ex_client_ = node_->create_client<rokae_xmate3_ros2::srv::WriteRegisterEx>("/xmate3/cobot/write_register_ex");
    xmate3_comm_set_xpanel_vout_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetXPanelVout>("/xmate3/cobot/set_xpanel_vout");

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
    xmate3_rl_pause_project_client_ = node_->create_client<rokae_xmate3_ros2::srv::PauseRLProject>("/xmate3/cobot/pause_rl_project");
    xmate3_rl_set_running_opt_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetProjectRunningOpt>("/xmate3/cobot/set_project_running_opt");
    xmate3_rl_get_project_info_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetRlProjectInfo>("/xmate3/cobot/get_rl_project_info");
    xmate3_rl_get_tools_info_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetToolCatalog>("/xmate3/cobot/get_tools_info");
    xmate3_rl_get_wobjs_info_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetWobjCatalog>("/xmate3/cobot/get_wobjs_info");
    xmate3_cobot_set_avoid_singularity_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetAvoidSingularity>("/xmate3/cobot/set_avoid_singularity");
    xmate3_cobot_get_avoid_singularity_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetAvoidSingularity>("/xmate3/cobot/get_avoid_singularity");
    xmate3_cobot_get_end_torque_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetEndEffectorTorque>("/xmate3/cobot/get_end_torque");
    xmate3_cobot_get_end_wrench_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetEndWrench>("/xmate3/cobot/get_end_wrench");
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

} // namespace rokae::ros2
