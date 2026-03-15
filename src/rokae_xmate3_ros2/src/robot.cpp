#include "rokae_xmate3_ros2/robot.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <system_error>
#include <thread>
#include <algorithm>

using namespace std::chrono_literals;
using namespace rokae;

namespace rokae::ros2 {

// ==================== Impl类内部初始化函数实现 ====================
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
    xmate3_robot_get_joint_torque_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetJointTorque>("/xmate3/cobot/get_joint_torque");
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

    // IO控制
    xmate3_io_get_di_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetDI>("/xmate3/io/get_di");
    xmate3_io_get_do_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetDO>("/xmate3/io/get_do");
    xmate3_io_set_di_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetDI>("/xmate3/io/set_di");
    xmate3_io_set_do_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetDO>("/xmate3/io/set_do");
    xmate3_io_get_ai_client_ = node_->create_client<rokae_xmate3_ros2::srv::GetAI>("/xmate3/io/get_ai");
    xmate3_io_set_ao_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetAO>("/xmate3/io/set_ao");

    xmate3_io_set_simulation_mode_client_ = node_->create_client<rokae_xmate3_ros2::srv::SetSimulationMode>("/xmate3/io/set_simulation_mode");

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
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

// SDK版本号
std::string xMateRobot::sdkVersion() {
    return "2.1.0 (xCore SDK ROS2 Wrapper)";
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return rokae::PowerState::unknown;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return rokae::PowerState::unknown;
    }
    ec.clear();
    // 核心修正：从msg中取出uint8_t原始值再强转
    return static_cast<rokae::PowerState>(result->state.state);
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return rokae::OperationState::unknown;
    }
    ec.clear();
    return static_cast<rokae::OperationState>(impl_->last_operation_state_.state);
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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

// 获取关节位置
std::array<double, 6> xMateRobot::jointPos(std::error_code& ec) {
    std::array<double, 6> pos = {0.0};
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return pos;
    }
    if (!impl_->wait_for_service(impl_->xmate3_robot_get_joint_pos_client_, ec)) {
        return pos;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetJointPos::Request>();
    auto future = impl_->xmate3_robot_get_joint_pos_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return pos;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return pos;
    }
    // 核心修正：和.srv文件字段名严格匹配，如joint_positions
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

    if (!impl_->wait_for_service(impl_->xmate3_robot_get_joint_vel_client_, ec)) {
        return vel;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetJointVel::Request>();
    auto future = impl_->xmate3_robot_get_joint_vel_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
std::array<double, 6> xMateRobot::jointTorque(std::error_code& ec) {
    std::array<double, 6> torque = {0.0};
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return torque;
    }

    if (!impl_->wait_for_service(impl_->xmate3_robot_get_joint_torque_client_, ec)) {
        return torque;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetJointTorque::Request>();
    auto future = impl_->xmate3_robot_get_joint_torque_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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

// 获取末端位姿
std::array<double, 6> xMateRobot::posture(rokae::CoordinateType ct, std::error_code& ec) {
    std::array<double, 6> pose = {0.0};
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return pose;
    }

    if (!impl_->wait_for_service(impl_->xmate3_robot_get_posture_client_, ec)) {
        return pose;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetPosture::Request>();
    request->coordinate_type = static_cast<uint8_t>(ct);

    auto future = impl_->xmate3_robot_get_posture_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return pose;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return pose;
    }

    std::copy(result->posture.begin(), result->posture.end(), pose.begin());
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

    if (!impl_->wait_for_service(impl_->xmate3_robot_get_cart_posture_client_, ec)) {
        return cart_pose;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetCartPosture::Request>();
    request->coordinate_type = static_cast<uint8_t>(ct);

    auto future = impl_->xmate3_robot_get_cart_posture_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return cart_pose;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return cart_pose;
    }

    cart_pose.x = result->x;
    cart_pose.y = result->y;
    cart_pose.z = result->z;
    cart_pose.rx = result->rx;
    cart_pose.ry = result->ry;
    cart_pose.rz = result->rz;
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    
    auto future = impl_->xmate3_robot_calc_ik_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    joints.j1 = result->joint_positions[0];
    joints.j2 = result->joint_positions[1];
    joints.j3 = result->joint_positions[2];
    joints.j4 = result->joint_positions[3];
    joints.j5 = result->joint_positions[4];
    joints.j6 = result->joint_positions[5];
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
    request->joint_positions[0] = joints.j1;
    request->joint_positions[1] = joints.j2;
    request->joint_positions[2] = joints.j3;
    request->joint_positions[3] = joints.j4;
    request->joint_positions[4] = joints.j5;
    request->joint_positions[5] = joints.j6;
    
    auto future = impl_->xmate3_robot_calc_fk_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    std::copy(result->tool_pose.begin(), result->tool_pose.end(), toolset.tool_pose.begin());
    std::copy(result->wobj_pose.begin(), result->wobj_pose.end(), toolset.wobj_pose.begin());
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
    std::copy(toolset.tool_pose.begin(), toolset.tool_pose.end(), request->tool_pose.begin());
    std::copy(toolset.wobj_pose.begin(), toolset.wobj_pose.end(), request->wobj_pose.begin());

    auto future = impl_->xmate3_robot_set_toolset_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    std::copy(sensitivity.begin(), sensitivity.end(), request->sensitivity.begin());
    request->behaviour = static_cast<uint8_t>(behaviour);
    request->fallback = fallback;

    auto future = impl_->xmate3_robot_enable_collision_detection_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    auto spin_result = rclcpp::spin_until_future_complete(impl_->node_, future);
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
void xMateRobot::setMotionControlMode(rokae::MotionControlMode mode, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_motion_set_control_mode_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetMotionControlMode::Request>();
    request->mode = static_cast<uint8_t>(mode);

    auto future = impl_->xmate3_motion_set_control_mode_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "启动运动失败: %s", result->message.c_str());
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

    auto request = std::make_shared<rokae_xmate3_ros2::srv::Stop::Request>();
    auto future = impl_->xmate3_motion_stop_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (rclcpp::spin_until_future_complete(impl_->node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cached_goal_ = rokae_xmate3_ros2::action::MoveAppend::Goal();
    active_goal_handles_.clear();
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

    // 发送goal（异步，但等待goal被接受）
    auto send_future = move_append_action_client_->async_send_goal(goal_to_send);

    // 等待goal被接受
    if (rclcpp::spin_until_future_complete(node_, send_future, 5s) !=
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
    RCLCPP_INFO(node_->get_logger(), "等待运动开始...");
    auto start_wait = std::chrono::steady_clock::now();
    bool motion_started = false;
    while (std::chrono::steady_clock::now() - start_wait < std::chrono::seconds(2)) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (last_operation_state_.state == rokae_xmate3_ros2::msg::OperationState::MOVING) {
                motion_started = true;
                break;
            }
        }
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (motion_started) {
        RCLCPP_INFO(node_->get_logger(), "运动已开始!");
    } else {
        RCLCPP_WARN(node_->get_logger(), "等待运动开始超时，继续执行...");
    }

    ec.clear();
    return true;
}

// helper to convert application commands into action goal parts
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
    m.target.x = cmd.target.x;
    m.target.y = cmd.target.y;
    m.target.z = cmd.target.z;
    m.target.rx = cmd.target.rx;
    m.target.ry = cmd.target.ry;
    m.target.rz = cmd.target.rz;
    m.speed = cmd.speed;
    m.zone = cmd.zone;
    // offset omitted for simplicity
    return m;
}
static rokae_xmate3_ros2::msg::MoveLCommand toMsg(const rokae::MoveLCommand &cmd) {
    rokae_xmate3_ros2::msg::MoveLCommand m;
    m.target.x = cmd.target.x;
    m.target.y = cmd.target.y;
    m.target.z = cmd.target.z;
    m.target.rx = cmd.target.rx;
    m.target.ry = cmd.target.ry;
    m.target.rz = cmd.target.rz;
    m.speed = cmd.speed;
    m.zone = cmd.zone;
    return m;
}
static rokae_xmate3_ros2::msg::MoveCCommand toMsg(const rokae::MoveCCommand &cmd) {
    rokae_xmate3_ros2::msg::MoveCCommand m;
    m.target.x = cmd.target.x;
    m.target.y = cmd.target.y;
    m.target.z = cmd.target.z;
    m.target.rx = cmd.target.rx;
    m.target.ry = cmd.target.ry;
    m.target.rz = cmd.target.rz;
    m.aux.x = cmd.aux.x;
    m.aux.y = cmd.aux.y;
    m.aux.z = cmd.aux.z;
    m.aux.rx = cmd.aux.rx;
    m.aux.ry = cmd.aux.ry;
    m.aux.rz = cmd.aux.rz;
    m.speed = cmd.speed;
    m.zone = cmd.zone;
    return m;
}
static rokae_xmate3_ros2::msg::MoveCFCommand toMsg(const rokae::MoveCFCommand &cmd) {
    rokae_xmate3_ros2::msg::MoveCFCommand m;
    m.target.x = cmd.target.x;
    m.target.y = cmd.target.y;
    m.target.z = cmd.target.z;
    m.target.rx = cmd.target.rx;
    m.target.ry = cmd.target.ry;
    m.target.rz = cmd.target.rz;
    m.aux.x = cmd.aux.x;
    m.aux.y = cmd.aux.y;
    m.aux.z = cmd.aux.z;
    m.aux.rx = cmd.aux.rx;
    m.aux.ry = cmd.aux.ry;
    m.aux.rz = cmd.aux.rz;
    m.speed = cmd.speed;
    m.zone = cmd.zone;
    m.angle = cmd.angle;
    return m;
}
static rokae_xmate3_ros2::msg::MoveSPCommand toMsg(const rokae::MoveSPCommand &cmd) {
    rokae_xmate3_ros2::msg::MoveSPCommand m;
    m.target.x = cmd.target.x;
    m.target.y = cmd.target.y;
    m.target.z = cmd.target.z;
    m.target.rx = cmd.target.rx;
    m.target.ry = cmd.target.ry;
    m.target.rz = cmd.target.rz;
    m.speed = cmd.speed;
    m.zone = cmd.zone;
    m.radius = cmd.radius;
    m.radius_step = cmd.radius_step;
    m.angle = cmd.angle;
    m.direction = cmd.direction;
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
