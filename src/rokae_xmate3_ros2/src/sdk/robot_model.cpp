#include "robot_internal.hpp"

namespace rokae::ros2 {

void xMateRobot::setRtControlMode(rokae::RtControllerMode mode, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }
    if (!impl_->wait_for_service(impl_->xmate3_rt_set_control_mode_client_, ec)) {
        return;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetRtControlMode::Request>();
    request->mode = static_cast<int32_t>(mode);
    auto future = impl_->xmate3_rt_set_control_mode_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置实时控制模式失败: %s", result->error_msg.c_str());
        return;
    }
    ec.clear();
}

bool xMateRobot::getRtJointData(std::array<double, 6>& position,
                                std::array<double, 6>& velocity,
                                std::array<double, 6>& torque,
                                std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_rt_get_joint_data_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetRtJointData::Request>();
    auto future = impl_->xmate3_rt_get_joint_data_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }
    for (size_t i = 0; i < 6; ++i) {
        position[i] = result->joint_position[i];
        velocity[i] = result->joint_velocity[i];
        torque[i] = result->joint_torque[i];
    }
    ec.clear();
    return true;
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

void xMateRobot::setAvoidSingularity(bool enable, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }
    if (!impl_->wait_for_service(impl_->xmate3_cobot_set_avoid_singularity_client_, ec)) {
        return;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetAvoidSingularity::Request>();
    request->enable = enable;
    auto future = impl_->xmate3_cobot_set_avoid_singularity_client_->async_send_request(request);
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

bool xMateRobot::getAvoidSingularity(std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_cobot_get_avoid_singularity_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetAvoidSingularity::Request>();
    auto future = impl_->xmate3_cobot_get_avoid_singularity_client_->async_send_request(request);
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
    return result->enabled;
}

std::array<double, 6> xMateRobot::getEndTorque(std::error_code& ec) {
    std::array<double, 6> wrench{};
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return wrench;
    }
    if (!impl_->wait_for_service(impl_->xmate3_cobot_get_end_torque_client_, ec)) {
        return wrench;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetEndTorque::Request>();
    auto future = impl_->xmate3_cobot_get_end_torque_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return wrench;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return wrench;
    }
    for (size_t i = 0; i < 6; ++i) {
        wrench[i] = result->end_torque[i];
    }
    ec.clear();
    return wrench;
}

bool xMateRobot::calcJointTorque(const std::array<double, 6>& joint_pos,
                                 const std::array<double, 6>& joint_vel,
                                 const std::array<double, 6>& joint_acc,
                                 const std::array<double, 6>& external_force,
                                 std::array<double, 6>& joint_torque,
                                 std::array<double, 6>& gravity_torque,
                                 std::array<double, 6>& coriolis_torque,
                                 std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_dyn_calc_joint_torque_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::CalcJointTorque::Request>();
    request->joint_pos = joint_pos;
    request->joint_vel = joint_vel;
    request->joint_acc = joint_acc;
    request->external_force = external_force;
    auto future = impl_->xmate3_dyn_calc_joint_torque_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }
    for (size_t i = 0; i < 6; ++i) {
        joint_torque[i] = result->joint_torque[i];
        gravity_torque[i] = result->gravity_torque[i];
        coriolis_torque[i] = result->coriolis_torque[i];
    }
    ec.clear();
    return true;
}

bool xMateRobot::generateSTrajectory(const std::array<double, 6>& start_joint_pos,
                                     const std::array<double, 6>& target_joint_pos,
                                     std::vector<std::array<double, 6>>& trajectory_points,
                                     double& total_time,
                                     std::error_code& ec) {
    trajectory_points.clear();
    total_time = 0.0;
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_dyn_generate_s_trajectory_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::GenerateSTrajectory::Request>();
    request->start_joint_pos = start_joint_pos;
    request->target_joint_pos = target_joint_pos;
    request->max_velocity = 1.0;
    request->max_acceleration = 0.5;
    request->blend_radius = 0.01;
    request->is_cartesian = false;
    auto future = impl_->xmate3_dyn_generate_s_trajectory_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }
    total_time = result->total_time;
    trajectory_points.reserve(result->trajectory_points.size());
    for (const auto& point : result->trajectory_points) {
        trajectory_points.push_back(point.pos);
    }
    ec.clear();
    return true;
}

bool xMateRobot::mapCartesianToJointTorque(const std::array<double, 6>& cart_force,
                                           const std::array<double, 6>& joint_pos,
                                           std::array<double, 6>& joint_torque,
                                           std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_dyn_map_cartesian_to_joint_torque_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::MapCartesianToJointTorque::Request>();
    request->cart_force = cart_force;
    request->joint_pos = joint_pos;
    auto future = impl_->xmate3_dyn_map_cartesian_to_joint_torque_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }
    for (size_t i = 0; i < 6; ++i) {
        joint_torque[i] = result->joint_torque[i];
    }
    ec.clear();
    return true;
}

// moveAppend 各类型重载
namespace {
constexpr size_t kMoveAppendMaxCount = 100;
constexpr size_t kExecuteCommandMaxCount = 1000;
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

// executeCommand 各类型重载
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

// getStateData 非模板版本
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

// 事件回调设置
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

// 最大缓存大小设置
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

// 实时状态接收
void xMateRobot::startReceiveRobotState(std::chrono::steady_clock::duration interval, const std::vector<std::string>& fields) {
    std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
    impl_->state_fields_ = fields;
    impl_->state_interval_ = interval;
    impl_->state_cache_.clear();
    impl_->has_previous_state_ = false;
}

void xMateRobot::stopReceiveRobotState() noexcept {
    std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
    impl_->state_fields_.clear();
    impl_->state_cache_.clear();
    impl_->has_previous_state_ = false;
}

unsigned xMateRobot::updateRobotState(std::chrono::steady_clock::duration timeout) {
    (void)timeout;
    std::error_code ec;
    // 更新状态缓存（简化版本）
    std::lock_guard<std::mutex> lock(impl_->state_cache_mutex_);
    if (impl_->state_fields_.empty()) {
        return 0;
    }
    // 获取关节数据
    auto joints = jointPos(ec);
    if (!ec) {
        auto joint_vel = jointVel(ec);
        auto joint_tau = jointTorque(ec);
        for (const auto& field : impl_->state_fields_) {
            if (field == rokae::RtSupportedFields::jointPos_m) {
                impl_->state_cache_[field] = joints;
            } else if (field == rokae::RtSupportedFields::jointVel_m) {
                impl_->state_cache_[field] = joint_vel;
            } else if (field == rokae::RtSupportedFields::tau_m) {
                impl_->state_cache_[field] = joint_tau;
            }
        }
    }
    return static_cast<unsigned>(impl_->state_cache_.size());
}

// 模型和实时控制器访问（占位实现）
std::shared_ptr<void> xMateRobot::model() {
    if (!impl_->model_) {
        impl_->model_ = std::make_shared<rokae::ros2::XMateModel>(*this);
    }
    return impl_->model_;
}

std::weak_ptr<void> xMateRobot::getRtMotionController() {
    if (!impl_->rt_controller_) {
        impl_->rt_controller_ = std::make_shared<int>(0);
    }
    return impl_->rt_controller_;
}

// 软限位SDK兼容版本
bool xMateRobot::getSoftLimit(std::array<double[2], 6>& limits, std::error_code& ec) {
    std::array<std::array<double, 2>, 6> tmp{};
    bool enabled = getSoftLimit(tmp, ec);
    if (!ec) {
        for (size_t i = 0; i < 6; ++i) {
            limits[i][0] = tmp[i][0];
            limits[i][1] = tmp[i][1];
        }
    }
    return enabled;
}

void xMateRobot::setSoftLimit(bool enable, std::error_code& ec, const std::array<double[2], 6>& limits) {
    std::array<std::array<double, 2>, 6> tmp{};
    for (size_t i = 0; i < 6; ++i) {
        tmp[i][0] = limits[i][0];
        tmp[i][1] = limits[i][1];
    }
    setSoftLimit(enable, ec, tmp);
}

// 坐标系标定 - 简化实现
rokae::FrameCalibrationResult xMateRobot::calibrateFrame(rokae::FrameType type,
                                                          const std::vector<std::array<double, 6>>& points,
                                                          bool is_held,
                                                          std::error_code& ec,
                                                          const std::array<double, 3>& base_aux) {
    rokae::FrameCalibrationResult result{};
    (void)is_held;
    (void)base_aux;

    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return result;
    }
    if (points.size() < 3) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return result;
    }

    Eigen::Vector3d mean_translation = Eigen::Vector3d::Zero();
    Eigen::Vector4d quaternion_accumulator = Eigen::Vector4d::Zero();
    for (const auto &point : points) {
        mean_translation += Eigen::Vector3d(point[0], point[1], point[2]);
        const auto quat = rokae_xmate3_ros2::runtime::pose_utils::rpyToQuaternion(point[3], point[4], point[5]);
        quaternion_accumulator += Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
    }
    mean_translation /= static_cast<double>(points.size());
    if (quaternion_accumulator.norm() < 1e-9) {
        quaternion_accumulator = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    }
    quaternion_accumulator.normalize();
    const Eigen::Quaterniond average_quaternion(
        quaternion_accumulator[0], quaternion_accumulator[1], quaternion_accumulator[2], quaternion_accumulator[3]);
    Eigen::Isometry3d calibrated_transform = Eigen::Isometry3d::Identity();
    calibrated_transform.translation() = mean_translation;
    calibrated_transform.linear() = average_quaternion.toRotationMatrix();
    const auto calibrated_pose = rokae_xmate3_ros2::runtime::pose_utils::isometryToPose(calibrated_transform);
    std::array<double, 6> calibrated_frame{};
    for (size_t i = 0; i < calibrated_frame.size() && i < calibrated_pose.size(); ++i) {
        calibrated_frame[i] = calibrated_pose[i];
    }
    result.frame = rokae::Frame(calibrated_frame);

    double translation_error = 0.0;
    double orientation_error = 0.0;
    for (const auto &point : points) {
        const Eigen::Vector3d translation(point[0], point[1], point[2]);
        translation_error += (translation - mean_translation).norm();
        orientation_error += rokae_xmate3_ros2::runtime::pose_utils::angularDistance(
            std::vector<double>(point.begin(), point.end()), calibrated_pose);
    }
    translation_error /= static_cast<double>(points.size());
    orientation_error /= static_cast<double>(points.size());

    result.success = true;
    result.errors = {translation_error, orientation_error, 0.0};
    ec.clear();

    // 更新缓存
    {
        std::lock_guard<std::mutex> lock(impl_->state_mutex_);
        if (type == rokae::FrameType::tool) {
            impl_->toolset_cache_.end = result.frame;
        } else if (type == rokae::FrameType::wobj || type == rokae::FrameType::base) {
            impl_->toolset_cache_.ref = result.frame;
        }
    }

    return result;
}


} // namespace rokae::ros2
