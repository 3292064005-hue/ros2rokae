#include "runtime/ros_bindings.hpp"

#include <array>
#include <thread>

#include "runtime/runtime_publish_bridge.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

template <typename ServiceT, typename FacadeT>
rclcpp::ServiceBase::SharedPtr CreateFacadeService(
    const rclcpp::Node::SharedPtr &node,
    const std::string &name,
    FacadeT *facade,
    void (FacadeT::*method)(const typename ServiceT::Request &, typename ServiceT::Response &) const) {
  return node->create_service<ServiceT>(
      name,
      [facade, method](const std::shared_ptr<typename ServiceT::Request> req,
                       std::shared_ptr<typename ServiceT::Response> res) {
        (facade->*method)(*req, *res);
      });
}

bool is_retryable_submission_failure(const std::string &message) {
  return message == "runtime is busy with another motion request" ||
         message == "runtime planner queue is busy";
}

}  // namespace

RosBindings::RosBindings(rclcpp::Node::SharedPtr node,
                         RuntimeContext &runtime_context,
                         RuntimePublishBridge *publish_bridge,
                         gazebo::xMate3Kinematics &kinematics,
                         JointStateFetcher joint_state_fetcher,
                         TimeProvider time_provider,
                         TrajectoryDtProvider trajectory_dt_provider,
                         RequestIdGenerator request_id_generator)
    : node_(std::move(node)),
      runtime_context_(runtime_context),
      publish_bridge_(publish_bridge),
      joint_state_fetcher_(std::move(joint_state_fetcher)),
      trajectory_dt_provider_(std::move(trajectory_dt_provider)),
      request_id_generator_(std::move(request_id_generator)),
      control_facade_(std::make_unique<ControlFacade>(runtime_context_.sessionState(),
                                                      runtime_context_.motionOptionsState(),
                                                      runtime_context_.backend(),
                                                      &runtime_context_.motionRuntime(),
                                                      &runtime_context_.requestCoordinator())),
      query_facade_(std::make_unique<QueryFacade>(runtime_context_.sessionState(),
                                                  runtime_context_.motionOptionsState(),
                                                  runtime_context_.toolingState(),
                                                  runtime_context_.dataStoreState(),
                                                  runtime_context_.programState(),
                                                  kinematics,
                                                  joint_state_fetcher_,
                                                  std::move(time_provider),
                                                  trajectory_dt_provider_,
                                                  6)),
      io_program_facade_(std::make_unique<IoProgramFacade>(runtime_context_.dataStoreState(),
                                                           runtime_context_.programState(),
                                                           [this]() { return node_->get_clock()->now(); })),
      path_facade_(std::make_unique<PathFacade>(runtime_context_.programState(),
                                                runtime_context_.toolingState(),
                                                &runtime_context_.requestCoordinator(),
                                                joint_state_fetcher_,
                                                trajectory_dt_provider_,
                                                request_id_generator_)) {
  initServices();
  initActionServers();
}

void RosBindings::initServices() {
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::Connect>(
      node_, "/xmate3/cobot/connect", control_facade_.get(), &ControlFacade::handleConnect));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::Disconnect>(
      node_, "/xmate3/cobot/disconnect", control_facade_.get(), &ControlFacade::handleDisconnect));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetPowerState>(
      node_, "/xmate3/cobot/set_power_state", control_facade_.get(), &ControlFacade::handleSetPowerState));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetPowerState>(
      node_, "/xmate3/cobot/get_power_state", query_facade_.get(), &QueryFacade::handleGetPowerState));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetInfo>(
      node_, "/xmate3/cobot/get_info", query_facade_.get(), &QueryFacade::handleGetInfo));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetOperateMode>(
      node_, "/xmate3/cobot/get_operate_mode", query_facade_.get(), &QueryFacade::handleGetOperateMode));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetOperateMode>(
      node_, "/xmate3/cobot/set_operate_mode", control_facade_.get(), &ControlFacade::handleSetOperateMode));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::QueryControllerLog>(
      node_, "/xmate3/cobot/query_controller_log", query_facade_.get(), &QueryFacade::handleQueryControllerLog));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::ClearServoAlarm>(
      node_, "/xmate3/cobot/clear_servo_alarm", control_facade_.get(), &ControlFacade::handleClearServoAlarm));

  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetJointPos>(
      node_, "/xmate3/cobot/get_joint_pos", query_facade_.get(), &QueryFacade::handleGetJointPos));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetJointVel>(
      node_, "/xmate3/cobot/get_joint_vel", query_facade_.get(), &QueryFacade::handleGetJointVel));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetJointTorque>(
      node_, "/xmate3/cobot/get_joint_torque", query_facade_.get(), &QueryFacade::handleGetJointTorque));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetPosture>(
      node_, "/xmate3/cobot/get_posture", query_facade_.get(), &QueryFacade::handleGetPosture));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetCartPosture>(
      node_, "/xmate3/cobot/get_cart_posture", query_facade_.get(), &QueryFacade::handleGetCartPosture));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetBaseFrame>(
      node_, "/xmate3/cobot/get_base_frame", query_facade_.get(), &QueryFacade::handleGetBaseFrame));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::CalcFk>(
      node_, "/xmate3/cobot/calc_fk", query_facade_.get(), &QueryFacade::handleCalcFk));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::CalcIk>(
      node_, "/xmate3/cobot/calc_ik", query_facade_.get(), &QueryFacade::handleCalcIk));

  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetToolset>(
      node_, "/xmate3/cobot/get_toolset", query_facade_.get(), &QueryFacade::handleGetToolset));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetToolset>(
      node_, "/xmate3/cobot/set_toolset", query_facade_.get(), &QueryFacade::handleSetToolset));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetToolsetByName>(
      node_, "/xmate3/cobot/set_toolset_by_name", query_facade_.get(), &QueryFacade::handleSetToolsetByName));

  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::EnableCollisionDetection>(
      node_, "/xmate3/cobot/enable_collision_detection", control_facade_.get(), &ControlFacade::handleEnableCollisionDetection));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::DisableCollisionDetection>(
      node_, "/xmate3/cobot/disable_collision_detection", control_facade_.get(), &ControlFacade::handleDisableCollisionDetection));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetSoftLimit>(
      node_, "/xmate3/cobot/get_soft_limit", query_facade_.get(), &QueryFacade::handleGetSoftLimit));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetSoftLimit>(
      node_, "/xmate3/cobot/set_soft_limit", control_facade_.get(), &ControlFacade::handleSetSoftLimit));

  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetMotionControlMode>(
      node_, "/xmate3/cobot/set_motion_control_mode", control_facade_.get(), &ControlFacade::handleSetMotionControlMode));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::MoveReset>(
      node_, "/xmate3/cobot/move_reset", control_facade_.get(), &ControlFacade::handleMoveReset));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::MoveStart>(
      node_, "/xmate3/cobot/move_start", control_facade_.get(), &ControlFacade::handleMoveStart));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::Stop>(
      node_, "/xmate3/cobot/stop", control_facade_.get(), &ControlFacade::handleStop));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetDefaultSpeed>(
      node_, "/xmate3/cobot/set_default_speed", control_facade_.get(), &ControlFacade::handleSetDefaultSpeed));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetDefaultZone>(
      node_, "/xmate3/cobot/set_default_zone", control_facade_.get(), &ControlFacade::handleSetDefaultZone));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetDefaultConfOpt>(
      node_, "/xmate3/cobot/set_default_conf_opt", control_facade_.get(), &ControlFacade::handleSetDefaultConfOpt));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::AdjustSpeedOnline>(
      node_, "/xmate3/cobot/adjust_speed_online", control_facade_.get(), &ControlFacade::handleAdjustSpeedOnline));

  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetRtControlMode>(
      node_, "/xmate3/cobot/set_rt_control_mode", control_facade_.get(), &ControlFacade::handleSetRtControlMode));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetRtJointData>(
      node_, "/xmate3/cobot/get_rt_joint_data", query_facade_.get(), &QueryFacade::handleGetRtJointData));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SendCustomData>(
      node_, "/xmate3/cobot/send_custom_data", io_program_facade_.get(), &IoProgramFacade::handleSendCustomData));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::RegisterDataCallback>(
      node_, "/xmate3/cobot/register_data_callback", io_program_facade_.get(), &IoProgramFacade::handleRegisterDataCallback));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::ReadRegister>(
      node_, "/xmate3/cobot/read_register", io_program_facade_.get(), &IoProgramFacade::handleReadRegister));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::WriteRegister>(
      node_, "/xmate3/cobot/write_register", io_program_facade_.get(), &IoProgramFacade::handleWriteRegister));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::LoadRLProject>(
      node_, "/xmate3/cobot/load_rl_project", io_program_facade_.get(), &IoProgramFacade::handleLoadRlProject));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::StartRLProject>(
      node_, "/xmate3/cobot/start_rl_project", io_program_facade_.get(), &IoProgramFacade::handleStartRlProject));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::StopRLProject>(
      node_, "/xmate3/cobot/stop_rl_project", io_program_facade_.get(), &IoProgramFacade::handleStopRlProject));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetAvoidSingularity>(
      node_, "/xmate3/cobot/set_avoid_singularity", control_facade_.get(), &ControlFacade::handleSetAvoidSingularity));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetAvoidSingularity>(
      node_, "/xmate3/cobot/get_avoid_singularity", query_facade_.get(), &QueryFacade::handleGetAvoidSingularity));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::CalcJointTorque>(
      node_, "/xmate3/cobot/calc_joint_torque", query_facade_.get(), &QueryFacade::handleCalcJointTorque));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GenerateSTrajectory>(
      node_, "/xmate3/cobot/generate_s_trajectory", query_facade_.get(), &QueryFacade::handleGenerateSTrajectory));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::MapCartesianToJointTorque>(
      node_, "/xmate3/cobot/map_cartesian_to_joint_torque", query_facade_.get(), &QueryFacade::handleMapCartesianToJointTorque));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetEndTorque>(
      node_, "/xmate3/cobot/get_end_torque", query_facade_.get(), &QueryFacade::handleGetEndTorque));

  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetDI>(
      node_, "/xmate3/io/get_di", io_program_facade_.get(), &IoProgramFacade::handleGetDI));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetDO>(
      node_, "/xmate3/io/get_do", io_program_facade_.get(), &IoProgramFacade::handleGetDO));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetDI>(
      node_, "/xmate3/io/set_di", io_program_facade_.get(), &IoProgramFacade::handleSetDI));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetDO>(
      node_, "/xmate3/io/set_do", io_program_facade_.get(), &IoProgramFacade::handleSetDO));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetAI>(
      node_, "/xmate3/io/get_ai", io_program_facade_.get(), &IoProgramFacade::handleGetAI));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetAO>(
      node_, "/xmate3/io/set_ao", io_program_facade_.get(), &IoProgramFacade::handleSetAO));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetSimulationMode>(
      node_, "/xmate3/io/set_simulation_mode", control_facade_.get(), &ControlFacade::handleSetSimulationMode));

  compatibility_services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetDI>(
      node_, "/xmate3/cobot/get_di", io_program_facade_.get(), &IoProgramFacade::handleGetDI));
  compatibility_services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetDO>(
      node_, "/xmate3/cobot/get_do", io_program_facade_.get(), &IoProgramFacade::handleGetDO));
  compatibility_services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetDI>(
      node_, "/xmate3/cobot/set_di", io_program_facade_.get(), &IoProgramFacade::handleSetDI));
  compatibility_services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetDO>(
      node_, "/xmate3/cobot/set_do", io_program_facade_.get(), &IoProgramFacade::handleSetDO));
  compatibility_services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::GetAI>(
      node_, "/xmate3/cobot/get_ai", io_program_facade_.get(), &IoProgramFacade::handleGetAI));
  compatibility_services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetAO>(
      node_, "/xmate3/cobot/set_ao", io_program_facade_.get(), &IoProgramFacade::handleSetAO));
  compatibility_services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SetSimulationMode>(
      node_, "/xmate3/cobot/set_simulation_mode", control_facade_.get(), &ControlFacade::handleSetSimulationMode));

  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::EnableDrag>(
      node_, "/xmate3/cobot/enable_drag", control_facade_.get(), &ControlFacade::handleEnableDrag));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::DisableDrag>(
      node_, "/xmate3/cobot/disable_drag", control_facade_.get(), &ControlFacade::handleDisableDrag));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::StartRecordPath>(
      node_, "/xmate3/cobot/start_record_path", path_facade_.get(), &PathFacade::handleStartRecordPath));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::StopRecordPath>(
      node_, "/xmate3/cobot/stop_record_path", path_facade_.get(), &PathFacade::handleStopRecordPath));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::CancelRecordPath>(
      node_, "/xmate3/cobot/cancel_record_path", path_facade_.get(), &PathFacade::handleCancelRecordPath));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::SaveRecordPath>(
      node_, "/xmate3/cobot/save_record_path", path_facade_.get(), &PathFacade::handleSaveRecordPath));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::ReplayPath>(
      node_, "/xmate3/cobot/replay_path", path_facade_.get(), &PathFacade::handleReplayPath));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::RemovePath>(
      node_, "/xmate3/cobot/remove_path", path_facade_.get(), &PathFacade::handleRemovePath));
  services_.push_back(CreateFacadeService<rokae_xmate3_ros2::srv::QueryPathLists>(
      node_, "/xmate3/cobot/query_path_lists", path_facade_.get(), &PathFacade::handleQueryPathLists));
}

void RosBindings::initActionServers() {
  move_append_action_server_ = rclcpp_action::create_server<rokae_xmate3_ros2::action::MoveAppend>(
      node_,
      "/xmate3/cobot/move_append",
      [this](const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const rokae_xmate3_ros2::action::MoveAppend::Goal> goal) {
        (void)uuid;
        (void)goal;
        if (!runtime_context_.sessionState().powerOn()) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<rokae_xmate3_ros2::action::MoveAppend>> goal_handle) {
        (void)goal_handle;
        runtime_context_.requestCoordinator().stop("move append canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<rokae_xmate3_ros2::action::MoveAppend>> goal_handle) {
        std::thread([this, goal_handle]() { executeMoveAppend(goal_handle); }).detach();
      });
}

void RosBindings::executeMoveAppend(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<rokae_xmate3_ros2::action::MoveAppend>> &goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Result>();

  std::array<double, 6> cached_pos{};
  std::array<double, 6> cached_vel{};
  std::array<double, 6> cached_torque{};
  joint_state_fetcher_(cached_pos, cached_vel, cached_torque);

  const auto request_id = request_id_generator_("move_");
  SubmissionResult submission;
  constexpr auto kSubmissionRetryTimeout = std::chrono::milliseconds(1200);
  constexpr auto kSubmissionRetrySleep = std::chrono::milliseconds(10);
  const auto retry_deadline = std::chrono::steady_clock::now() + kSubmissionRetryTimeout;

  while (true) {
    joint_state_fetcher_(cached_pos, cached_vel, cached_torque);
    submission = runtime_context_.requestCoordinator().submitMoveAppend(
        *goal,
        cached_pos,
        trajectory_dt_provider_(),
        request_id);
    if (submission.success) {
      break;
    }
    if (!is_retryable_submission_failure(submission.message) ||
        std::chrono::steady_clock::now() >= retry_deadline) {
      break;
    }
    std::this_thread::sleep_for(kSubmissionRetrySleep);
  }

  if (!submission.success) {
    result->success = false;
    result->cmd_id = request_id;
    result->message = submission.message;
    goal_handle->abort(result);
    return;
  }

  if (publish_bridge_ != nullptr) {
    publish_bridge_->driveMoveAppendGoal(goal_handle, request_id);
    return;
  }

  result->success = false;
  result->cmd_id = request_id;
  result->message = "MoveAppend bridge unavailable";
  goal_handle->abort(result);
}

}  // namespace rokae_xmate3_ros2::runtime
