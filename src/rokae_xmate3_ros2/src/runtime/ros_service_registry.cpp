#include "runtime/ros_bindings.hpp"

#include "runtime/ros_service_factory.hpp"

namespace rokae_xmate3_ros2::runtime {

void RosBindings::initServices() {
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::Connect>(
      node_, "/xmate3/cobot/connect", control_facade_.get(), &ControlFacade::handleConnect));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::Disconnect>(
      node_, "/xmate3/cobot/disconnect", control_facade_.get(), &ControlFacade::handleDisconnect));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetPowerState>(
      node_, "/xmate3/cobot/set_power_state", control_facade_.get(), &ControlFacade::handleSetPowerState));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetPowerState>(
      node_, "/xmate3/cobot/get_power_state", query_facade_.get(), &QueryFacade::handleGetPowerState));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetInfo>(
      node_, "/xmate3/cobot/get_info", query_facade_.get(), &QueryFacade::handleGetInfo));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetOperateMode>(
      node_, "/xmate3/cobot/get_operate_mode", query_facade_.get(), &QueryFacade::handleGetOperateMode));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetOperateMode>(
      node_, "/xmate3/cobot/set_operate_mode", control_facade_.get(), &ControlFacade::handleSetOperateMode));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::QueryControllerLog>(
      node_, "/xmate3/cobot/query_controller_log", query_facade_.get(), &QueryFacade::handleQueryControllerLog));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::ClearServoAlarm>(
      node_, "/xmate3/cobot/clear_servo_alarm", control_facade_.get(), &ControlFacade::handleClearServoAlarm));

  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetJointPos>(
      node_, "/xmate3/cobot/get_joint_pos", query_facade_.get(), &QueryFacade::handleGetJointPos));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetJointVel>(
      node_, "/xmate3/cobot/get_joint_vel", query_facade_.get(), &QueryFacade::handleGetJointVel));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetJointTorques>(
      node_, "/xmate3/cobot/get_joint_torque", query_facade_.get(), &QueryFacade::handleGetJointTorques));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetPosture>(
      node_, "/xmate3/cobot/get_posture", query_facade_.get(), &QueryFacade::handleGetPosture));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetCartPosture>(
      node_, "/xmate3/cobot/get_cart_posture", query_facade_.get(), &QueryFacade::handleGetCartPosture));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetBaseFrame>(
      node_, "/xmate3/cobot/get_base_frame", query_facade_.get(), &QueryFacade::handleGetBaseFrame));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::CalcFk>(
      node_, "/xmate3/cobot/calc_fk", query_facade_.get(), &QueryFacade::handleCalcFk));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::CalcIk>(
      node_, "/xmate3/cobot/calc_ik", query_facade_.get(), &QueryFacade::handleCalcIk));

  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetToolset>(
      node_, "/xmate3/cobot/get_toolset", query_facade_.get(), &QueryFacade::handleGetToolset));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetToolset>(
      node_, "/xmate3/cobot/set_toolset", query_facade_.get(), &QueryFacade::handleSetToolset));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetToolsetByName>(
      node_, "/xmate3/cobot/set_toolset_by_name", query_facade_.get(), &QueryFacade::handleSetToolsetByName));

  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::EnableCollisionDetection>(
      node_, "/xmate3/cobot/enable_collision_detection", control_facade_.get(), &ControlFacade::handleEnableCollisionDetection));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::DisableCollisionDetection>(
      node_, "/xmate3/cobot/disable_collision_detection", control_facade_.get(), &ControlFacade::handleDisableCollisionDetection));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetSoftLimit>(
      node_, "/xmate3/cobot/get_soft_limit", query_facade_.get(), &QueryFacade::handleGetSoftLimit));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetSoftLimit>(
      node_, "/xmate3/cobot/set_soft_limit", control_facade_.get(), &ControlFacade::handleSetSoftLimit));

  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetMotionControlMode>(
      node_, "/xmate3/cobot/set_motion_control_mode", control_facade_.get(), &ControlFacade::handleSetMotionControlMode));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::MoveReset>(
      node_, "/xmate3/cobot/move_reset", control_facade_.get(), &ControlFacade::handleMoveReset));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::MoveStart>(
      node_, "/xmate3/cobot/move_start", control_facade_.get(), &ControlFacade::handleMoveStart));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::Stop>(
      node_, "/xmate3/cobot/stop", control_facade_.get(), &ControlFacade::handleStop));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetDefaultSpeed>(
      node_, "/xmate3/cobot/set_default_speed", control_facade_.get(), &ControlFacade::handleSetDefaultSpeed));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetDefaultZone>(
      node_, "/xmate3/cobot/set_default_zone", control_facade_.get(), &ControlFacade::handleSetDefaultZone));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetDefaultConfOpt>(
      node_, "/xmate3/cobot/set_default_conf_opt", control_facade_.get(), &ControlFacade::handleSetDefaultConfOpt));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::AdjustSpeedOnline>(
      node_, "/xmate3/cobot/adjust_speed_online", control_facade_.get(), &ControlFacade::handleAdjustSpeedOnline));

  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetRtControlMode>(
      node_, "/xmate3/cobot/set_rt_control_mode", control_facade_.get(), &ControlFacade::handleSetRtControlMode));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetRtJointData>(
      node_, "/xmate3/cobot/get_rt_joint_data", query_facade_.get(), &QueryFacade::handleGetRtJointData));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SendCustomData>(
      node_, "/xmate3/cobot/send_custom_data", io_program_facade_.get(), &IoProgramFacade::handleSendCustomData));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::RegisterDataCallback>(
      node_, "/xmate3/cobot/register_data_callback", io_program_facade_.get(), &IoProgramFacade::handleRegisterDataCallback));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::ReadRegister>(
      node_, "/xmate3/cobot/read_register", io_program_facade_.get(), &IoProgramFacade::handleReadRegister));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::ReadRegisterEx>(
      node_, "/xmate3/cobot/read_register_ex", io_program_facade_.get(), &IoProgramFacade::handleReadRegisterEx));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::WriteRegister>(
      node_, "/xmate3/cobot/write_register", io_program_facade_.get(), &IoProgramFacade::handleWriteRegister));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::WriteRegisterEx>(
      node_, "/xmate3/cobot/write_register_ex", io_program_facade_.get(), &IoProgramFacade::handleWriteRegisterEx));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetXPanelVout>(
      node_, "/xmate3/cobot/set_xpanel_vout", io_program_facade_.get(), &IoProgramFacade::handleSetXPanelVout));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::LoadRLProject>(
      node_, "/xmate3/cobot/load_rl_project", io_program_facade_.get(), &IoProgramFacade::handleLoadRlProject));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::StartRLProject>(
      node_, "/xmate3/cobot/start_rl_project", io_program_facade_.get(), &IoProgramFacade::handleStartRlProject));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::StopRLProject>(
      node_, "/xmate3/cobot/stop_rl_project", io_program_facade_.get(), &IoProgramFacade::handleStopRlProject));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::PauseRLProject>(
      node_, "/xmate3/cobot/pause_rl_project", io_program_facade_.get(), &IoProgramFacade::handlePauseRlProject));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetProjectRunningOpt>(
      node_, "/xmate3/cobot/set_project_running_opt", io_program_facade_.get(), &IoProgramFacade::handleSetProjectRunningOpt));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetRlProjectInfo>(
      node_, "/xmate3/cobot/get_rl_project_info", query_facade_.get(), &QueryFacade::handleGetRlProjectInfo));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetToolCatalog>(
      node_, "/xmate3/cobot/get_tools_info", query_facade_.get(), &QueryFacade::handleGetToolCatalog));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetWobjCatalog>(
      node_, "/xmate3/cobot/get_wobjs_info", query_facade_.get(), &QueryFacade::handleGetWobjCatalog));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetAvoidSingularity>(
      node_, "/xmate3/cobot/set_avoid_singularity", control_facade_.get(), &ControlFacade::handleSetAvoidSingularity));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetAvoidSingularity>(
      node_, "/xmate3/cobot/get_avoid_singularity", query_facade_.get(), &QueryFacade::handleGetAvoidSingularity));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::CalcJointTorque>(
      node_, "/xmate3/cobot/calc_joint_torque", query_facade_.get(), &QueryFacade::handleCalcJointTorque));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GenerateSTrajectory>(
      node_, "/xmate3/cobot/generate_s_trajectory", query_facade_.get(), &QueryFacade::handleGenerateSTrajectory));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::MapCartesianToJointTorque>(
      node_, "/xmate3/cobot/map_cartesian_to_joint_torque", query_facade_.get(), &QueryFacade::handleMapCartesianToJointTorque));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetEndEffectorTorque>(
      node_, "/xmate3/cobot/get_end_torque", query_facade_.get(), &QueryFacade::handleGetEndEffectorTorque));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetEndWrench>(
      node_, "/xmate3/cobot/get_end_wrench", query_facade_.get(), &QueryFacade::handleGetEndWrench));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::ValidateMotion>(
      node_, "/xmate3/internal/validate_motion", query_facade_.get(), &QueryFacade::handleValidateMotion));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetRuntimeDiagnostics>(
      node_, "/xmate3/internal/get_runtime_diagnostics", query_facade_.get(), &QueryFacade::handleGetRuntimeDiagnostics));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetProfileCapabilities>(
      node_, "/xmate3/internal/get_profile_capabilities", query_facade_.get(), &QueryFacade::handleGetProfileCapabilities));

  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetDI>(
      node_, "/xmate3/io/get_di", io_program_facade_.get(), &IoProgramFacade::handleGetDI));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetDO>(
      node_, "/xmate3/io/get_do", io_program_facade_.get(), &IoProgramFacade::handleGetDO));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetDI>(
      node_, "/xmate3/io/set_di", io_program_facade_.get(), &IoProgramFacade::handleSetDI));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetDO>(
      node_, "/xmate3/io/set_do", io_program_facade_.get(), &IoProgramFacade::handleSetDO));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::GetAI>(
      node_, "/xmate3/io/get_ai", io_program_facade_.get(), &IoProgramFacade::handleGetAI));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetAO>(
      node_, "/xmate3/io/set_ao", io_program_facade_.get(), &IoProgramFacade::handleSetAO));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SetSimulationMode>(
      node_, "/xmate3/io/set_simulation_mode", control_facade_.get(), &ControlFacade::handleSetSimulationMode));

  registerCompatibilityAliases();

  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::EnableDrag>(
      node_, "/xmate3/cobot/enable_drag", control_facade_.get(), &ControlFacade::handleEnableDrag));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::DisableDrag>(
      node_, "/xmate3/cobot/disable_drag", control_facade_.get(), &ControlFacade::handleDisableDrag));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::StartRecordPath>(
      node_, "/xmate3/cobot/start_record_path", path_facade_.get(), &PathFacade::handleStartRecordPath));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::StopRecordPath>(
      node_, "/xmate3/cobot/stop_record_path", path_facade_.get(), &PathFacade::handleStopRecordPath));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::CancelRecordPath>(
      node_, "/xmate3/cobot/cancel_record_path", path_facade_.get(), &PathFacade::handleCancelRecordPath));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::SaveRecordPath>(
      node_, "/xmate3/cobot/save_record_path", path_facade_.get(), &PathFacade::handleSaveRecordPath));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::ReplayPath>(
      node_, "/xmate3/cobot/replay_path", path_facade_.get(), &PathFacade::handleReplayPath));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::RemovePath>(
      node_, "/xmate3/cobot/remove_path", path_facade_.get(), &PathFacade::handleRemovePath));
  services_.push_back(detail::CreateFacadeService<rokae_xmate3_ros2::srv::QueryPathLists>(
      node_, "/xmate3/cobot/query_path_lists", path_facade_.get(), &PathFacade::handleQueryPathLists));
}

}  // namespace rokae_xmate3_ros2::runtime
