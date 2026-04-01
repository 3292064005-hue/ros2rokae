#include "runtime/ros_bindings.hpp"

#include <stdexcept>
#include <vector>

#include "runtime/service_registration.hpp"

namespace rokae_xmate3_ros2::runtime {

namespace {
std::vector<ServiceRegistrationDescriptor> makePrimaryServiceDescriptors() {
  std::vector<ServiceRegistrationDescriptor> descriptors;
#define ROKAE_ADD_PRIMARY(ServiceT, domain, name, facade_accessor, method)   descriptors.push_back(makeServiceRegistrationDescriptor<ServiceT>(domain, name, false, facade_accessor, method))

  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::Connect, "control", "/xmate3/cobot/connect", &RosBindings::controlFacade, &ControlFacade::handleConnect);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::Disconnect, "control", "/xmate3/cobot/disconnect", &RosBindings::controlFacade, &ControlFacade::handleDisconnect);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetPowerState, "control", "/xmate3/cobot/set_power_state", &RosBindings::controlFacade, &ControlFacade::handleSetPowerState);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetPowerState, "query", "/xmate3/cobot/get_power_state", &RosBindings::queryFacade, &QueryFacade::handleGetPowerState);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetInfo, "query", "/xmate3/cobot/get_info", &RosBindings::queryFacade, &QueryFacade::handleGetInfo);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetRuntimeDiagnostics, "query", "/xmate3/internal/get_runtime_diagnostics", &RosBindings::queryFacade, &QueryFacade::handleGetRuntimeDiagnostics);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetProfileCapabilities, "query", "/xmate3/internal/get_profile_capabilities", &RosBindings::queryFacade, &QueryFacade::handleGetProfileCapabilities);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetOperateMode, "query", "/xmate3/cobot/get_operate_mode", &RosBindings::queryFacade, &QueryFacade::handleGetOperateMode);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetOperateMode, "control", "/xmate3/cobot/set_operate_mode", &RosBindings::controlFacade, &ControlFacade::handleSetOperateMode);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::QueryControllerLog, "query", "/xmate3/cobot/query_controller_log", &RosBindings::queryFacade, &QueryFacade::handleQueryControllerLog);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::ClearServoAlarm, "control", "/xmate3/cobot/clear_servo_alarm", &RosBindings::controlFacade, &ControlFacade::handleClearServoAlarm);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetJointPos, "query", "/xmate3/cobot/get_joint_pos", &RosBindings::queryFacade, &QueryFacade::handleGetJointPos);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetJointVel, "query", "/xmate3/cobot/get_joint_vel", &RosBindings::queryFacade, &QueryFacade::handleGetJointVel);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetJointTorques, "query", "/xmate3/cobot/get_joint_torque", &RosBindings::queryFacade, &QueryFacade::handleGetJointTorques);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetPosture, "query", "/xmate3/cobot/get_posture", &RosBindings::queryFacade, &QueryFacade::handleGetPosture);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetCartPosture, "query", "/xmate3/cobot/get_cart_posture", &RosBindings::queryFacade, &QueryFacade::handleGetCartPosture);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetBaseFrame, "query", "/xmate3/cobot/get_base_frame", &RosBindings::queryFacade, &QueryFacade::handleGetBaseFrame);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::CalcFk, "query", "/xmate3/cobot/calc_fk", &RosBindings::queryFacade, &QueryFacade::handleCalcFk);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::CalcIk, "query", "/xmate3/cobot/calc_ik", &RosBindings::queryFacade, &QueryFacade::handleCalcIk);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetToolset, "query", "/xmate3/cobot/get_toolset", &RosBindings::queryFacade, &QueryFacade::handleGetToolset);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetToolset, "query", "/xmate3/cobot/set_toolset", &RosBindings::queryFacade, &QueryFacade::handleSetToolset);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetToolsetByName, "query", "/xmate3/cobot/set_toolset_by_name", &RosBindings::queryFacade, &QueryFacade::handleSetToolsetByName);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::EnableCollisionDetection, "control", "/xmate3/cobot/enable_collision_detection", &RosBindings::controlFacade, &ControlFacade::handleEnableCollisionDetection);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::DisableCollisionDetection, "control", "/xmate3/cobot/disable_collision_detection", &RosBindings::controlFacade, &ControlFacade::handleDisableCollisionDetection);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetSoftLimit, "query", "/xmate3/cobot/get_soft_limit", &RosBindings::queryFacade, &QueryFacade::handleGetSoftLimit);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetSoftLimit, "control", "/xmate3/cobot/set_soft_limit", &RosBindings::controlFacade, &ControlFacade::handleSetSoftLimit);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetMotionControlMode, "control", "/xmate3/cobot/set_motion_control_mode", &RosBindings::controlFacade, &ControlFacade::handleSetMotionControlMode);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::MoveReset, "control", "/xmate3/cobot/move_reset", &RosBindings::controlFacade, &ControlFacade::handleMoveReset);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::MoveStart, "control", "/xmate3/cobot/move_start", &RosBindings::controlFacade, &ControlFacade::handleMoveStart);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::Stop, "control", "/xmate3/cobot/stop", &RosBindings::controlFacade, &ControlFacade::handleStop);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetDefaultSpeed, "control", "/xmate3/cobot/set_default_speed", &RosBindings::controlFacade, &ControlFacade::handleSetDefaultSpeed);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetDefaultZone, "control", "/xmate3/cobot/set_default_zone", &RosBindings::controlFacade, &ControlFacade::handleSetDefaultZone);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetDefaultConfOpt, "control", "/xmate3/cobot/set_default_conf_opt", &RosBindings::controlFacade, &ControlFacade::handleSetDefaultConfOpt);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::AdjustSpeedOnline, "control", "/xmate3/cobot/adjust_speed_online", &RosBindings::controlFacade, &ControlFacade::handleAdjustSpeedOnline);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetRtControlMode, "control", "/xmate3/cobot/set_rt_control_mode", &RosBindings::controlFacade, &ControlFacade::handleSetRtControlMode);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetRtJointData, "query", "/xmate3/cobot/get_rt_joint_data", &RosBindings::queryFacade, &QueryFacade::handleGetRtJointData);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::ValidateMotion, "query", "/xmate3/internal/validate_motion", &RosBindings::queryFacade, &QueryFacade::handleValidateMotion);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SendCustomData, "io_program", "/xmate3/cobot/send_custom_data", &RosBindings::ioProgramFacade, &IoProgramFacade::handleSendCustomData);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::RegisterDataCallback, "io_program", "/xmate3/cobot/register_data_callback", &RosBindings::ioProgramFacade, &IoProgramFacade::handleRegisterDataCallback);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::ReadRegister, "io_program", "/xmate3/cobot/read_register", &RosBindings::ioProgramFacade, &IoProgramFacade::handleReadRegister);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::ReadRegisterEx, "io_program", "/xmate3/cobot/read_register_ex", &RosBindings::ioProgramFacade, &IoProgramFacade::handleReadRegisterEx);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::WriteRegister, "io_program", "/xmate3/cobot/write_register", &RosBindings::ioProgramFacade, &IoProgramFacade::handleWriteRegister);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::WriteRegisterEx, "io_program", "/xmate3/cobot/write_register_ex", &RosBindings::ioProgramFacade, &IoProgramFacade::handleWriteRegisterEx);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetXPanelVout, "io_program", "/xmate3/cobot/set_xpanel_vout", &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetXPanelVout);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetDI, "io_program", "/xmate3/io/get_di", &RosBindings::ioProgramFacade, &IoProgramFacade::handleGetDI);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetDO, "io_program", "/xmate3/io/get_do", &RosBindings::ioProgramFacade, &IoProgramFacade::handleGetDO);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetDI, "io_program", "/xmate3/io/set_di", &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetDI);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetDO, "io_program", "/xmate3/io/set_do", &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetDO);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetAI, "io_program", "/xmate3/io/get_ai", &RosBindings::ioProgramFacade, &IoProgramFacade::handleGetAI);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetAO, "io_program", "/xmate3/io/set_ao", &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetAO);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetSimulationMode, "control", "/xmate3/io/set_simulation_mode", &RosBindings::controlFacade, &ControlFacade::handleSetSimulationMode);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::LoadRLProject, "io_program", "/xmate3/cobot/load_rl_project", &RosBindings::ioProgramFacade, &IoProgramFacade::handleLoadRlProject);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::StartRLProject, "io_program", "/xmate3/cobot/start_rl_project", &RosBindings::ioProgramFacade, &IoProgramFacade::handleStartRlProject);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::StopRLProject, "io_program", "/xmate3/cobot/stop_rl_project", &RosBindings::ioProgramFacade, &IoProgramFacade::handleStopRlProject);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::PauseRLProject, "io_program", "/xmate3/cobot/pause_rl_project", &RosBindings::ioProgramFacade, &IoProgramFacade::handlePauseRlProject);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetProjectRunningOpt, "io_program", "/xmate3/cobot/set_project_running_opt", &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetProjectRunningOpt);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetRlProjectInfo, "query", "/xmate3/cobot/get_rl_project_info", &RosBindings::queryFacade, &QueryFacade::handleGetRlProjectInfo);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetToolCatalog, "query", "/xmate3/cobot/get_tools_info", &RosBindings::queryFacade, &QueryFacade::handleGetToolCatalog);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetWobjCatalog, "query", "/xmate3/cobot/get_wobjs_info", &RosBindings::queryFacade, &QueryFacade::handleGetWobjCatalog);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SetAvoidSingularity, "control", "/xmate3/cobot/set_avoid_singularity", &RosBindings::controlFacade, &ControlFacade::handleSetAvoidSingularity);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetAvoidSingularity, "query", "/xmate3/cobot/get_avoid_singularity", &RosBindings::queryFacade, &QueryFacade::handleGetAvoidSingularity);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetEndEffectorTorque, "query", "/xmate3/cobot/get_end_torque", &RosBindings::queryFacade, &QueryFacade::handleGetEndEffectorTorque);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GetEndWrench, "query", "/xmate3/cobot/get_end_wrench", &RosBindings::queryFacade, &QueryFacade::handleGetEndWrench);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::CalcJointTorque, "query", "/xmate3/cobot/calc_joint_torque", &RosBindings::queryFacade, &QueryFacade::handleCalcJointTorque);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::GenerateSTrajectory, "query", "/xmate3/cobot/generate_s_trajectory", &RosBindings::queryFacade, &QueryFacade::handleGenerateSTrajectory);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::MapCartesianToJointTorque, "query", "/xmate3/cobot/map_cartesian_to_joint_torque", &RosBindings::queryFacade, &QueryFacade::handleMapCartesianToJointTorque);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::EnableDrag, "control", "/xmate3/cobot/enable_drag", &RosBindings::controlFacade, &ControlFacade::handleEnableDrag);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::DisableDrag, "control", "/xmate3/cobot/disable_drag", &RosBindings::controlFacade, &ControlFacade::handleDisableDrag);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::StartRecordPath, "path", "/xmate3/cobot/start_record_path", &RosBindings::pathFacade, &PathFacade::handleStartRecordPath);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::StopRecordPath, "path", "/xmate3/cobot/stop_record_path", &RosBindings::pathFacade, &PathFacade::handleStopRecordPath);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::CancelRecordPath, "path", "/xmate3/cobot/cancel_record_path", &RosBindings::pathFacade, &PathFacade::handleCancelRecordPath);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::SaveRecordPath, "path", "/xmate3/cobot/save_record_path", &RosBindings::pathFacade, &PathFacade::handleSaveRecordPath);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::ReplayPath, "path", "/xmate3/cobot/replay_path", &RosBindings::pathFacade, &PathFacade::handleReplayPath);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::RemovePath, "path", "/xmate3/cobot/remove_path", &RosBindings::pathFacade, &PathFacade::handleRemovePath);
  ROKAE_ADD_PRIMARY(rokae_xmate3_ros2::srv::QueryPathLists, "path", "/xmate3/cobot/query_path_lists", &RosBindings::pathFacade, &PathFacade::handleQueryPathLists);
#undef ROKAE_ADD_PRIMARY
  return descriptors;
}
}  // namespace

void RosBindings::initServices() {
  const auto descriptors = makePrimaryServiceDescriptors();
  std::string error_message;
  if (!validateServiceDescriptors(descriptors, error_message)) {
    throw std::runtime_error(error_message);
  }
  services_.reserve(descriptors.size());
  for (const auto &descriptor : descriptors) {
    services_.push_back(descriptor.create(*this));
  }
}

}  // namespace rokae_xmate3_ros2::runtime
