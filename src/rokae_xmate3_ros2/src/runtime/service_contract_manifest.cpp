#include "runtime/service_contract_manifest.hpp"

#include <vector>

namespace rokae_xmate3_ros2::runtime {

namespace {

template <typename ServiceT, typename FacadeT>
void appendServiceDescriptor(std::vector<ServiceRegistrationDescriptor> &descriptors,
                             const char *domain,
                             const char *name,
                             bool compatibility_alias,
                             FacadeT *(RosBindings::*facade_accessor)() const,
                             void (FacadeT::*method)(const typename ServiceT::Request &, typename ServiceT::Response &) const) {
  descriptors.push_back(makeServiceRegistrationDescriptor<ServiceT>(domain, name, compatibility_alias, facade_accessor, method));
}

}  // namespace

std::vector<ServiceRegistrationDescriptor> buildPublicPrimaryServiceContractManifest() {
  std::vector<ServiceRegistrationDescriptor> descriptors;
  descriptors.reserve(41);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::Connect, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/connect", false, &RosBindings::controlFacade, &ControlFacade::handleConnect);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::Disconnect, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/disconnect", false, &RosBindings::controlFacade, &ControlFacade::handleDisconnect);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetPowerState, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/set_power_state", false, &RosBindings::controlFacade, &ControlFacade::handleSetPowerState);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetPowerState, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_power_state", false, &RosBindings::queryFacade, &QueryFacade::handleGetPowerState);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetInfo, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_info", false, &RosBindings::queryFacade, &QueryFacade::handleGetInfo);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetRuntimeDiagnostics, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_runtime_diagnostics", false, &RosBindings::queryFacade, &QueryFacade::handleGetRuntimeDiagnostics);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetRuntimeStateSnapshot, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_runtime_state_snapshot", false, &RosBindings::queryFacade, &QueryFacade::handleGetRuntimeStateSnapshot);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetProfileCapabilities, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_profile_capabilities", false, &RosBindings::queryFacade, &QueryFacade::handleGetProfileCapabilities);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetOperateMode, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_operate_mode", false, &RosBindings::queryFacade, &QueryFacade::handleGetOperateMode);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetOperateMode, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/set_operate_mode", false, &RosBindings::controlFacade, &ControlFacade::handleSetOperateMode);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::QueryControllerLog, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/query_controller_log", false, &RosBindings::queryFacade, &QueryFacade::handleQueryControllerLog);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::ClearServoAlarm, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/clear_servo_alarm", false, &RosBindings::controlFacade, &ControlFacade::handleClearServoAlarm);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetJointPos, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_joint_pos", false, &RosBindings::queryFacade, &QueryFacade::handleGetJointPos);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetJointVel, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_joint_vel", false, &RosBindings::queryFacade, &QueryFacade::handleGetJointVel);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetJointTorques, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_joint_torque", false, &RosBindings::queryFacade, &QueryFacade::handleGetJointTorques);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetPosture, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_posture", false, &RosBindings::queryFacade, &QueryFacade::handleGetPosture);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetCartPosture, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_cart_posture", false, &RosBindings::queryFacade, &QueryFacade::handleGetCartPosture);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetBaseFrame, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_base_frame", false, &RosBindings::queryFacade, &QueryFacade::handleGetBaseFrame);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::CalcFk, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/calc_fk", false, &RosBindings::queryFacade, &QueryFacade::handleCalcFk);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::CalcIk, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/calc_ik", false, &RosBindings::queryFacade, &QueryFacade::handleCalcIk);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetToolset, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_toolset", false, &RosBindings::queryFacade, &QueryFacade::handleGetToolset);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetToolset, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/set_toolset", false, &RosBindings::controlFacade, &ControlFacade::handleSetToolset);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetToolsetByName, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/set_toolset_by_name", false, &RosBindings::controlFacade, &ControlFacade::handleSetToolsetByName);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::EnableCollisionDetection, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/enable_collision_detection", false, &RosBindings::controlFacade, &ControlFacade::handleEnableCollisionDetection);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::DisableCollisionDetection, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/disable_collision_detection", false, &RosBindings::controlFacade, &ControlFacade::handleDisableCollisionDetection);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetSoftLimit, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_soft_limit", false, &RosBindings::queryFacade, &QueryFacade::handleGetSoftLimit);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetSoftLimit, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/set_soft_limit", false, &RosBindings::controlFacade, &ControlFacade::handleSetSoftLimit);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetMotionControlMode, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/set_motion_control_mode", false, &RosBindings::controlFacade, &ControlFacade::handleSetMotionControlMode);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::MoveReset, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/move_reset", false, &RosBindings::controlFacade, &ControlFacade::handleMoveReset);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::MoveStart, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/move_start", false, &RosBindings::controlFacade, &ControlFacade::handleMoveStart);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::Stop, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/stop", false, &RosBindings::controlFacade, &ControlFacade::handleStop);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetDefaultSpeed, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/set_default_speed", false, &RosBindings::controlFacade, &ControlFacade::handleSetDefaultSpeed);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetDefaultZone, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/set_default_zone", false, &RosBindings::controlFacade, &ControlFacade::handleSetDefaultZone);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetDefaultConfOpt, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/set_default_conf_opt", false, &RosBindings::controlFacade, &ControlFacade::handleSetDefaultConfOpt);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::AdjustSpeedOnline, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/adjust_speed_online", false, &RosBindings::controlFacade, &ControlFacade::handleAdjustSpeedOnline);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetEndEffectorTorque, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_end_torque", false, &RosBindings::queryFacade, &QueryFacade::handleGetEndEffectorTorque);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetEndWrench, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_end_wrench", false, &RosBindings::queryFacade, &QueryFacade::handleGetEndWrench);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::CalcJointTorque, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/calc_joint_torque", false, &RosBindings::queryFacade, &QueryFacade::handleCalcJointTorque);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GenerateSTrajectory, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/generate_s_trajectory", false, &RosBindings::queryFacade, &QueryFacade::handleGenerateSTrajectory);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::MapCartesianToJointTorque, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/map_cartesian_to_joint_torque", false, &RosBindings::queryFacade, &QueryFacade::handleMapCartesianToJointTorque);
  return descriptors;
}

std::vector<ServiceRegistrationDescriptor> buildPublicExperimentalPrimaryServiceContractManifest() {
  std::vector<ServiceRegistrationDescriptor> descriptors;
  descriptors.reserve(11);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetRtControlMode, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/set_rt_control_mode", false, &RosBindings::controlFacade, &ControlFacade::handleSetRtControlMode);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetRtJointData, QueryFacade>(descriptors, "query", "/xmate_er3/cobot/get_rt_joint_data", false, &RosBindings::queryFacade, &QueryFacade::handleGetRtJointData);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::EnableDrag, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/enable_drag", false, &RosBindings::controlFacade, &ControlFacade::handleEnableDrag);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::DisableDrag, ControlFacade>(descriptors, "control", "/xmate_er3/cobot/disable_drag", false, &RosBindings::controlFacade, &ControlFacade::handleDisableDrag);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::StartRecordPath, PathFacade>(descriptors, "path", "/xmate_er3/cobot/start_record_path", false, &RosBindings::pathFacade, &PathFacade::handleStartRecordPath);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::StopRecordPath, PathFacade>(descriptors, "path", "/xmate_er3/cobot/stop_record_path", false, &RosBindings::pathFacade, &PathFacade::handleStopRecordPath);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::CancelRecordPath, PathFacade>(descriptors, "path", "/xmate_er3/cobot/cancel_record_path", false, &RosBindings::pathFacade, &PathFacade::handleCancelRecordPath);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SaveRecordPath, PathFacade>(descriptors, "path", "/xmate_er3/cobot/save_record_path", false, &RosBindings::pathFacade, &PathFacade::handleSaveRecordPath);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::ReplayPath, PathFacade>(descriptors, "path", "/xmate_er3/cobot/replay_path", false, &RosBindings::pathFacade, &PathFacade::handleReplayPath);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::RemovePath, PathFacade>(descriptors, "path", "/xmate_er3/cobot/remove_path", false, &RosBindings::pathFacade, &PathFacade::handleRemovePath);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::QueryPathLists, PathFacade>(descriptors, "path", "/xmate_er3/cobot/query_path_lists", false, &RosBindings::pathFacade, &PathFacade::handleQueryPathLists);
  return descriptors;
}

std::vector<ServiceRegistrationDescriptor> buildPublicCompatibilityAliasContractManifest() {
  std::vector<ServiceRegistrationDescriptor> descriptors;
  descriptors.reserve(37);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::Connect, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/connect", true, &RosBindings::controlFacade, &ControlFacade::handleConnect);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::Disconnect, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/disconnect", true, &RosBindings::controlFacade, &ControlFacade::handleDisconnect);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetPowerState, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/set_power_state", true, &RosBindings::controlFacade, &ControlFacade::handleSetPowerState);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetPowerState, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/get_power_state", true, &RosBindings::queryFacade, &QueryFacade::handleGetPowerState);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetInfo, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/get_info", true, &RosBindings::queryFacade, &QueryFacade::handleGetInfo);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetOperateMode, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/get_operate_mode", true, &RosBindings::queryFacade, &QueryFacade::handleGetOperateMode);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetOperateMode, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/set_operate_mode", true, &RosBindings::controlFacade, &ControlFacade::handleSetOperateMode);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::QueryControllerLog, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/query_controller_log", true, &RosBindings::queryFacade, &QueryFacade::handleQueryControllerLog);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::ClearServoAlarm, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/clear_servo_alarm", true, &RosBindings::controlFacade, &ControlFacade::handleClearServoAlarm);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetJointPos, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/get_joint_pos", true, &RosBindings::queryFacade, &QueryFacade::handleGetJointPos);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetJointVel, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/get_joint_vel", true, &RosBindings::queryFacade, &QueryFacade::handleGetJointVel);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetJointTorques, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/get_joint_torque", true, &RosBindings::queryFacade, &QueryFacade::handleGetJointTorques);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetPosture, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/get_posture", true, &RosBindings::queryFacade, &QueryFacade::handleGetPosture);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetCartPosture, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/get_cart_posture", true, &RosBindings::queryFacade, &QueryFacade::handleGetCartPosture);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetBaseFrame, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/get_base_frame", true, &RosBindings::queryFacade, &QueryFacade::handleGetBaseFrame);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::CalcFk, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/calc_fk", true, &RosBindings::queryFacade, &QueryFacade::handleCalcFk);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::CalcIk, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/calc_ik", true, &RosBindings::queryFacade, &QueryFacade::handleCalcIk);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetToolset, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/get_toolset", true, &RosBindings::queryFacade, &QueryFacade::handleGetToolset);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetToolset, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/set_toolset", true, &RosBindings::controlFacade, &ControlFacade::handleSetToolset);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetToolsetByName, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/set_toolset_by_name", true, &RosBindings::controlFacade, &ControlFacade::handleSetToolsetByName);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::EnableCollisionDetection, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/enable_collision_detection", true, &RosBindings::controlFacade, &ControlFacade::handleEnableCollisionDetection);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::DisableCollisionDetection, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/disable_collision_detection", true, &RosBindings::controlFacade, &ControlFacade::handleDisableCollisionDetection);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetSoftLimit, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/get_soft_limit", true, &RosBindings::queryFacade, &QueryFacade::handleGetSoftLimit);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetSoftLimit, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/set_soft_limit", true, &RosBindings::controlFacade, &ControlFacade::handleSetSoftLimit);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetMotionControlMode, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/set_motion_control_mode", true, &RosBindings::controlFacade, &ControlFacade::handleSetMotionControlMode);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::MoveReset, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/move_reset", true, &RosBindings::controlFacade, &ControlFacade::handleMoveReset);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::MoveStart, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/move_start", true, &RosBindings::controlFacade, &ControlFacade::handleMoveStart);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::Stop, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/stop", true, &RosBindings::controlFacade, &ControlFacade::handleStop);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetDefaultSpeed, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/set_default_speed", true, &RosBindings::controlFacade, &ControlFacade::handleSetDefaultSpeed);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetDefaultZone, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/set_default_zone", true, &RosBindings::controlFacade, &ControlFacade::handleSetDefaultZone);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetDefaultConfOpt, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/set_default_conf_opt", true, &RosBindings::controlFacade, &ControlFacade::handleSetDefaultConfOpt);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::AdjustSpeedOnline, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/adjust_speed_online", true, &RosBindings::controlFacade, &ControlFacade::handleAdjustSpeedOnline);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetEndEffectorTorque, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/get_end_torque", true, &RosBindings::queryFacade, &QueryFacade::handleGetEndEffectorTorque);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetEndWrench, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/get_end_wrench", true, &RosBindings::queryFacade, &QueryFacade::handleGetEndWrench);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::CalcJointTorque, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/calc_joint_torque", true, &RosBindings::queryFacade, &QueryFacade::handleCalcJointTorque);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GenerateSTrajectory, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/generate_s_trajectory", true, &RosBindings::queryFacade, &QueryFacade::handleGenerateSTrajectory);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::MapCartesianToJointTorque, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/map_cartesian_to_joint_torque", true, &RosBindings::queryFacade, &QueryFacade::handleMapCartesianToJointTorque);
  return descriptors;
}

std::vector<ServiceRegistrationDescriptor> buildPublicExperimentalCompatibilityAliasContractManifest() {
  std::vector<ServiceRegistrationDescriptor> descriptors;
  descriptors.reserve(11);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetRtControlMode, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/set_rt_control_mode", true, &RosBindings::controlFacade, &ControlFacade::handleSetRtControlMode);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetRtJointData, QueryFacade>(descriptors, "compatibility", "/xmate3/cobot/get_rt_joint_data", true, &RosBindings::queryFacade, &QueryFacade::handleGetRtJointData);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::EnableDrag, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/enable_drag", true, &RosBindings::controlFacade, &ControlFacade::handleEnableDrag);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::DisableDrag, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/disable_drag", true, &RosBindings::controlFacade, &ControlFacade::handleDisableDrag);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::StartRecordPath, PathFacade>(descriptors, "compatibility", "/xmate3/cobot/start_record_path", true, &RosBindings::pathFacade, &PathFacade::handleStartRecordPath);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::StopRecordPath, PathFacade>(descriptors, "compatibility", "/xmate3/cobot/stop_record_path", true, &RosBindings::pathFacade, &PathFacade::handleStopRecordPath);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::CancelRecordPath, PathFacade>(descriptors, "compatibility", "/xmate3/cobot/cancel_record_path", true, &RosBindings::pathFacade, &PathFacade::handleCancelRecordPath);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SaveRecordPath, PathFacade>(descriptors, "compatibility", "/xmate3/cobot/save_record_path", true, &RosBindings::pathFacade, &PathFacade::handleSaveRecordPath);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::ReplayPath, PathFacade>(descriptors, "compatibility", "/xmate3/cobot/replay_path", true, &RosBindings::pathFacade, &PathFacade::handleReplayPath);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::RemovePath, PathFacade>(descriptors, "compatibility", "/xmate3/cobot/remove_path", true, &RosBindings::pathFacade, &PathFacade::handleRemovePath);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::QueryPathLists, PathFacade>(descriptors, "compatibility", "/xmate3/cobot/query_path_lists", true, &RosBindings::pathFacade, &PathFacade::handleQueryPathLists);
  return descriptors;
}

#if ROKAE_ENABLE_INTERNAL_SURFACE
std::vector<ServiceRegistrationDescriptor> buildInternalPrimaryServiceContractManifest() {
  std::vector<ServiceRegistrationDescriptor> descriptors;
  descriptors.reserve(24);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::ValidateMotion, QueryFacade>(descriptors, "query", "/xmate_er3/internal/validate_motion", false, &RosBindings::queryFacade, &QueryFacade::handleValidateMotion);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::PlannerPreflightReport, QueryFacade>(descriptors, "query", "/xmate_er3/internal/planner_preflight_report", false, &RosBindings::queryFacade, &QueryFacade::handlePlannerPreflightReport);
#if ROKAE_ENABLE_INTERNAL_SURFACE && ROKAE_ENABLE_NON_TARGET_INTERNAL_MODULES
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SendCustomData, IoProgramFacade>(descriptors, "io_program", "/xmate3/cobot/send_custom_data", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleSendCustomData);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::RegisterDataCallback, IoProgramFacade>(descriptors, "io_program", "/xmate3/cobot/register_data_callback", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleRegisterDataCallback);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::ReadRegister, IoProgramFacade>(descriptors, "io_program", "/xmate3/cobot/read_register", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleReadRegister);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::ReadRegisterEx, IoProgramFacade>(descriptors, "io_program", "/xmate3/cobot/read_register_ex", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleReadRegisterEx);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::WriteRegister, IoProgramFacade>(descriptors, "io_program", "/xmate3/cobot/write_register", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleWriteRegister);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::WriteRegisterEx, IoProgramFacade>(descriptors, "io_program", "/xmate3/cobot/write_register_ex", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleWriteRegisterEx);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetXPanelVout, IoProgramFacade>(descriptors, "io_program", "/xmate3/cobot/set_xpanel_vout", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetXPanelVout);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetDI, IoProgramFacade>(descriptors, "io_program", "/xmate3/io/get_di", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleGetDI);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetDO, IoProgramFacade>(descriptors, "io_program", "/xmate3/io/get_do", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleGetDO);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetDI, IoProgramFacade>(descriptors, "io_program", "/xmate3/io/set_di", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetDI);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetDO, IoProgramFacade>(descriptors, "io_program", "/xmate3/io/set_do", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetDO);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetAI, IoProgramFacade>(descriptors, "io_program", "/xmate3/io/get_ai", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleGetAI);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetAO, IoProgramFacade>(descriptors, "io_program", "/xmate3/io/set_ao", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetAO);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetSimulationMode, ControlFacade>(descriptors, "control", "/xmate3/io/set_simulation_mode", false, &RosBindings::controlFacade, &ControlFacade::handleSetSimulationMode);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::LoadRLProject, IoProgramFacade>(descriptors, "io_program", "/xmate3/cobot/load_rl_project", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleLoadRlProject);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::StartRLProject, IoProgramFacade>(descriptors, "io_program", "/xmate3/cobot/start_rl_project", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleStartRlProject);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::StopRLProject, IoProgramFacade>(descriptors, "io_program", "/xmate3/cobot/stop_rl_project", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleStopRlProject);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::PauseRLProject, IoProgramFacade>(descriptors, "io_program", "/xmate3/cobot/pause_rl_project", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handlePauseRlProject);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetProjectRunningOpt, IoProgramFacade>(descriptors, "io_program", "/xmate3/cobot/set_project_running_opt", false, &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetProjectRunningOpt);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetRlProjectInfo, QueryFacade>(descriptors, "query", "/xmate3/cobot/get_rl_project_info", false, &RosBindings::queryFacade, &QueryFacade::handleGetRlProjectInfo);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetToolCatalog, QueryFacade>(descriptors, "query", "/xmate3/cobot/get_tools_info", false, &RosBindings::queryFacade, &QueryFacade::handleGetToolCatalog);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetWobjCatalog, QueryFacade>(descriptors, "query", "/xmate3/cobot/get_wobjs_info", false, &RosBindings::queryFacade, &QueryFacade::handleGetWobjCatalog);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetAvoidSingularity, ControlFacade>(descriptors, "control", "/xmate3/cobot/set_avoid_singularity", false, &RosBindings::controlFacade, &ControlFacade::handleSetAvoidSingularity);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetAvoidSingularity, QueryFacade>(descriptors, "query", "/xmate3/cobot/get_avoid_singularity", false, &RosBindings::queryFacade, &QueryFacade::handleGetAvoidSingularity);
#endif
  return descriptors;
}
#endif

#if ROKAE_ENABLE_INTERNAL_SURFACE
std::vector<ServiceRegistrationDescriptor> buildInternalCompatibilityAliasContractManifest() {
  std::vector<ServiceRegistrationDescriptor> descriptors;
  descriptors.reserve(9);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::ValidateMotion, QueryFacade>(descriptors, "compatibility", "/xmate3/internal/validate_motion", true, &RosBindings::queryFacade, &QueryFacade::handleValidateMotion);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::PlannerPreflightReport, QueryFacade>(descriptors, "compatibility", "/xmate3/internal/planner_preflight_report", true, &RosBindings::queryFacade, &QueryFacade::handlePlannerPreflightReport);
#if ROKAE_ENABLE_INTERNAL_SURFACE && ROKAE_ENABLE_NON_TARGET_INTERNAL_MODULES
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetDI, IoProgramFacade>(descriptors, "compatibility", "/xmate3/cobot/get_di", true, &RosBindings::ioProgramFacade, &IoProgramFacade::handleGetDI);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetDO, IoProgramFacade>(descriptors, "compatibility", "/xmate3/cobot/get_do", true, &RosBindings::ioProgramFacade, &IoProgramFacade::handleGetDO);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetDI, IoProgramFacade>(descriptors, "compatibility", "/xmate3/cobot/set_di", true, &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetDI);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetDO, IoProgramFacade>(descriptors, "compatibility", "/xmate3/cobot/set_do", true, &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetDO);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::GetAI, IoProgramFacade>(descriptors, "compatibility", "/xmate3/cobot/get_ai", true, &RosBindings::ioProgramFacade, &IoProgramFacade::handleGetAI);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetAO, IoProgramFacade>(descriptors, "compatibility", "/xmate3/cobot/set_ao", true, &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetAO);
  appendServiceDescriptor<rokae_xmate3_ros2::srv::SetSimulationMode, ControlFacade>(descriptors, "compatibility", "/xmate3/cobot/set_simulation_mode", true, &RosBindings::controlFacade, &ControlFacade::handleSetSimulationMode);
#endif
  return descriptors;
}
#endif

}  // namespace rokae_xmate3_ros2::runtime
