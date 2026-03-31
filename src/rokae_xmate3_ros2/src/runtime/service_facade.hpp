#ifndef ROKAE_XMATE3_ROS2_RUNTIME_SERVICE_FACADE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_SERVICE_FACADE_HPP

#include <array>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/time.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "rokae_xmate3_ros2/gazebo/trajectory_planner.hpp"
#include "runtime/runtime_state.hpp"
#include "runtime/motion_runtime.hpp"
#include "runtime/request_coordinator.hpp"

#include "rokae_xmate3_ros2/srv/adjust_speed_online.hpp"
#include "rokae_xmate3_ros2/srv/calc_fk.hpp"
#include "rokae_xmate3_ros2/srv/calc_ik.hpp"
#include "rokae_xmate3_ros2/srv/calc_joint_torque.hpp"
#include "rokae_xmate3_ros2/srv/cancel_record_path.hpp"
#include "rokae_xmate3_ros2/srv/clear_servo_alarm.hpp"
#include "rokae_xmate3_ros2/srv/connect.hpp"
#include "rokae_xmate3_ros2/srv/disable_collision_detection.hpp"
#include "rokae_xmate3_ros2/srv/disable_drag.hpp"
#include "rokae_xmate3_ros2/srv/disconnect.hpp"
#include "rokae_xmate3_ros2/srv/enable_collision_detection.hpp"
#include "rokae_xmate3_ros2/srv/enable_drag.hpp"
#include "rokae_xmate3_ros2/srv/generate_s_trajectory.hpp"
#include "rokae_xmate3_ros2/srv/get_ai.hpp"
#include "rokae_xmate3_ros2/srv/get_avoid_singularity.hpp"
#include "rokae_xmate3_ros2/srv/get_base_frame.hpp"
#include "rokae_xmate3_ros2/srv/get_cart_posture.hpp"
#include "rokae_xmate3_ros2/srv/get_di.hpp"
#include "rokae_xmate3_ros2/srv/get_do.hpp"
#include "rokae_xmate3_ros2/srv/get_end_effector_torque.hpp"
#include "rokae_xmate3_ros2/srv/get_end_wrench.hpp"
#include "rokae_xmate3_ros2/srv/get_rl_project_info.hpp"
#include "rokae_xmate3_ros2/srv/get_tool_catalog.hpp"
#include "rokae_xmate3_ros2/srv/get_wobj_catalog.hpp"
#include "rokae_xmate3_ros2/srv/get_info.hpp"
#include "rokae_xmate3_ros2/srv/get_joint_pos.hpp"
#include "rokae_xmate3_ros2/srv/get_joint_torques.hpp"
#include "rokae_xmate3_ros2/srv/get_joint_vel.hpp"
#include "rokae_xmate3_ros2/srv/get_operate_mode.hpp"
#include "rokae_xmate3_ros2/srv/get_posture.hpp"
#include "rokae_xmate3_ros2/srv/get_power_state.hpp"
#include "rokae_xmate3_ros2/srv/get_runtime_diagnostics.hpp"
#include "rokae_xmate3_ros2/srv/get_profile_capabilities.hpp"
#include "rokae_xmate3_ros2/srv/get_rt_joint_data.hpp"
#include "rokae_xmate3_ros2/srv/get_soft_limit.hpp"
#include "rokae_xmate3_ros2/srv/get_toolset.hpp"
#include "rokae_xmate3_ros2/srv/load_rl_project.hpp"
#include "rokae_xmate3_ros2/srv/map_cartesian_to_joint_torque.hpp"
#include "rokae_xmate3_ros2/srv/move_reset.hpp"
#include "rokae_xmate3_ros2/srv/move_start.hpp"
#include "rokae_xmate3_ros2/srv/query_controller_log.hpp"
#include "rokae_xmate3_ros2/srv/query_path_lists.hpp"
#include "rokae_xmate3_ros2/srv/read_register.hpp"
#include "rokae_xmate3_ros2/srv/read_register_ex.hpp"
#include "rokae_xmate3_ros2/srv/register_data_callback.hpp"
#include "rokae_xmate3_ros2/srv/remove_path.hpp"
#include "rokae_xmate3_ros2/srv/replay_path.hpp"
#include "rokae_xmate3_ros2/srv/save_record_path.hpp"
#include "rokae_xmate3_ros2/srv/send_custom_data.hpp"
#include "rokae_xmate3_ros2/srv/set_ao.hpp"
#include "rokae_xmate3_ros2/srv/set_avoid_singularity.hpp"
#include "rokae_xmate3_ros2/srv/set_default_conf_opt.hpp"
#include "rokae_xmate3_ros2/srv/set_default_speed.hpp"
#include "rokae_xmate3_ros2/srv/set_default_zone.hpp"
#include "rokae_xmate3_ros2/srv/set_di.hpp"
#include "rokae_xmate3_ros2/srv/set_do.hpp"
#include "rokae_xmate3_ros2/srv/set_motion_control_mode.hpp"
#include "rokae_xmate3_ros2/srv/set_operate_mode.hpp"
#include "rokae_xmate3_ros2/srv/set_power_state.hpp"
#include "rokae_xmate3_ros2/srv/set_rt_control_mode.hpp"
#include "rokae_xmate3_ros2/srv/set_simulation_mode.hpp"
#include "rokae_xmate3_ros2/srv/set_soft_limit.hpp"
#include "rokae_xmate3_ros2/srv/set_toolset.hpp"
#include "rokae_xmate3_ros2/srv/set_toolset_by_name.hpp"
#include "rokae_xmate3_ros2/srv/start_record_path.hpp"
#include "rokae_xmate3_ros2/srv/pause_rl_project.hpp"
#include "rokae_xmate3_ros2/srv/set_project_running_opt.hpp"
#include "rokae_xmate3_ros2/srv/set_x_panel_vout.hpp"
#include "rokae_xmate3_ros2/srv/start_rl_project.hpp"
#include "rokae_xmate3_ros2/srv/stop.hpp"
#include "rokae_xmate3_ros2/srv/stop_record_path.hpp"
#include "rokae_xmate3_ros2/srv/stop_rl_project.hpp"
#include "rokae_xmate3_ros2/srv/validate_motion.hpp"
#include "rokae_xmate3_ros2/srv/write_register.hpp"
#include "rokae_xmate3_ros2/srv/write_register_ex.hpp"

namespace rokae_xmate3_ros2::runtime {

using JointStateFetcher = std::function<void(std::array<double, 6> &, std::array<double, 6> &, std::array<double, 6> &)>;
using TimeProvider = std::function<rclcpp::Time()>;
using RequestIdGenerator = std::function<std::string(const std::string &)>;
using TrajectoryDtProvider = std::function<double()>;

class ControlFacade {
 public:
  ControlFacade(SessionState &session_state,
                MotionOptionsState &motion_options_state,
                BackendInterface *backend,
                MotionRuntime *motion_runtime,
                MotionRequestCoordinator *request_coordinator);

  void handleConnect(const rokae_xmate3_ros2::srv::Connect::Request &req,
                     rokae_xmate3_ros2::srv::Connect::Response &res) const;
  void handleDisconnect(const rokae_xmate3_ros2::srv::Disconnect::Request &req,
                        rokae_xmate3_ros2::srv::Disconnect::Response &res) const;
  void handleSetPowerState(const rokae_xmate3_ros2::srv::SetPowerState::Request &req,
                           rokae_xmate3_ros2::srv::SetPowerState::Response &res) const;
  void handleSetOperateMode(const rokae_xmate3_ros2::srv::SetOperateMode::Request &req,
                            rokae_xmate3_ros2::srv::SetOperateMode::Response &res) const;
  void handleClearServoAlarm(const rokae_xmate3_ros2::srv::ClearServoAlarm::Request &req,
                             rokae_xmate3_ros2::srv::ClearServoAlarm::Response &res) const;
  void handleEnableCollisionDetection(const rokae_xmate3_ros2::srv::EnableCollisionDetection::Request &req,
                                      rokae_xmate3_ros2::srv::EnableCollisionDetection::Response &res) const;
  void handleDisableCollisionDetection(const rokae_xmate3_ros2::srv::DisableCollisionDetection::Request &req,
                                       rokae_xmate3_ros2::srv::DisableCollisionDetection::Response &res) const;
  void handleSetSoftLimit(const rokae_xmate3_ros2::srv::SetSoftLimit::Request &req,
                          rokae_xmate3_ros2::srv::SetSoftLimit::Response &res) const;
  void handleSetMotionControlMode(const rokae_xmate3_ros2::srv::SetMotionControlMode::Request &req,
                                  rokae_xmate3_ros2::srv::SetMotionControlMode::Response &res) const;
  void handleMoveReset(const rokae_xmate3_ros2::srv::MoveReset::Request &req,
                       rokae_xmate3_ros2::srv::MoveReset::Response &res) const;
  void handleMoveStart(const rokae_xmate3_ros2::srv::MoveStart::Request &req,
                       rokae_xmate3_ros2::srv::MoveStart::Response &res) const;
  void handleStop(const rokae_xmate3_ros2::srv::Stop::Request &req,
                  rokae_xmate3_ros2::srv::Stop::Response &res) const;
  void handleSetDefaultSpeed(const rokae_xmate3_ros2::srv::SetDefaultSpeed::Request &req,
                             rokae_xmate3_ros2::srv::SetDefaultSpeed::Response &res) const;
  void handleSetDefaultZone(const rokae_xmate3_ros2::srv::SetDefaultZone::Request &req,
                            rokae_xmate3_ros2::srv::SetDefaultZone::Response &res) const;
  void handleSetDefaultConfOpt(const rokae_xmate3_ros2::srv::SetDefaultConfOpt::Request &req,
                               rokae_xmate3_ros2::srv::SetDefaultConfOpt::Response &res) const;
  void handleAdjustSpeedOnline(const rokae_xmate3_ros2::srv::AdjustSpeedOnline::Request &req,
                               rokae_xmate3_ros2::srv::AdjustSpeedOnline::Response &res) const;
  void handleSetRtControlMode(const rokae_xmate3_ros2::srv::SetRtControlMode::Request &req,
                              rokae_xmate3_ros2::srv::SetRtControlMode::Response &res) const;
  void handleSetSimulationMode(const rokae_xmate3_ros2::srv::SetSimulationMode::Request &req,
                               rokae_xmate3_ros2::srv::SetSimulationMode::Response &res) const;
  void handleEnableDrag(const rokae_xmate3_ros2::srv::EnableDrag::Request &req,
                        rokae_xmate3_ros2::srv::EnableDrag::Response &res) const;
  void handleDisableDrag(const rokae_xmate3_ros2::srv::DisableDrag::Request &req,
                         rokae_xmate3_ros2::srv::DisableDrag::Response &res) const;
  void handleSetAvoidSingularity(const rokae_xmate3_ros2::srv::SetAvoidSingularity::Request &req,
                                 rokae_xmate3_ros2::srv::SetAvoidSingularity::Response &res) const;

 private:
  void stopRuntime(const std::string &message) const;
  void clearBackendControl() const;
  void syncBrakeState(bool locked) const;

  SessionState &session_state_;
  MotionOptionsState &motion_options_state_;
  BackendInterface *backend_;
  MotionRuntime *motion_runtime_;
  MotionRequestCoordinator *request_coordinator_;
};

class QueryFacade {
 public:
  QueryFacade(SessionState &session_state,
              MotionOptionsState &motion_options_state,
              ToolingState &tooling_state,
              DataStoreState &data_store_state,
              ProgramState &program_state,
              RuntimeDiagnosticsState &diagnostics_state,
              gazebo::xMate3Kinematics &kinematics,
              JointStateFetcher joint_state_fetcher,
              TimeProvider time_provider,
              TrajectoryDtProvider trajectory_dt_provider,
              int joint_num);

  void handleGetPowerState(const rokae_xmate3_ros2::srv::GetPowerState::Request &req,
                           rokae_xmate3_ros2::srv::GetPowerState::Response &res) const;
  void handleGetRuntimeDiagnostics(const rokae_xmate3_ros2::srv::GetRuntimeDiagnostics::Request &req,
                                   rokae_xmate3_ros2::srv::GetRuntimeDiagnostics::Response &res) const;
  void handleGetProfileCapabilities(const rokae_xmate3_ros2::srv::GetProfileCapabilities::Request &req,
                                     rokae_xmate3_ros2::srv::GetProfileCapabilities::Response &res) const;
  void handleGetInfo(const rokae_xmate3_ros2::srv::GetInfo::Request &req,
                     rokae_xmate3_ros2::srv::GetInfo::Response &res) const;
  void handleGetOperateMode(const rokae_xmate3_ros2::srv::GetOperateMode::Request &req,
                            rokae_xmate3_ros2::srv::GetOperateMode::Response &res) const;
  void handleQueryControllerLog(const rokae_xmate3_ros2::srv::QueryControllerLog::Request &req,
                                rokae_xmate3_ros2::srv::QueryControllerLog::Response &res) const;
  void handleGetJointPos(const rokae_xmate3_ros2::srv::GetJointPos::Request &req,
                         rokae_xmate3_ros2::srv::GetJointPos::Response &res) const;
  void handleGetJointVel(const rokae_xmate3_ros2::srv::GetJointVel::Request &req,
                         rokae_xmate3_ros2::srv::GetJointVel::Response &res) const;
  void handleGetJointTorques(const rokae_xmate3_ros2::srv::GetJointTorques::Request &req,
                            rokae_xmate3_ros2::srv::GetJointTorques::Response &res) const;
  void handleGetPosture(const rokae_xmate3_ros2::srv::GetPosture::Request &req,
                        rokae_xmate3_ros2::srv::GetPosture::Response &res) const;
  void handleGetCartPosture(const rokae_xmate3_ros2::srv::GetCartPosture::Request &req,
                            rokae_xmate3_ros2::srv::GetCartPosture::Response &res) const;
  void handleGetBaseFrame(const rokae_xmate3_ros2::srv::GetBaseFrame::Request &req,
                          rokae_xmate3_ros2::srv::GetBaseFrame::Response &res) const;
  void handleCalcFk(const rokae_xmate3_ros2::srv::CalcFk::Request &req,
                    rokae_xmate3_ros2::srv::CalcFk::Response &res) const;
  void handleCalcIk(const rokae_xmate3_ros2::srv::CalcIk::Request &req,
                    rokae_xmate3_ros2::srv::CalcIk::Response &res) const;
  void handleGetToolset(const rokae_xmate3_ros2::srv::GetToolset::Request &req,
                        rokae_xmate3_ros2::srv::GetToolset::Response &res) const;
  void handleSetToolset(const rokae_xmate3_ros2::srv::SetToolset::Request &req,
                        rokae_xmate3_ros2::srv::SetToolset::Response &res) const;
  void handleSetToolsetByName(const rokae_xmate3_ros2::srv::SetToolsetByName::Request &req,
                              rokae_xmate3_ros2::srv::SetToolsetByName::Response &res) const;
  void handleGetSoftLimit(const rokae_xmate3_ros2::srv::GetSoftLimit::Request &req,
                          rokae_xmate3_ros2::srv::GetSoftLimit::Response &res) const;
  void handleGetRtJointData(const rokae_xmate3_ros2::srv::GetRtJointData::Request &req,
                            rokae_xmate3_ros2::srv::GetRtJointData::Response &res) const;
  void handleGetAvoidSingularity(const rokae_xmate3_ros2::srv::GetAvoidSingularity::Request &req,
                                 rokae_xmate3_ros2::srv::GetAvoidSingularity::Response &res) const;
  void handleGetRlProjectInfo(const rokae_xmate3_ros2::srv::GetRlProjectInfo::Request &req,
                              rokae_xmate3_ros2::srv::GetRlProjectInfo::Response &res) const;
  void handleGetToolCatalog(const rokae_xmate3_ros2::srv::GetToolCatalog::Request &req,
                            rokae_xmate3_ros2::srv::GetToolCatalog::Response &res) const;
  void handleGetWobjCatalog(const rokae_xmate3_ros2::srv::GetWobjCatalog::Request &req,
                            rokae_xmate3_ros2::srv::GetWobjCatalog::Response &res) const;
  void handleCalcJointTorque(const rokae_xmate3_ros2::srv::CalcJointTorque::Request &req,
                             rokae_xmate3_ros2::srv::CalcJointTorque::Response &res) const;
  void handleGenerateSTrajectory(const rokae_xmate3_ros2::srv::GenerateSTrajectory::Request &req,
                                 rokae_xmate3_ros2::srv::GenerateSTrajectory::Response &res) const;
  void handleValidateMotion(const rokae_xmate3_ros2::srv::ValidateMotion::Request &req,
                            rokae_xmate3_ros2::srv::ValidateMotion::Response &res) const;
  void handleMapCartesianToJointTorque(const rokae_xmate3_ros2::srv::MapCartesianToJointTorque::Request &req,
                                       rokae_xmate3_ros2::srv::MapCartesianToJointTorque::Response &res) const;
  void handleGetEndEffectorTorque(const rokae_xmate3_ros2::srv::GetEndEffectorTorque::Request &req,
                          rokae_xmate3_ros2::srv::GetEndEffectorTorque::Response &res) const;
  void handleGetEndWrench(const rokae_xmate3_ros2::srv::GetEndWrench::Request &req,
                          rokae_xmate3_ros2::srv::GetEndWrench::Response &res) const;

 private:
  SessionState &session_state_;
  MotionOptionsState &motion_options_state_;
  ToolingState &tooling_state_;
  DataStoreState &data_store_state_;
  ProgramState &program_state_;
  RuntimeDiagnosticsState &diagnostics_state_;
  gazebo::xMate3Kinematics &kinematics_;
  JointStateFetcher joint_state_fetcher_;
  TimeProvider time_provider_;
  TrajectoryDtProvider trajectory_dt_provider_;
  int joint_num_;
};

class IoProgramFacade {
 public:
  explicit IoProgramFacade(DataStoreState &data_store_state,
                           ProgramState &program_state,
                           ToolingState &tooling_state,
                           TimeProvider time_provider);

  void handleSendCustomData(const rokae_xmate3_ros2::srv::SendCustomData::Request &req,
                            rokae_xmate3_ros2::srv::SendCustomData::Response &res) const;
  void handleRegisterDataCallback(const rokae_xmate3_ros2::srv::RegisterDataCallback::Request &req,
                                  rokae_xmate3_ros2::srv::RegisterDataCallback::Response &res) const;
  void handleReadRegister(const rokae_xmate3_ros2::srv::ReadRegister::Request &req,
                          rokae_xmate3_ros2::srv::ReadRegister::Response &res) const;
  void handleReadRegisterEx(const rokae_xmate3_ros2::srv::ReadRegisterEx::Request &req,
                            rokae_xmate3_ros2::srv::ReadRegisterEx::Response &res) const;
  void handleWriteRegister(const rokae_xmate3_ros2::srv::WriteRegister::Request &req,
                           rokae_xmate3_ros2::srv::WriteRegister::Response &res) const;
  void handleWriteRegisterEx(const rokae_xmate3_ros2::srv::WriteRegisterEx::Request &req,
                             rokae_xmate3_ros2::srv::WriteRegisterEx::Response &res) const;
  void handleSetXPanelVout(const rokae_xmate3_ros2::srv::SetXPanelVout::Request &req,
                           rokae_xmate3_ros2::srv::SetXPanelVout::Response &res) const;
  void handleLoadRlProject(const rokae_xmate3_ros2::srv::LoadRLProject::Request &req,
                           rokae_xmate3_ros2::srv::LoadRLProject::Response &res) const;
  void handleStartRlProject(const rokae_xmate3_ros2::srv::StartRLProject::Request &req,
                            rokae_xmate3_ros2::srv::StartRLProject::Response &res) const;
  void handleStopRlProject(const rokae_xmate3_ros2::srv::StopRLProject::Request &req,
                           rokae_xmate3_ros2::srv::StopRLProject::Response &res) const;
  void handlePauseRlProject(const rokae_xmate3_ros2::srv::PauseRLProject::Request &req,
                            rokae_xmate3_ros2::srv::PauseRLProject::Response &res) const;
  void handleSetProjectRunningOpt(const rokae_xmate3_ros2::srv::SetProjectRunningOpt::Request &req,
                                  rokae_xmate3_ros2::srv::SetProjectRunningOpt::Response &res) const;
  void handleGetDI(const rokae_xmate3_ros2::srv::GetDI::Request &req,
                   rokae_xmate3_ros2::srv::GetDI::Response &res) const;
  void handleGetDO(const rokae_xmate3_ros2::srv::GetDO::Request &req,
                   rokae_xmate3_ros2::srv::GetDO::Response &res) const;
  void handleSetDI(const rokae_xmate3_ros2::srv::SetDI::Request &req,
                   rokae_xmate3_ros2::srv::SetDI::Response &res) const;
  void handleSetDO(const rokae_xmate3_ros2::srv::SetDO::Request &req,
                   rokae_xmate3_ros2::srv::SetDO::Response &res) const;
  void handleGetAI(const rokae_xmate3_ros2::srv::GetAI::Request &req,
                   rokae_xmate3_ros2::srv::GetAI::Response &res) const;
  void handleSetAO(const rokae_xmate3_ros2::srv::SetAO::Request &req,
                   rokae_xmate3_ros2::srv::SetAO::Response &res) const;

 private:
  DataStoreState &data_store_state_;
  ProgramState &program_state_;
  ToolingState &tooling_state_;
  TimeProvider time_provider_;
};

class PathFacade {
 public:
  PathFacade(ProgramState &program_state,
             ToolingState &tooling_state,
             MotionRequestCoordinator *request_coordinator,
             JointStateFetcher joint_state_fetcher,
             TrajectoryDtProvider trajectory_dt_provider,
             RequestIdGenerator request_id_generator);

  void handleStartRecordPath(const rokae_xmate3_ros2::srv::StartRecordPath::Request &req,
                             rokae_xmate3_ros2::srv::StartRecordPath::Response &res) const;
  void handleStopRecordPath(const rokae_xmate3_ros2::srv::StopRecordPath::Request &req,
                            rokae_xmate3_ros2::srv::StopRecordPath::Response &res) const;
  void handleCancelRecordPath(const rokae_xmate3_ros2::srv::CancelRecordPath::Request &req,
                              rokae_xmate3_ros2::srv::CancelRecordPath::Response &res) const;
  void handleSaveRecordPath(const rokae_xmate3_ros2::srv::SaveRecordPath::Request &req,
                            rokae_xmate3_ros2::srv::SaveRecordPath::Response &res) const;
  void handleReplayPath(const rokae_xmate3_ros2::srv::ReplayPath::Request &req,
                        rokae_xmate3_ros2::srv::ReplayPath::Response &res) const;
  void handleRemovePath(const rokae_xmate3_ros2::srv::RemovePath::Request &req,
                        rokae_xmate3_ros2::srv::RemovePath::Response &res) const;
  void handleQueryPathLists(const rokae_xmate3_ros2::srv::QueryPathLists::Request &req,
                            rokae_xmate3_ros2::srv::QueryPathLists::Response &res) const;


 private:
  ProgramState &program_state_;
  ToolingState &tooling_state_;
  MotionRequestCoordinator *request_coordinator_;
  JointStateFetcher joint_state_fetcher_;
  TrajectoryDtProvider trajectory_dt_provider_;
  RequestIdGenerator request_id_generator_;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
