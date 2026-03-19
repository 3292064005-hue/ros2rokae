#include "runtime/service_facade.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <utility>

#include "runtime/planning_utils.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

static builtin_interfaces::msg::Time ToBuiltinTime(const rclcpp::Time &time) {
  builtin_interfaces::msg::Time stamp;
  const auto ns = time.nanoseconds();
  stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return stamp;
}

std::string basename_without_extension(const std::string &path) {
  std::string name = path;
  const auto slash = name.find_last_of("/\\");
  if (slash != std::string::npos) {
    name = name.substr(slash + 1);
  }
  const auto dot = name.find_last_of('.');
  if (dot != std::string::npos) {
    name = name.substr(0, dot);
  }
  return name.empty() ? std::string("default_rl_project") : name;
}

std::vector<double> snapshot_joints(const std::array<double, 6> &position) {
  return std::vector<double>(position.begin(), position.end());
}

}  // namespace

ControlFacade::ControlFacade(SessionState &session_state,
                             MotionOptionsState &motion_options_state,
                             BackendInterface *backend,
                             MotionRuntime *motion_runtime,
                             MotionRequestCoordinator *request_coordinator)
    : session_state_(session_state),
      motion_options_state_(motion_options_state),
      backend_(backend),
      motion_runtime_(motion_runtime),
      request_coordinator_(request_coordinator) {}

void ControlFacade::stopRuntime(const std::string &message) const {
  if (request_coordinator_ != nullptr) {
    request_coordinator_->stop(message);
    return;
  }
  if (motion_runtime_ != nullptr) {
    motion_runtime_->stop(message);
  }
}

void ControlFacade::clearBackendControl() const {
  if (backend_ != nullptr) {
    backend_->clearControl();
  }
}

void ControlFacade::syncBrakeState(bool locked) const {
  if (backend_ == nullptr) {
    return;
  }
  const auto snapshot = backend_->readSnapshot();
  if (backend_->brakesLocked() != locked) {
    backend_->setBrakeLock(snapshot, locked);
  }
  if (locked) {
    backend_->clearControl();
  }
}

void ControlFacade::handleConnect(const rokae_xmate3_ros2::srv::Connect::Request &req,
                                  rokae_xmate3_ros2::srv::Connect::Response &res) const {
  session_state_.connect(req.remote_ip);
  res.success = true;
  res.message = "Connected to xCore Gazebo Controller";
}

void ControlFacade::handleDisconnect(const rokae_xmate3_ros2::srv::Disconnect::Request &req,
                                     rokae_xmate3_ros2::srv::Disconnect::Response &res) const {
  (void)req;
  stopRuntime("disconnect");
  session_state_.setDragMode(false);
  session_state_.setPowerOn(false);
  session_state_.disconnect();
  syncBrakeState(true);
  res.success = true;
}

void ControlFacade::handleSetPowerState(const rokae_xmate3_ros2::srv::SetPowerState::Request &req,
                                        rokae_xmate3_ros2::srv::SetPowerState::Response &res) const {
  if (!session_state_.connected()) {
    res.success = false;
    res.message = "Robot not connected";
    return;
  }
  session_state_.setPowerOn(req.on);
  if (!req.on) {
    session_state_.setDragMode(false);
    stopRuntime("power off");
    syncBrakeState(true);
  } else {
    syncBrakeState(false);
  }
  res.success = true;
}

void ControlFacade::handleSetOperateMode(const rokae_xmate3_ros2::srv::SetOperateMode::Request &req,
                                         rokae_xmate3_ros2::srv::SetOperateMode::Response &res) const {
  session_state_.setOperateMode(req.mode);
  res.success = true;
}

void ControlFacade::handleClearServoAlarm(const rokae_xmate3_ros2::srv::ClearServoAlarm::Request &req,
                                          rokae_xmate3_ros2::srv::ClearServoAlarm::Response &res) const {
  (void)req;
  res.success = true;
}

void ControlFacade::handleEnableCollisionDetection(
    const rokae_xmate3_ros2::srv::EnableCollisionDetection::Request &req,
    rokae_xmate3_ros2::srv::EnableCollisionDetection::Response &res) const {
  (void)req;
  session_state_.setCollisionDetectionEnabled(true);
  res.success = true;
}

void ControlFacade::handleDisableCollisionDetection(
    const rokae_xmate3_ros2::srv::DisableCollisionDetection::Request &req,
    rokae_xmate3_ros2::srv::DisableCollisionDetection::Response &res) const {
  (void)req;
  session_state_.setCollisionDetectionEnabled(false);
  res.success = true;
}

void ControlFacade::handleSetSoftLimit(const rokae_xmate3_ros2::srv::SetSoftLimit::Request &req,
                                       rokae_xmate3_ros2::srv::SetSoftLimit::Response &res) const {
  auto soft_limit = motion_options_state_.softLimit();
  if (req.limits.size() >= 12) {
    for (int i = 0; i < 6; ++i) {
      soft_limit.limits[i][0] = req.limits[i * 2];
      soft_limit.limits[i][1] = req.limits[i * 2 + 1];
    }
  }
  motion_options_state_.setSoftLimit(req.enable, soft_limit.limits);
  res.success = true;
}

void ControlFacade::handleSetMotionControlMode(
    const rokae_xmate3_ros2::srv::SetMotionControlMode::Request &req,
    rokae_xmate3_ros2::srv::SetMotionControlMode::Response &res) const {
  if (session_state_.motionMode() != req.mode) {
    stopRuntime("motion control mode changed");
    clearBackendControl();
  }
  session_state_.setMotionMode(req.mode);
  res.success = true;
}

void ControlFacade::handleMoveReset(const rokae_xmate3_ros2::srv::MoveReset::Request &req,
                                    rokae_xmate3_ros2::srv::MoveReset::Response &res) const {
  (void)req;
  if (request_coordinator_ != nullptr) {
    request_coordinator_->reset();
  } else if (motion_runtime_ != nullptr) {
    motion_runtime_->reset();
  }
  clearBackendControl();
  res.success = true;
}

void ControlFacade::handleMoveStart(const rokae_xmate3_ros2::srv::MoveStart::Request &req,
                                    rokae_xmate3_ros2::srv::MoveStart::Response &res) const {
  (void)req;
  if (!session_state_.powerOn()) {
    res.success = false;
    res.message = "Robot not powered on";
    return;
  }
  res.success = true;
}

void ControlFacade::handleStop(const rokae_xmate3_ros2::srv::Stop::Request &req,
                               rokae_xmate3_ros2::srv::Stop::Response &res) const {
  (void)req;
  stopRuntime("stop service called");
  clearBackendControl();
  res.success = true;
}

void ControlFacade::handleSetDefaultSpeed(const rokae_xmate3_ros2::srv::SetDefaultSpeed::Request &req,
                                          rokae_xmate3_ros2::srv::SetDefaultSpeed::Response &res) const {
  motion_options_state_.setDefaultSpeed(req.speed);
  res.success = true;
}

void ControlFacade::handleSetDefaultZone(const rokae_xmate3_ros2::srv::SetDefaultZone::Request &req,
                                         rokae_xmate3_ros2::srv::SetDefaultZone::Response &res) const {
  motion_options_state_.setDefaultZone(req.zone);
  res.success = true;
}

void ControlFacade::handleSetDefaultConfOpt(
    const rokae_xmate3_ros2::srv::SetDefaultConfOpt::Request &req,
    rokae_xmate3_ros2::srv::SetDefaultConfOpt::Response &res) const {
  motion_options_state_.setDefaultConfOpt(req.forced);
  res.success = true;
}

void ControlFacade::handleAdjustSpeedOnline(
    const rokae_xmate3_ros2::srv::AdjustSpeedOnline::Request &req,
    rokae_xmate3_ros2::srv::AdjustSpeedOnline::Response &res) const {
  motion_options_state_.setSpeedScale(req.scale);
  res.success = true;
}

void ControlFacade::handleSetRtControlMode(const rokae_xmate3_ros2::srv::SetRtControlMode::Request &req,
                                           rokae_xmate3_ros2::srv::SetRtControlMode::Response &res) const {
  if (!session_state_.powerOn()) {
    res.success = false;
    res.error_code = 1002;
    res.error_msg = "robot power is off";
    return;
  }
  if (req.mode < 0 || req.mode > 4) {
    res.success = false;
    res.error_code = 1001;
    res.error_msg = "invalid realtime control mode";
    return;
  }
  stopRuntime("rt control mode changed");
  clearBackendControl();
  session_state_.setRtControlMode(req.mode);
  session_state_.setMotionMode(1);
  syncBrakeState(false);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void ControlFacade::handleSetSimulationMode(
    const rokae_xmate3_ros2::srv::SetSimulationMode::Request &req,
    rokae_xmate3_ros2::srv::SetSimulationMode::Response &res) const {
  session_state_.setSimulationMode(req.state);
  res.success = true;
}

void ControlFacade::handleEnableDrag(const rokae_xmate3_ros2::srv::EnableDrag::Request &req,
                                     rokae_xmate3_ros2::srv::EnableDrag::Response &res) const {
  (void)req;
  session_state_.setDragMode(true);
  stopRuntime("drag mode enabled");
  syncBrakeState(false);
  clearBackendControl();
  res.success = true;
}

void ControlFacade::handleDisableDrag(const rokae_xmate3_ros2::srv::DisableDrag::Request &req,
                                      rokae_xmate3_ros2::srv::DisableDrag::Response &res) const {
  (void)req;
  session_state_.setDragMode(false);
  clearBackendControl();
  res.success = true;
}

void ControlFacade::handleSetAvoidSingularity(
    const rokae_xmate3_ros2::srv::SetAvoidSingularity::Request &req,
    rokae_xmate3_ros2::srv::SetAvoidSingularity::Response &res) const {
  motion_options_state_.setAvoidSingularity(req.enable);
  res.success = true;
  res.message = motion_options_state_.avoidSingularityEnabled() ? "enabled" : "disabled";
}

QueryFacade::QueryFacade(SessionState &session_state,
                         MotionOptionsState &motion_options_state,
                         ToolingState &tooling_state,
                         DataStoreState &data_store_state,
                         ProgramState &program_state,
                         gazebo::xMate3Kinematics &kinematics,
                         JointStateFetcher joint_state_fetcher,
                         TimeProvider time_provider,
                         TrajectoryDtProvider trajectory_dt_provider,
                         int joint_num)
    : session_state_(session_state),
      motion_options_state_(motion_options_state),
      tooling_state_(tooling_state),
      data_store_state_(data_store_state),
      program_state_(program_state),
      kinematics_(kinematics),
      joint_state_fetcher_(std::move(joint_state_fetcher)),
      time_provider_(std::move(time_provider)),
      trajectory_dt_provider_(std::move(trajectory_dt_provider)),
      joint_num_(joint_num) {}

void QueryFacade::handleGetPowerState(const rokae_xmate3_ros2::srv::GetPowerState::Request &req,
                                      rokae_xmate3_ros2::srv::GetPowerState::Response &res) const {
  (void)req;
  res.state.state = session_state_.powerOn() ? rokae_xmate3_ros2::msg::PowerState::ON
                                                : rokae_xmate3_ros2::msg::PowerState::OFF;
  res.success = true;
}

void QueryFacade::handleGetInfo(const rokae_xmate3_ros2::srv::GetInfo::Request &req,
                                rokae_xmate3_ros2::srv::GetInfo::Response &res) const {
  (void)req;
  res.model = "xMate3";
  res.robot_type = "3";
  res.serial_number = "SIMULATION";
  res.firmware_version = "2.1.0";
  res.sdk_version = "2.1.0";
  res.joint_num = 6;
  res.success = true;
}

void QueryFacade::handleGetOperateMode(const rokae_xmate3_ros2::srv::GetOperateMode::Request &req,
                                       rokae_xmate3_ros2::srv::GetOperateMode::Response &res) const {
  (void)req;
  res.mode = session_state_.operateMode();
  res.success = true;
}

void QueryFacade::handleQueryControllerLog(
    const rokae_xmate3_ros2::srv::QueryControllerLog::Request &req,
    rokae_xmate3_ros2::srv::QueryControllerLog::Response &res) const {
  res.logs = data_store_state_.queryLogs(req.count);
  res.success = true;
}

void QueryFacade::handleGetJointPos(const rokae_xmate3_ros2::srv::GetJointPos::Request &req,
                                    rokae_xmate3_ros2::srv::GetJointPos::Response &res) const {
  (void)req;
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  for (int i = 0; i < 6 && i < joint_num_; ++i) {
    res.joint_positions[i] = pos[i];
  }
  res.success = true;
}

void QueryFacade::handleGetJointVel(const rokae_xmate3_ros2::srv::GetJointVel::Request &req,
                                    rokae_xmate3_ros2::srv::GetJointVel::Response &res) const {
  (void)req;
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  for (int i = 0; i < 6 && i < joint_num_; ++i) {
    res.joint_vel[i] = vel[i];
  }
  res.success = true;
}

void QueryFacade::handleGetJointTorque(const rokae_xmate3_ros2::srv::GetJointTorque::Request &req,
                                       rokae_xmate3_ros2::srv::GetJointTorque::Response &res) const {
  (void)req;
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  for (int i = 0; i < 6 && i < joint_num_; ++i) {
    res.joint_torque[i] = tau[i];
  }
  res.success = true;
}

void QueryFacade::handleGetPosture(const rokae_xmate3_ros2::srv::GetPosture::Request &req,
                                   rokae_xmate3_ros2::srv::GetPosture::Response &res) const {
  (void)req;
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  const auto pose = kinematics_.forwardKinematicsRPY(snapshot_joints(pos));
  for (int i = 0; i < 6; ++i) {
    res.posture[i] = pose[i];
  }
  res.success = true;
}

void QueryFacade::handleGetCartPosture(const rokae_xmate3_ros2::srv::GetCartPosture::Request &req,
                                       rokae_xmate3_ros2::srv::GetCartPosture::Response &res) const {
  (void)req;
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  const auto pose = kinematics_.forwardKinematicsRPY(snapshot_joints(pos));
  res.x = pose[0];
  res.y = pose[1];
  res.z = pose[2];
  res.rx = pose[3];
  res.ry = pose[4];
  res.rz = pose[5];
  res.success = true;
}

void QueryFacade::handleGetBaseFrame(const rokae_xmate3_ros2::srv::GetBaseFrame::Request &req,
                                     rokae_xmate3_ros2::srv::GetBaseFrame::Response &res) const {
  (void)req;
  res.base_frame = {0, 0, 0, 0, 0, 0};
  res.success = true;
}

void QueryFacade::handleCalcFk(const rokae_xmate3_ros2::srv::CalcFk::Request &req,
                               rokae_xmate3_ros2::srv::CalcFk::Response &res) const {
  std::vector<double> joints(req.joint_positions.begin(), req.joint_positions.end());
  const auto pose = kinematics_.forwardKinematicsRPY(joints);
  for (int i = 0; i < 6; ++i) {
    res.posture[i] = pose[i];
  }
  res.success = true;
}

void QueryFacade::handleCalcIk(const rokae_xmate3_ros2::srv::CalcIk::Request &req,
                               rokae_xmate3_ros2::srv::CalcIk::Response &res) const {
  std::vector<double> target(req.target_posture.begin(), req.target_posture.end());
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  const auto current = snapshot_joints(pos);
  const auto candidates = kinematics_.inverseKinematicsMultiSolution(target, current);
  const auto soft_limit = motion_options_state_.softLimit();
  const auto selected = select_ik_solution(
      kinematics_, candidates, target, current, req.conf_data,
      motion_options_state_.defaultConfOptForced(), true, soft_limit.enabled, soft_limit.limits);
  if (!selected.success) {
    res.success = false;
    res.message = selected.message;
    return;
  }
  for (size_t i = 0; i < 6 && i < selected.joints.size(); ++i) {
    res.joint_positions[i] = selected.joints[i];
  }
  res.success = true;
  res.message.clear();
}

void QueryFacade::handleGetToolset(const rokae_xmate3_ros2::srv::GetToolset::Request &req,
                                   rokae_xmate3_ros2::srv::GetToolset::Response &res) const {
  (void)req;
  const auto toolset = tooling_state_.toolset();
  res.tool_name = toolset.tool_name;
  res.wobj_name = toolset.wobj_name;
  res.tool_pose = toolset.tool_pose;
  res.wobj_pose = toolset.wobj_pose;
  res.success = true;
}

void QueryFacade::handleSetToolset(const rokae_xmate3_ros2::srv::SetToolset::Request &req,
                                   rokae_xmate3_ros2::srv::SetToolset::Response &res) const {
  tooling_state_.setToolset(req.tool_name, req.wobj_name, req.tool_pose, req.wobj_pose);
  res.success = true;
}

void QueryFacade::handleSetToolsetByName(const rokae_xmate3_ros2::srv::SetToolsetByName::Request &req,
                                         rokae_xmate3_ros2::srv::SetToolsetByName::Response &res) const {
  (void)req;
  res.success = true;
}

void QueryFacade::handleGetSoftLimit(const rokae_xmate3_ros2::srv::GetSoftLimit::Request &req,
                                     rokae_xmate3_ros2::srv::GetSoftLimit::Response &res) const {
  (void)req;
  const auto soft_limit = motion_options_state_.softLimit();
  res.enable = soft_limit.enabled;
  for (int i = 0; i < 6; ++i) {
    res.limits[i * 2] = soft_limit.limits[i][0];
    res.limits[i * 2 + 1] = soft_limit.limits[i][1];
  }
  res.success = true;
}

void QueryFacade::handleGetRtJointData(const rokae_xmate3_ros2::srv::GetRtJointData::Request &req,
                                       rokae_xmate3_ros2::srv::GetRtJointData::Response &res) const {
  (void)req;
  if (session_state_.rtControlMode() < 0) {
    res.success = false;
    res.error_code = 2001;
    res.error_msg = "realtime control mode is not active";
    return;
  }
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  for (int i = 0; i < 6 && i < joint_num_; ++i) {
    res.joint_position[i] = pos[i];
    res.joint_velocity[i] = vel[i];
    res.joint_torque[i] = tau[i];
  }
  res.stamp = ToBuiltinTime(time_provider_());
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void QueryFacade::handleGetAvoidSingularity(
    const rokae_xmate3_ros2::srv::GetAvoidSingularity::Request &req,
    rokae_xmate3_ros2::srv::GetAvoidSingularity::Response &res) const {
  (void)req;
  res.success = true;
  res.enabled = motion_options_state_.avoidSingularityEnabled();
  res.message = motion_options_state_.avoidSingularityEnabled() ? "enabled" : "disabled";
}

void QueryFacade::handleCalcJointTorque(
    const rokae_xmate3_ros2::srv::CalcJointTorque::Request &req,
    rokae_xmate3_ros2::srv::CalcJointTorque::Response &res) const {
  std::vector<double> joints(req.joint_pos.begin(), req.joint_pos.end());
  if (joints.size() != 6) {
    res.success = false;
    res.error_code = 4002;
    res.error_msg = "joint_pos size is invalid";
    return;
  }
  const Eigen::MatrixXd jacobian = kinematics_.computeJacobian(joints);
  Eigen::Matrix<double, 6, 1> wrench;
  for (int i = 0; i < 6; ++i) {
    wrench(i) = req.external_force[i];
  }
  const Eigen::Matrix<double, 6, 1> tau_ext = jacobian.transpose() * wrench;
  for (int i = 0; i < 6; ++i) {
    res.gravity_torque[i] = std::sin(req.joint_pos[i]) * 1.5;
    res.coriolis_torque[i] = req.joint_vel[i] * 0.05;
    res.joint_torque[i] = res.gravity_torque[i] + res.coriolis_torque[i] + req.joint_acc[i] * 0.02 + tau_ext(i);
  }
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void QueryFacade::handleGenerateSTrajectory(
    const rokae_xmate3_ros2::srv::GenerateSTrajectory::Request &req,
    rokae_xmate3_ros2::srv::GenerateSTrajectory::Response &res) const {
  std::vector<double> start(req.start_joint_pos.begin(), req.start_joint_pos.end());
  std::vector<double> target(req.target_joint_pos.begin(), req.target_joint_pos.end());
  const auto trajectory = gazebo::TrajectoryPlanner::planJointMove(
      start, target, 32.0, std::max(trajectory_dt_provider_(), 1e-3));
  if (trajectory.empty()) {
    res.success = false;
    res.error_code = 3003;
    res.error_msg = "trajectory planning failed";
    return;
  }
  res.trajectory_points.reserve(trajectory.size());
  for (const auto &point : trajectory) {
    rokae_xmate3_ros2::msg::JointPos6 joint_msg;
    for (int i = 0; i < 6 && i < static_cast<int>(point.size()); ++i) {
      joint_msg.pos[i] = point[i];
    }
    res.trajectory_points.push_back(joint_msg);
  }
  const auto dt = std::max(trajectory_dt_provider_(), 1e-3);
  res.total_time = trajectory.size() > 1 ? (trajectory.size() - 1) * dt : 0.0;
  res.stamp = ToBuiltinTime(time_provider_());
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void QueryFacade::handleMapCartesianToJointTorque(
    const rokae_xmate3_ros2::srv::MapCartesianToJointTorque::Request &req,
    rokae_xmate3_ros2::srv::MapCartesianToJointTorque::Response &res) const {
  std::vector<double> joints(req.joint_pos.begin(), req.joint_pos.end());
  if (joints.size() != 6) {
    res.success = false;
    res.error_code = 6002;
    res.error_msg = "joint_pos size is invalid";
    return;
  }
  const Eigen::MatrixXd jacobian = kinematics_.computeJacobian(joints);
  Eigen::Matrix<double, 6, 1> wrench;
  for (int i = 0; i < 6; ++i) {
    wrench(i) = req.cart_force[i];
  }
  const Eigen::Matrix<double, 6, 1> tau = jacobian.transpose() * wrench;
  for (int i = 0; i < 6; ++i) {
    res.joint_torque[i] = tau(i);
  }
  res.stamp = ToBuiltinTime(time_provider_());
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void QueryFacade::handleGetEndTorque(const rokae_xmate3_ros2::srv::GetEndTorque::Request &req,
                                     rokae_xmate3_ros2::srv::GetEndTorque::Response &res) const {
  (void)req;
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau_array{};
  joint_state_fetcher_(pos, vel, tau_array);
  const auto joints = snapshot_joints(pos);
  Eigen::Matrix<double, 6, 1> tau = Eigen::Matrix<double, 6, 1>::Zero();
  for (int i = 0; i < 6; ++i) {
    tau(i) = tau_array[i];
  }
  const Eigen::MatrixXd jacobian = kinematics_.computeJacobian(joints);
  const Eigen::Matrix<double, 6, 1> wrench =
      jacobian.transpose().completeOrthogonalDecomposition().solve(tau);
  for (int i = 0; i < 6; ++i) {
    res.end_torque[i] = wrench(i);
  }
  res.success = true;
  res.message.clear();
}

IoProgramFacade::IoProgramFacade(DataStoreState &data_store_state, ProgramState &program_state, TimeProvider time_provider)
    : data_store_state_(data_store_state), program_state_(program_state), time_provider_(std::move(time_provider)) {}

void IoProgramFacade::handleSendCustomData(
    const rokae_xmate3_ros2::srv::SendCustomData::Request &req,
    rokae_xmate3_ros2::srv::SendCustomData::Response &res) const {
  data_store_state_.setCustomData(req.data_topic, req.custom_data);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.response_data = std::string("ACK:") + req.data_topic;
  res.send_time = ToBuiltinTime(time_provider_());
}

void IoProgramFacade::handleRegisterDataCallback(
    const rokae_xmate3_ros2::srv::RegisterDataCallback::Request &req,
    rokae_xmate3_ros2::srv::RegisterDataCallback::Response &res) const {
  if (req.data_topic.empty() || req.callback_id.empty()) {
    res.success = false;
    res.error_code = 12001;
    res.error_msg = "data_topic and callback_id must not be empty";
    return;
  }
  if (data_store_state_.hasCallback(req.callback_id)) {
    res.success = false;
    res.error_code = 12004;
    res.error_msg = "callback_id already exists";
    return;
  }
  data_store_state_.registerCallback(req.callback_id, req.data_topic);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.register_time = ToBuiltinTime(time_provider_());
}

void IoProgramFacade::handleReadRegister(const rokae_xmate3_ros2::srv::ReadRegister::Request &req,
                                         rokae_xmate3_ros2::srv::ReadRegister::Response &res) const {
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.value = data_store_state_.registerValue(req.key);
}

void IoProgramFacade::handleWriteRegister(const rokae_xmate3_ros2::srv::WriteRegister::Request &req,
                                          rokae_xmate3_ros2::srv::WriteRegister::Response &res) const {
  data_store_state_.setRegister(req.key, req.value);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void IoProgramFacade::handleLoadRlProject(const rokae_xmate3_ros2::srv::LoadRLProject::Request &req,
                                          rokae_xmate3_ros2::srv::LoadRLProject::Response &res) const {
  const auto effective_project_name = basename_without_extension(req.project_path);
  program_state_.loadRlProject(req.project_path, effective_project_name);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.project_name = program_state_.loadedRlProjectName();
  res.load_time = 0.0;
}

void IoProgramFacade::handleStartRlProject(const rokae_xmate3_ros2::srv::StartRLProject::Request &req,
                                           rokae_xmate3_ros2::srv::StartRLProject::Response &res) const {
  if (!program_state_.rlProjectLoaded() || req.project_id != program_state_.loadedRlProjectName()) {
    res.success = false;
    res.error_code = 7001;
    res.error_msg = "RL project is not loaded";
    return;
  }
  if (program_state_.rlProjectRunning()) {
    res.success = false;
    res.error_code = 7002;
    res.error_msg = "RL project is already running";
    return;
  }
  program_state_.setRlProjectRunning(true, 1);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.current_episode = program_state_.rlCurrentEpisode();
  res.start_time = ToBuiltinTime(time_provider_());
}

void IoProgramFacade::handleStopRlProject(const rokae_xmate3_ros2::srv::StopRLProject::Request &req,
                                          rokae_xmate3_ros2::srv::StopRLProject::Response &res) const {
  if (!program_state_.rlProjectRunning() || req.project_id != program_state_.loadedRlProjectName()) {
    res.success = false;
    res.error_code = 8001;
    res.error_msg = "RL project is not running";
    return;
  }
  program_state_.setRlProjectRunning(false, program_state_.rlCurrentEpisode());
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  res.finished_episode = std::max(program_state_.rlCurrentEpisode(), 1);
  res.stop_time = ToBuiltinTime(time_provider_());
}

void IoProgramFacade::handleGetDI(const rokae_xmate3_ros2::srv::GetDI::Request &req,
                                  rokae_xmate3_ros2::srv::GetDI::Response &res) const {
  res.state = data_store_state_.getDI(req.board, req.port);
  res.success = true;
}

void IoProgramFacade::handleGetDO(const rokae_xmate3_ros2::srv::GetDO::Request &req,
                                  rokae_xmate3_ros2::srv::GetDO::Response &res) const {
  res.state = data_store_state_.getDO(req.board, req.port);
  res.success = true;
}

void IoProgramFacade::handleSetDI(const rokae_xmate3_ros2::srv::SetDI::Request &req,
                                  rokae_xmate3_ros2::srv::SetDI::Response &res) const {
  data_store_state_.setDI(req.board, req.port, req.state);
  res.success = true;
}

void IoProgramFacade::handleSetDO(const rokae_xmate3_ros2::srv::SetDO::Request &req,
                                  rokae_xmate3_ros2::srv::SetDO::Response &res) const {
  data_store_state_.setDO(req.board, req.port, req.state);
  res.success = true;
}

void IoProgramFacade::handleGetAI(const rokae_xmate3_ros2::srv::GetAI::Request &req,
                                  rokae_xmate3_ros2::srv::GetAI::Response &res) const {
  res.value = data_store_state_.getAI(req.board, req.port);
  res.success = true;
}

void IoProgramFacade::handleSetAO(const rokae_xmate3_ros2::srv::SetAO::Request &req,
                                  rokae_xmate3_ros2::srv::SetAO::Response &res) const {
  data_store_state_.setAO(req.board, req.port, req.value);
  res.success = true;
}

PathFacade::PathFacade(ProgramState &program_state,
                       MotionRequestCoordinator *request_coordinator,
                       JointStateFetcher joint_state_fetcher,
                       TrajectoryDtProvider trajectory_dt_provider,
                       RequestIdGenerator request_id_generator)
    : program_state_(program_state),
      request_coordinator_(request_coordinator),
      joint_state_fetcher_(std::move(joint_state_fetcher)),
      trajectory_dt_provider_(std::move(trajectory_dt_provider)),
      request_id_generator_(std::move(request_id_generator)) {}

void PathFacade::handleStartRecordPath(const rokae_xmate3_ros2::srv::StartRecordPath::Request &req,
                                       rokae_xmate3_ros2::srv::StartRecordPath::Response &res) const {
  (void)req;
  program_state_.startRecordingPath();
  res.success = true;
}

void PathFacade::handleStopRecordPath(const rokae_xmate3_ros2::srv::StopRecordPath::Request &req,
                                      rokae_xmate3_ros2::srv::StopRecordPath::Response &res) const {
  (void)req;
  program_state_.stopRecordingPath();
  res.success = true;
}

void PathFacade::handleCancelRecordPath(
    const rokae_xmate3_ros2::srv::CancelRecordPath::Request &req,
    rokae_xmate3_ros2::srv::CancelRecordPath::Response &res) const {
  (void)req;
  program_state_.cancelRecordingPath();
  res.success = true;
}

void PathFacade::handleSaveRecordPath(const rokae_xmate3_ros2::srv::SaveRecordPath::Request &req,
                                      rokae_xmate3_ros2::srv::SaveRecordPath::Response &res) const {
  program_state_.saveRecordedPath(req.name);
  res.success = true;
}

void PathFacade::handleReplayPath(const rokae_xmate3_ros2::srv::ReplayPath::Request &req,
                                  rokae_xmate3_ros2::srv::ReplayPath::Response &res) const {
  std::vector<std::vector<double>> saved_path;
  if (!program_state_.getSavedPath(req.name, saved_path)) {
    res.success = false;
    res.message = "Path not found";
    return;
  }
  if (saved_path.empty()) {
    res.success = false;
    res.message = "Path is empty";
    return;
  }
  if (request_coordinator_ == nullptr) {
    res.success = false;
    res.message = "Runtime is not initialized";
    return;
  }
  if (!request_coordinator_->canAcceptRequest()) {
    res.success = false;
    res.message = "Runtime is busy";
    return;
  }
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  joint_state_fetcher_(pos, vel, tau);
  const auto submission = request_coordinator_->submitReplayPath(
      saved_path,
      req.rate,
      pos,
      trajectory_dt_provider_(),
      request_id_generator_(std::string("replay_") + req.name));
  res.success = submission.success;
  res.message = submission.message;
}

void PathFacade::handleRemovePath(const rokae_xmate3_ros2::srv::RemovePath::Request &req,
                                  rokae_xmate3_ros2::srv::RemovePath::Response &res) const {
  program_state_.removeSavedPath(req.name, req.remove_all);
  res.success = true;
}

void PathFacade::handleQueryPathLists(const rokae_xmate3_ros2::srv::QueryPathLists::Request &req,
                                      rokae_xmate3_ros2::srv::QueryPathLists::Response &res) const {
  (void)req;
  res.path_names = program_state_.querySavedPaths();
  res.success = true;
}

bool PathFacade::isRecording() const { return program_state_.isRecordingPath(); }

void PathFacade::recordSample(const std::array<double, 6> &joint_position) const {
  program_state_.recordPathSample(joint_position);
}

}  // namespace rokae_xmate3_ros2::runtime
