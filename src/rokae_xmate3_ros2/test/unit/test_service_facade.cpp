#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cmath>
#include <limits>

#include "runtime/joint_retimer.hpp"
#include "runtime/runtime_state.hpp"
#include "runtime/motion_runtime.hpp"
#include "runtime/pose_utils.hpp"
#include "runtime/service_facade.hpp"
#include "runtime/unified_retimer.hpp"
#include "rokae_xmate3_ros2/gazebo/model_facade.hpp"
#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

namespace {

class FakeBackend final : public rt::BackendInterface {
 public:
  rt::RobotSnapshot readSnapshot() const override { return snapshot; }

  void applyControl(const rt::ControlCommand &command) override {
    last_command = command;
    apply_count++;
  }

  void clearControl() override { clear_count++; }

  void setBrakeLock(const rt::RobotSnapshot &, bool locked) override {
    brake_locked = locked;
    brake_set_count++;
  }

  bool brakesLocked() const override { return brake_locked; }

  mutable rt::RobotSnapshot snapshot{};
  rt::ControlCommand last_command{};
  int apply_count = 0;
  int clear_count = 0;
  int brake_set_count = 0;
  bool brake_locked = false;
};

}  // namespace

TEST(ServiceFacadeTest, ControlFacadeCoordinatesPowerDragAndStopWithRuntime) {
  rt::SessionState session_state;
  rt::MotionOptionsState motion_options_state;
  rt::ToolingState tooling_state;
  rt::MotionRuntime motion_runtime;
  rt::MotionRequestCoordinator coordinator(motion_options_state, tooling_state, session_state, motion_runtime);
  FakeBackend backend;
  rt::ControlFacade facade(session_state, motion_options_state, tooling_state, &backend, &motion_runtime, &coordinator);

  rokae_xmate3_ros2::srv::Connect::Request connect_req;
  rokae_xmate3_ros2::srv::Connect::Response connect_res;
  connect_req.remote_ip = "127.0.0.1";
  facade.handleConnect(connect_req, connect_res);
  ASSERT_TRUE(connect_res.success);
  EXPECT_TRUE(session_state.connected());

  rokae_xmate3_ros2::srv::SetPowerState::Request power_req;
  rokae_xmate3_ros2::srv::SetPowerState::Response power_res;
  power_req.on = true;
  facade.handleSetPowerState(power_req, power_res);
  ASSERT_TRUE(power_res.success);
  EXPECT_TRUE(session_state.powerOn());
  EXPECT_FALSE(backend.brakesLocked());

  rokae_xmate3_ros2::srv::EnableDrag::Request drag_req;
  rokae_xmate3_ros2::srv::EnableDrag::Response drag_res;
  facade.handleEnableDrag(drag_req, drag_res);
  EXPECT_FALSE(drag_res.success);
  EXPECT_EQ(drag_res.message, "drag mode requires manual operate mode");

  rokae_xmate3_ros2::srv::SetOperateMode::Request operate_req;
  rokae_xmate3_ros2::srv::SetOperateMode::Response operate_res;
  operate_req.mode = static_cast<std::uint8_t>(rokae::OperateMode::manual);
  facade.handleSetOperateMode(operate_req, operate_res);
  ASSERT_TRUE(operate_res.success);

  power_req.on = false;
  facade.handleSetPowerState(power_req, power_res);
  ASSERT_TRUE(power_res.success);
  EXPECT_FALSE(session_state.powerOn());

  facade.handleEnableDrag(drag_req, drag_res);
  ASSERT_TRUE(drag_res.success);
  EXPECT_TRUE(session_state.dragMode());
  EXPECT_EQ(backend.clear_count, 2);

  rokae_xmate3_ros2::srv::DisableDrag::Request undrag_req;
  rokae_xmate3_ros2::srv::DisableDrag::Response undrag_res;
  facade.handleDisableDrag(undrag_req, undrag_res);
  ASSERT_TRUE(undrag_res.success);
  EXPECT_FALSE(session_state.dragMode());
  EXPECT_EQ(backend.clear_count, 3);

  rokae_xmate3_ros2::srv::AdjustSpeedOnline::Request speed_req;
  rokae_xmate3_ros2::srv::AdjustSpeedOnline::Response speed_res;
  speed_req.scale = 1.6;
  facade.handleAdjustSpeedOnline(speed_req, speed_res);
  ASSERT_TRUE(speed_res.success);
  EXPECT_DOUBLE_EQ(motion_options_state.speedScale(), 1.6);
  EXPECT_DOUBLE_EQ(motion_runtime.activeSpeedScale(), 1.6);

  rokae_xmate3_ros2::srv::EnableCollisionDetection::Request collision_req;
  rokae_xmate3_ros2::srv::EnableCollisionDetection::Response collision_res;
  collision_req.sensitivity = {2.0, 1.5, 1.0, 0.5, 1.2, 1.8};
  collision_req.behaviour = 2;
  collision_req.fallback = 0.7;
  facade.handleEnableCollisionDetection(collision_req, collision_res);
  ASSERT_TRUE(collision_res.success);
  const auto collision_snapshot = session_state.collisionDetection();
  EXPECT_TRUE(collision_snapshot.enabled);
  EXPECT_EQ(collision_snapshot.behaviour, 2u);
  EXPECT_DOUBLE_EQ(collision_snapshot.fallback, 0.7);
  EXPECT_DOUBLE_EQ(collision_snapshot.sensitivity[0], 2.0);

  rokae_xmate3_ros2::srv::DisableCollisionDetection::Request collision_disable_req;
  rokae_xmate3_ros2::srv::DisableCollisionDetection::Response collision_disable_res;
  facade.handleDisableCollisionDetection(collision_disable_req, collision_disable_res);
  ASSERT_TRUE(collision_disable_res.success);
  EXPECT_FALSE(session_state.collisionDetectionEnabled());

  rokae_xmate3_ros2::srv::Stop::Request stop_req;
  rokae_xmate3_ros2::srv::Stop::Response stop_res;
  facade.handleStop(stop_req, stop_res);
  ASSERT_TRUE(stop_res.success);
  EXPECT_EQ(backend.clear_count, 4);
  EXPECT_TRUE(motion_runtime.status().state == rt::ExecutionState::stopped ||
              motion_runtime.status().state == rt::ExecutionState::idle);

  power_req.on = false;
  facade.handleSetPowerState(power_req, power_res);
  ASSERT_TRUE(power_res.success);
  EXPECT_FALSE(session_state.powerOn());
  EXPECT_TRUE(backend.brakesLocked());
  EXPECT_GE(backend.brake_set_count, 1);
  EXPECT_TRUE(motion_runtime.status().state == rt::ExecutionState::stopped ||
              motion_runtime.status().state == rt::ExecutionState::idle);
}




TEST(ServiceFacadeTest, ControlFacadeStopPausesAndMoveStartResumesWithoutBackendSnapshot) {
  rt::SessionState session_state;
  rt::MotionOptionsState motion_options_state;
  rt::ToolingState tooling_state;
  rt::MotionRuntime motion_runtime;
  motion_runtime.reset();
  rt::MotionRequestCoordinator coordinator(motion_options_state, tooling_state, session_state, motion_runtime);
  rt::ControlFacade facade(session_state, motion_options_state, tooling_state, nullptr, &motion_runtime, &coordinator);

  session_state.connect("127.0.0.1");
  session_state.setPowerOn(true);
  session_state.setOperateMode(static_cast<std::uint8_t>(rokae::OperateMode::automatic));
  session_state.setMotionMode(rt::kSessionMotionModeNrt);

  rokae_xmate3_ros2::action::MoveAppend::Goal goal;
  goal.absj_cmds.resize(1);
  goal.absj_cmds.front().target.joints = {0.1, -0.2, 1.4, 0.0, 1.1, 3.14};
  goal.absj_cmds.front().speed = 20;
  goal.absj_cmds.front().zone = 5;

  const std::array<double, 6> current = {0.0, 0.0, 1.5, 0.0, 1.2, 3.14};
  const auto queued = coordinator.queueMoveAppend(goal, current, 0.01, "req_pause_resume");
  ASSERT_TRUE(queued.success) << queued.message;

  rokae_xmate3_ros2::srv::Stop::Request stop_req;
  rokae_xmate3_ros2::srv::Stop::Response stop_res;
  facade.handleStop(stop_req, stop_res);
  ASSERT_TRUE(stop_res.success);

  const auto paused_view = motion_runtime.view();
  EXPECT_TRUE(paused_view.has_request);
  EXPECT_TRUE(paused_view.queue_has_pending_commands);
  EXPECT_EQ(paused_view.status.request_id, "req_pause_resume");
  EXPECT_EQ(paused_view.status.state, rt::ExecutionState::paused);

  rokae_xmate3_ros2::srv::MoveStart::Request move_start_req;
  rokae_xmate3_ros2::srv::MoveStart::Response move_start_res;
  facade.handleMoveStart(move_start_req, move_start_res);
  ASSERT_TRUE(move_start_res.success) << move_start_res.message;

  const auto resumed_view = motion_runtime.view();
  EXPECT_EQ(resumed_view.status.request_id, "req_pause_resume");
  EXPECT_TRUE(resumed_view.has_request);
  EXPECT_FALSE(resumed_view.terminal);
  EXPECT_TRUE(resumed_view.status.state == rt::ExecutionState::planning ||
              resumed_view.status.state == rt::ExecutionState::queued);
}

TEST(ServiceFacadeTest, ControlFacadeTreatsRepeatedLifecycleAndModeRequestsAsIdempotentSuccess) {
  rt::SessionState session_state;
  rt::MotionOptionsState motion_options_state;
  rt::ToolingState tooling_state;
  FakeBackend backend;
  rt::MotionRuntime motion_runtime;
  motion_runtime.reset();
  rt::MotionRequestCoordinator coordinator(motion_options_state, tooling_state, session_state, motion_runtime);
  rt::ControlFacade facade(session_state, motion_options_state, tooling_state, &backend, &motion_runtime, &coordinator);

  rokae_xmate3_ros2::srv::Connect::Request connect_req;
  rokae_xmate3_ros2::srv::Connect::Response connect_res;
  connect_req.remote_ip = "127.0.0.1";
  facade.handleConnect(connect_req, connect_res);
  ASSERT_TRUE(connect_res.success);
  facade.handleConnect(connect_req, connect_res);
  EXPECT_TRUE(connect_res.success);
  EXPECT_EQ(connect_res.message, "connect request treated as idempotent success");

  rokae_xmate3_ros2::srv::SetPowerState::Request power_req;
  rokae_xmate3_ros2::srv::SetPowerState::Response power_res;
  power_req.on = true;
  facade.handleSetPowerState(power_req, power_res);
  ASSERT_TRUE(power_res.success);
  facade.handleSetPowerState(power_req, power_res);
  EXPECT_TRUE(power_res.success);
  EXPECT_EQ(power_res.message, "power state already matches request");

  rokae_xmate3_ros2::srv::SetOperateMode::Request operate_req;
  rokae_xmate3_ros2::srv::SetOperateMode::Response operate_res;
  operate_req.mode = static_cast<std::uint8_t>(rokae::OperateMode::manual);
  facade.handleSetOperateMode(operate_req, operate_res);
  ASSERT_TRUE(operate_res.success);
  facade.handleSetOperateMode(operate_req, operate_res);
  EXPECT_TRUE(operate_res.success);
  EXPECT_EQ(operate_res.message, "operate mode already matches request");

  rokae_xmate3_ros2::srv::SetMotionControlMode::Request motion_req;
  rokae_xmate3_ros2::srv::SetMotionControlMode::Response motion_res;
  motion_req.mode = 0;
  facade.handleSetMotionControlMode(motion_req, motion_res);
  ASSERT_TRUE(motion_res.success);
  facade.handleSetMotionControlMode(motion_req, motion_res);
  EXPECT_TRUE(motion_res.success);
  EXPECT_EQ(motion_res.message, "motion control mode already matches request");

  rokae_xmate3_ros2::srv::Disconnect::Request disconnect_req;
  rokae_xmate3_ros2::srv::Disconnect::Response disconnect_res;
  facade.handleDisconnect(disconnect_req, disconnect_res);
  ASSERT_TRUE(disconnect_res.success);
  facade.handleDisconnect(disconnect_req, disconnect_res);
  EXPECT_TRUE(disconnect_res.success);
  EXPECT_EQ(disconnect_res.message, "disconnect request treated as idempotent success");
}

TEST(ServiceFacadeTest, ControlFacadeRejectsOutOfContractCollisionAndSoftLimitRequests) {
  rt::SessionState session_state;
  rt::MotionOptionsState motion_options_state;
  rt::ToolingState tooling_state;
  FakeBackend backend;
  backend.snapshot.joint_position = {0.0, 0.0, 1.0, 0.0, 1.0, 0.0};
  rt::ControlFacade facade(session_state, motion_options_state, tooling_state, &backend, nullptr, nullptr);

  rokae_xmate3_ros2::srv::EnableCollisionDetection::Request collision_req;
  rokae_xmate3_ros2::srv::EnableCollisionDetection::Response collision_res;
  collision_req.sensitivity = {2.5, 1.0, 1.0, 1.0, 1.0, 1.0};
  collision_req.behaviour = 1;
  collision_req.fallback = 0.01;
  facade.handleEnableCollisionDetection(collision_req, collision_res);
  EXPECT_FALSE(collision_res.success);
  EXPECT_EQ(collision_res.message, "collision sensitivity must stay within [0.01, 2.0]");

  session_state.connect("127.0.0.1");
  rokae_xmate3_ros2::srv::SetSoftLimit::Request soft_req;
  rokae_xmate3_ros2::srv::SetSoftLimit::Response soft_res;
  soft_req.enable = true;
  soft_req.limits = {-3.0, 3.0, -2.0, 2.0, -2.0, 2.0, -3.0, 3.0, -2.0, 2.0, -6.0, 6.0};
  facade.handleSetSoftLimit(soft_req, soft_res);
  EXPECT_FALSE(soft_res.success);
  EXPECT_EQ(soft_res.message, "soft limit requires manual operate mode");

  session_state.setOperateMode(static_cast<int>(rokae::OperateMode::manual));
  session_state.setPowerOn(true);
  facade.handleSetSoftLimit(soft_req, soft_res);
  EXPECT_FALSE(soft_res.success);
  EXPECT_EQ(soft_res.message, "soft limit requires robot power off");

  session_state.setPowerOn(false);
  soft_req.limits[0] = -100.0;
  facade.handleSetSoftLimit(soft_req, soft_res);
  EXPECT_FALSE(soft_res.success);
  EXPECT_NE(soft_res.message.find("exceeds mechanical joint limits"), std::string::npos);
}

TEST(ServiceFacadeTest, PathFacadeSubmitsReplayRequestsThroughCoordinator) {
  rt::ProgramState program_state;
  rt::MotionOptionsState motion_options_state;
  rt::ToolingState tooling_state;
  rt::SessionState session_state;
  session_state.setPowerOn(true);
  rt::MotionRuntime motion_runtime;
  rt::MotionRequestCoordinator coordinator(motion_options_state, tooling_state, session_state, motion_runtime);

  auto joint_state_fetcher = [](std::array<double, 6> &position,
                                std::array<double, 6> &velocity,
                                std::array<double, 6> &torque) {
    position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    velocity.fill(0.0);
    torque.fill(0.0);
  };
  auto dt_provider = []() { return 0.01; };
  auto request_id_generator = [](const std::string &prefix) { return prefix + "_001"; };

  session_state.connect("127.0.0.1");
  rt::PathFacade facade(
      session_state, program_state, tooling_state, &coordinator, joint_state_fetcher, dt_provider, request_id_generator);
  coordinator.reset();

  program_state.startRecordingPath(tooling_state.toolset(), "test_record");
  program_state.recordPathSample(0.00, {0.0, 0.1, 0.2, 0.3, 0.4, 0.5}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  program_state.recordPathSample(0.20, {0.2, 0.3, 0.4, 0.5, 0.6, 0.7}, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  program_state.stopRecordingPath();
  program_state.saveRecordedPath("demo_path");

  rokae_xmate3_ros2::srv::ReplayPath::Request req;
  rokae_xmate3_ros2::srv::ReplayPath::Response res;
  req.name = "demo_path";
  req.rate = 1.0;
  facade.handleReplayPath(req, res);

  ASSERT_TRUE(res.success) << res.message;
  const auto view = coordinator.currentView();
  EXPECT_TRUE(view.has_request);
  EXPECT_EQ(view.status.request_id, "replay_demo_path_001");
  EXPECT_EQ(view.status.state, rt::ExecutionState::planning);
}

TEST(ServiceFacadeTest, PathFacadeRejectsDisconnectedBusyAndInvalidRequests) {
  rt::ProgramState program_state;
  rt::ToolingState tooling_state;
  rt::SessionState session_state;
  auto joint_state_fetcher = [](std::array<double, 6> &position,
                                std::array<double, 6> &velocity,
                                std::array<double, 6> &torque) {
    position.fill(0.0);
    velocity.fill(0.0);
    torque.fill(0.0);
  };
  rt::PathFacade facade(session_state,
                        program_state,
                        tooling_state,
                        nullptr,
                        joint_state_fetcher,
                        []() { return 0.01; },
                        [](const std::string &prefix) { return prefix + "_001"; });

  rokae_xmate3_ros2::srv::StartRecordPath::Request start_req;
  rokae_xmate3_ros2::srv::StartRecordPath::Response start_res;
  start_req.duration = 1;
  facade.handleStartRecordPath(start_req, start_res);
  EXPECT_FALSE(start_res.success);
  EXPECT_EQ(start_res.message, "Robot not connected");

  session_state.connect("127.0.0.1");
  start_req.duration = 0;
  facade.handleStartRecordPath(start_req, start_res);
  EXPECT_FALSE(start_res.success);
  EXPECT_EQ(start_res.message, "record duration must be positive");
}

TEST(ServiceFacadeTest, QueryFacadeAppliesToolingCoordinateSemanticsAndApproximateServices) {
  rt::SessionState session_state;
  rt::MotionOptionsState motion_options_state;
  rt::ToolingState tooling_state;
  rt::DataStoreState data_store_state;
  rt::ProgramState program_state;
  rt::RuntimeDiagnosticsState diagnostics_state;
  gazebo::xMate3Kinematics kinematics;

  const std::array<double, 6> joints = {0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
  auto joint_state_fetcher = [&](std::array<double, 6> &position,
                                 std::array<double, 6> &velocity,
                                 std::array<double, 6> &torque) {
    position = joints;
    velocity = {0.10, -0.05, 0.03, 0.0, 0.0, 0.0};
    torque = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5};
  };
  rt::MotionRuntime motion_runtime;
  rt::MotionRequestCoordinator coordinator(motion_options_state, tooling_state, session_state, motion_runtime);
  FakeBackend backend;
  backend.snapshot.joint_position = joints;
  backend.snapshot.joint_velocity = {0.10, -0.05, 0.03, 0.0, 0.0, 0.0};
  backend.snapshot.joint_torque = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5};
  motion_runtime.attachBackend(&backend);

  rt::QueryFacade facade(session_state,
                         motion_options_state,
                         tooling_state,
                         data_store_state,
                         program_state,
                         diagnostics_state,
                         motion_runtime,
                         coordinator,
                         kinematics,
                         joint_state_fetcher,
                         []() { return rclcpp::Time(0); },
                         []() { return 0.01; },
                         6);
  rt::ControlFacade control_facade(session_state, motion_options_state, tooling_state, nullptr, nullptr, nullptr);
  session_state.connect("127.0.0.1");

  rokae_xmate3_ros2::srv::SetToolset::Request set_tool_req;
  rokae_xmate3_ros2::srv::SetToolset::Response set_tool_res;
  set_tool_req.tool_name = "tcpA";
  set_tool_req.wobj_name = "fixtureA";
  set_tool_req.tool_pose = {0.0, 0.0, 0.10, 0.0, 0.0, 0.0};
  set_tool_req.wobj_pose = {0.20, -0.10, 0.30, 0.0, 0.0, 0.0};
  control_facade.handleSetToolset(set_tool_req, set_tool_res);
  ASSERT_TRUE(set_tool_res.success);
  tooling_state.setToolDynamics("tcpA", 0.6, {0.0, 0.0, 0.08});

  rokae_xmate3_ros2::srv::GetBaseFrame::Request base_req;
  rokae_xmate3_ros2::srv::GetBaseFrame::Response base_res;
  facade.handleGetBaseFrame(base_req, base_res);
  ASSERT_TRUE(base_res.success);
  EXPECT_DOUBLE_EQ(base_res.base_frame[0], 0.0);
  EXPECT_DOUBLE_EQ(base_res.base_frame[2], 0.0);

  rokae_xmate3_ros2::srv::GetPosture::Request flange_req;
  rokae_xmate3_ros2::srv::GetPosture::Response flange_res;
  flange_req.coordinate_type = 0;
  facade.handleGetPosture(flange_req, flange_res);
  ASSERT_TRUE(flange_res.success);

  rokae_xmate3_ros2::srv::GetPosture::Request end_req;
  rokae_xmate3_ros2::srv::GetPosture::Response end_res;
  end_req.coordinate_type = 1;
  facade.handleGetPosture(end_req, end_res);
  ASSERT_TRUE(end_res.success);

  const auto flange_pose = kinematics.forwardKinematicsRPY(std::vector<double>(joints.begin(), joints.end()));
  const auto expected_end_pose = rt::pose_utils::convertFlangeInBaseToEndInRef(
      flange_pose, std::vector<double>(set_tool_req.tool_pose.begin(), set_tool_req.tool_pose.end()),
      std::vector<double>(set_tool_req.wobj_pose.begin(), set_tool_req.wobj_pose.end()));
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(flange_res.posture[i], flange_pose[i], 1e-9);
    EXPECT_NEAR(end_res.posture[i], expected_end_pose[i], 1e-9);
  }

  rokae_xmate3_ros2::srv::SetToolsetByName::Request named_req;
  rokae_xmate3_ros2::srv::SetToolsetByName::Response named_res;
  named_req.tool_name = "tcpA";
  named_req.wobj_name = "fixtureA";
  control_facade.handleSetToolsetByName(named_req, named_res);
  EXPECT_TRUE(named_res.success);
  named_req.tool_name = "missing";
  control_facade.handleSetToolsetByName(named_req, named_res);
  EXPECT_FALSE(named_res.success);
  EXPECT_EQ(named_res.message, "unknown tool_name or wobj_name");

  rokae_xmate3_ros2::srv::CalcFk::Request fk_req;
  rokae_xmate3_ros2::srv::CalcFk::Response fk_res;
  fk_req.joint_positions = joints;
  facade.handleCalcFk(fk_req, fk_res);
  ASSERT_TRUE(fk_res.success);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(fk_res.posture[i], expected_end_pose[i], 1e-9);
  }

  rokae_xmate3_ros2::srv::CalcIk::Request ik_req;
  rokae_xmate3_ros2::srv::CalcIk::Response ik_res;
  ik_req.target_posture = fk_res.posture;
  facade.handleCalcIk(ik_req, ik_res);
  ASSERT_TRUE(ik_res.success) << ik_res.message;
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(ik_res.joint_positions[i], joints[i], 1e-3);
  }

  rokae_xmate3_ros2::srv::GetPosture::Request invalid_posture_req;
  rokae_xmate3_ros2::srv::GetPosture::Response invalid_posture_res;
  invalid_posture_req.coordinate_type = 9;
  facade.handleGetPosture(invalid_posture_req, invalid_posture_res);
  EXPECT_FALSE(invalid_posture_res.success);
  EXPECT_EQ(invalid_posture_res.message, "unsupported coordinate_type");

  rokae_xmate3_ros2::srv::GetCartPosture::Request invalid_cart_req;
  rokae_xmate3_ros2::srv::GetCartPosture::Response invalid_cart_res;
  invalid_cart_req.coordinate_type = 9;
  facade.handleGetCartPosture(invalid_cart_req, invalid_cart_res);
  EXPECT_FALSE(invalid_cart_res.success);
  EXPECT_EQ(invalid_cart_res.message, "unsupported coordinate_type");

  rokae_xmate3_ros2::srv::CalcJointTorque::Request torque_req;
  rokae_xmate3_ros2::srv::CalcJointTorque::Response torque_res;
  torque_req.joint_pos = joints;
  torque_req.joint_vel = {0.10, 0.05, -0.02, 0.0, 0.0, 0.0};
  torque_req.joint_acc = {0.2, 0.1, -0.1, 0.0, 0.0, 0.0};
  torque_req.external_force = {0.0, 0.0, 3.0, 0.0, 0.0, 0.0};
  facade.handleCalcJointTorque(torque_req, torque_res);
  ASSERT_TRUE(torque_res.success);
  const auto expected_torque = rokae_xmate3_ros2::gazebo_model::computeApproximateDynamics(
      kinematics,
      torque_req.joint_pos,
      torque_req.joint_vel,
      torque_req.joint_acc,
      torque_req.external_force,
      rokae_xmate3_ros2::gazebo_model::LoadContext{0.6, {0.0, 0.0, 0.08}});
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(torque_res.joint_torque[i], expected_torque.full[i], 1e-9);
    EXPECT_NEAR(torque_res.gravity_torque[i], expected_torque.gravity[i], 1e-9);
    EXPECT_NEAR(torque_res.coriolis_torque[i], expected_torque.coriolis[i], 1e-9);
  }
  torque_req.joint_pos[0] = std::numeric_limits<double>::quiet_NaN();
  facade.handleCalcJointTorque(torque_req, torque_res);
  EXPECT_FALSE(torque_res.success);

  rokae_xmate3_ros2::srv::GenerateSTrajectory::Request traj_req;
  rokae_xmate3_ros2::srv::GenerateSTrajectory::Response traj_res;
  traj_req.start_joint_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  traj_req.target_joint_pos = {0.2, -0.1, 0.05, 0.0, 0.0, 0.0};
  traj_req.max_velocity = 1.0;
  traj_req.max_acceleration = 2.0;
  traj_req.blend_radius = 0.1;
  traj_req.is_cartesian = false;
  facade.handleGenerateSTrajectory(traj_req, traj_res);
  ASSERT_TRUE(traj_res.success);
  EXPECT_GT(traj_res.total_time, 0.0);
  ASSERT_GT(traj_res.trajectory_points.size(), 2u);
  std::vector<double> start_joint(traj_req.start_joint_pos.begin(), traj_req.start_joint_pos.end());
  std::vector<double> target_joint(traj_req.target_joint_pos.begin(), traj_req.target_joint_pos.end());
  const auto retimed = rt::retimeJointPathWithUnifiedConfig(
      {start_joint, target_joint}, 0.01, traj_req.max_velocity, traj_req.max_acceleration, traj_req.blend_radius);
  EXPECT_NEAR(traj_res.total_time, retimed.samples.total_time, 1e-9);
  EXPECT_EQ(traj_res.trajectory_points.size(), retimed.samples.positions.size());
  EXPECT_DOUBLE_EQ(traj_res.trajectory_points.front().pos[0], retimed.samples.positions.front()[0]);
  EXPECT_DOUBLE_EQ(traj_res.trajectory_points.back().pos[0], retimed.samples.positions.back()[0]);
  const auto nrt_logs = data_store_state.queryLogs(8);
  ASSERT_FALSE(nrt_logs.empty());
  EXPECT_NE(nrt_logs.front().content.find("generate_s_trajectory retimer[s_trajectory]"), std::string::npos);
  traj_req.is_cartesian = true;
  traj_req.max_velocity = 0.6;
  traj_req.max_acceleration = 1.5;
  facade.handleGenerateSTrajectory(traj_req, traj_res);
  ASSERT_TRUE(traj_res.success) << traj_res.error_msg;
  EXPECT_GT(traj_res.total_time, 0.0);
  ASSERT_GT(traj_res.trajectory_points.size(), 2u);
  for (int j = 0; j < 6; ++j) {
    EXPECT_NEAR(traj_res.trajectory_points.front().pos[j], traj_req.start_joint_pos[j], 1e-9);
    EXPECT_NEAR(traj_res.trajectory_points.back().pos[j], traj_req.target_joint_pos[j], 1e-3);
  }
  const auto cartesian_logs = data_store_state.queryLogs(16);
  ASSERT_FALSE(cartesian_logs.empty());
  EXPECT_NE(cartesian_logs.front().content.find("generate_s_trajectory retimer[s_trajectory]"), std::string::npos);

  rokae_xmate3_ros2::srv::SetDefaultZone::Request zone_req;
  rokae_xmate3_ros2::srv::SetDefaultZone::Response zone_res;
  zone_req.zone = 250;
  control_facade.handleSetDefaultZone(zone_req, zone_res);
  EXPECT_FALSE(zone_res.success);

  motion_options_state.setZoneValidRange(0, 80);
  zone_req.zone = 81;
  control_facade.handleSetDefaultZone(zone_req, zone_res);
  EXPECT_FALSE(zone_res.success);
  EXPECT_EQ(zone_res.message, "default zone must be within [0, 80] mm");
  zone_req.zone = 60;
  control_facade.handleSetDefaultZone(zone_req, zone_res);
  EXPECT_TRUE(zone_res.success);
}

TEST(ServiceFacadeTest, ControlFacadeRejectsDisconnectedAndInvalidModeRequests) {
  rt::SessionState session_state;
  rt::MotionOptionsState motion_options_state;
  rt::ToolingState tooling_state;
  FakeBackend backend;
  rt::MotionRuntime motion_runtime;
  motion_runtime.reset();
  rt::MotionRequestCoordinator coordinator(motion_options_state, tooling_state, session_state, motion_runtime);
  rt::ControlFacade facade(session_state, motion_options_state, tooling_state, &backend, &motion_runtime, &coordinator);

  rokae_xmate3_ros2::srv::SetOperateMode::Request operate_req;
  rokae_xmate3_ros2::srv::SetOperateMode::Response operate_res;
  operate_req.mode = static_cast<std::uint8_t>(rokae::OperateMode::manual);
  facade.handleSetOperateMode(operate_req, operate_res);
  EXPECT_FALSE(operate_res.success);
  EXPECT_EQ(operate_res.message, "Robot not connected");

  rokae_xmate3_ros2::srv::SetMotionControlMode::Request motion_req;
  rokae_xmate3_ros2::srv::SetMotionControlMode::Response motion_res;
  motion_req.mode = 1;
  facade.handleSetMotionControlMode(motion_req, motion_res);
  EXPECT_FALSE(motion_res.success);
  EXPECT_EQ(motion_res.message, "Robot not connected");

  session_state.connect("127.0.0.1");
  operate_req.mode = 255;
  facade.handleSetOperateMode(operate_req, operate_res);
  EXPECT_FALSE(operate_res.success);
  EXPECT_EQ(operate_res.message, "operate mode must be manual or automatic");

  motion_req.mode = 42;
  facade.handleSetMotionControlMode(motion_req, motion_res);
  EXPECT_FALSE(motion_res.success);
  EXPECT_EQ(motion_res.message,
            "motion control mode must be one of Idle/NrtCommand/RtCommand-compatible values");

  rokae_xmate3_ros2::srv::MoveStart::Request move_start_req;
  rokae_xmate3_ros2::srv::MoveStart::Response move_start_res;
  session_state.setPowerOn(true);
  session_state.setMotionMode(1);
  facade.handleMoveStart(move_start_req, move_start_res);
  EXPECT_FALSE(move_start_res.success);
  EXPECT_EQ(move_start_res.message, "moveStart requires NrtCommand-compatible motion mode");

  session_state.setMotionMode(0);
  session_state.setDragMode(true);
  facade.handleMoveStart(move_start_req, move_start_res);
  EXPECT_FALSE(move_start_res.success);
  EXPECT_EQ(move_start_res.message, "moveStart is not allowed while drag mode is enabled");

  session_state.setDragMode(false);
  facade.handleMoveStart(move_start_req, move_start_res);
  EXPECT_FALSE(move_start_res.success);
  EXPECT_EQ(move_start_res.message, "moveStart rejected because no queued NRT request exists");
}

TEST(ServiceFacadeTest, PathFacadeSaveRequiresRecordedDataAndSupportsRenameOnlyWhenNoPendingRecording) {
  rt::ProgramState program_state;
  rt::ToolingState tooling_state;
  rt::SessionState session_state;
  session_state.connect("127.0.0.1");
  rt::PathFacade facade(session_state,
                        program_state,
                        tooling_state,
                        nullptr,
                        [](std::array<double, 6> &position,
                           std::array<double, 6> &velocity,
                           std::array<double, 6> &torque) {
                          position.fill(0.0);
                          velocity.fill(0.0);
                          torque.fill(0.0);
                        },
                        []() { return 0.01; },
                        [](const std::string &prefix) { return prefix + "_001"; });

  rokae_xmate3_ros2::srv::SaveRecordPath::Request save_req;
  rokae_xmate3_ros2::srv::SaveRecordPath::Response save_res;
  save_req.name = "demo_path";
  facade.handleSaveRecordPath(save_req, save_res);
  EXPECT_FALSE(save_res.success);
  EXPECT_EQ(save_res.message, "no recorded path available to save");

  program_state.startRecordingPath(tooling_state.toolset(), "test_record");
  program_state.recordPathSample(0.0, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  program_state.stopRecordingPath();
  facade.handleSaveRecordPath(save_req, save_res);
  ASSERT_TRUE(save_res.success);
  EXPECT_EQ(save_res.message, "path saved");

  rokae_xmate3_ros2::srv::SaveRecordPath::Request save_as_req;
  rokae_xmate3_ros2::srv::SaveRecordPath::Response save_as_res;
  program_state.startRecordingPath(tooling_state.toolset(), "test_record");
  program_state.recordPathSample(0.1, {0.1, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  program_state.stopRecordingPath();
  save_as_req.name = "ignored_pending_name";
  save_as_req.save_as = "saved_with_save_as";
  facade.handleSaveRecordPath(save_as_req, save_as_res);
  ASSERT_TRUE(save_as_res.success);
  EXPECT_EQ(save_as_res.message, "path saved");

  rt::ReplayPathAsset pending_asset;
  EXPECT_FALSE(program_state.getReplayAsset("ignored_pending_name", pending_asset));
  EXPECT_TRUE(program_state.getReplayAsset("saved_with_save_as", pending_asset));

  rokae_xmate3_ros2::srv::SaveRecordPath::Request rename_req;
  rokae_xmate3_ros2::srv::SaveRecordPath::Response rename_res;
  rename_req.name = "demo_path";
  rename_req.save_as = "renamed_path";
  facade.handleSaveRecordPath(rename_req, rename_res);
  ASSERT_TRUE(rename_res.success);
  EXPECT_EQ(rename_res.message, "path renamed");

  rt::ReplayPathAsset asset;
  EXPECT_FALSE(program_state.getReplayAsset("demo_path", asset));
  EXPECT_TRUE(program_state.getReplayAsset("renamed_path", asset));
}

TEST(ServiceFacadeTest, ControlFacadeRejectsAvoidSingularityOnXMate6Lane) {
  rt::SessionState session_state;
  rt::MotionOptionsState motion_options_state;
  rt::ToolingState tooling_state;
  FakeBackend backend;
  rt::MotionRuntime motion_runtime;
  motion_runtime.reset();
  rt::MotionRequestCoordinator coordinator(motion_options_state, tooling_state, session_state, motion_runtime);
  rt::ControlFacade facade(session_state, motion_options_state, tooling_state, &backend, &motion_runtime, &coordinator);

  rokae_xmate3_ros2::srv::SetAvoidSingularity::Request req;
  rokae_xmate3_ros2::srv::SetAvoidSingularity::Response res;
  req.enable = true;
  facade.handleSetAvoidSingularity(req, res);
  EXPECT_FALSE(res.success);
  EXPECT_EQ(res.message, "Robot not connected");

  session_state.connect("127.0.0.1");
  facade.handleSetAvoidSingularity(req, res);
  EXPECT_FALSE(res.success);
  EXPECT_EQ(res.message, "avoid singularity is not supported on the xMate6 compatibility lane");
  EXPECT_FALSE(motion_options_state.avoidSingularityEnabled());

  rt::DataStoreState data_store_state;
  rt::ProgramState program_state;
  rt::RuntimeDiagnosticsState diagnostics_state;
  gazebo::xMate3Kinematics kinematics;
  rt::QueryFacade query_facade(session_state,
                               motion_options_state,
                               tooling_state,
                               data_store_state,
                               program_state,
                               diagnostics_state,
                               motion_runtime,
                               coordinator,
                               kinematics,
                               [](std::array<double, 6> &position,
                                  std::array<double, 6> &velocity,
                                  std::array<double, 6> &torque) {
                                 position.fill(0.0);
                                 velocity.fill(0.0);
                                 torque.fill(0.0);
                               },
                               []() { return rclcpp::Time(0); },
                               []() { return 0.01; },
                               6);
  rokae_xmate3_ros2::srv::GetAvoidSingularity::Request get_req;
  rokae_xmate3_ros2::srv::GetAvoidSingularity::Response get_res;
  query_facade.handleGetAvoidSingularity(get_req, get_res);
  EXPECT_FALSE(get_res.success);
  EXPECT_FALSE(get_res.enabled);
  EXPECT_EQ(get_res.message, "avoid singularity is not supported on the xMate6 compatibility lane");
}
