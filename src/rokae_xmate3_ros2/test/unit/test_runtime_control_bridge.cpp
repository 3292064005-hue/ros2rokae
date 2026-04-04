#include <gtest/gtest.h>

#include <cmath>
#include <chrono>
#include <thread>

#include "runtime/runtime_control_bridge.hpp"
#include "runtime/session_state.hpp"
#include "rokae_xmate3_ros2/runtime/rt_semantic_topics.hpp"
#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

namespace {

class FakeBackend final : public rt::BackendInterface {
 public:
  rt::RobotSnapshot readSnapshot() const override { return snapshot; }

  void setControlOwner(rt::ControlOwner owner) override { control_owner = owner; }
  [[nodiscard]] rt::ControlOwner controlOwner() const override { return control_owner; }

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
  rt::ControlOwner control_owner = rt::ControlOwner::none;
};

}  // namespace


TEST(RuntimeControlBridgeTest, DirectRtJointCommandBypassesQueuedRuntime) {
  rt::RuntimeContext context;
  context.sessionState().setPowerOn(true);
  context.sessionState().setMotionMode(rt::kSessionMotionModeRt);
  context.sessionState().setRtControlMode(static_cast<int>(rokae::RtControllerMode::jointPosition));
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlSurface, "preferred");
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlDispatchMode, "independent_rt");
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlJointPosition,
                                         "seq=1;finished=0;values=0.20,0.10,0.05,0.00,0.00,0.00");
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kConfigRtNetworkTolerance, "20");
  FakeBackend backend;
  backend.snapshot.power_on = true;
  rt::RuntimeControlBridge bridge(context);
  context.motionRuntime().reset();

  rt::MotionRequest request;
  request.request_id = "queued_should_not_run";
  request.start_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  rt::MotionCommandSpec cmd;
  cmd.kind = rt::MotionKind::move_absj;
  cmd.target_joints = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
  request.commands.push_back(cmd);
  std::string message;
  ASSERT_TRUE(context.motionRuntime().submit(request, message)) << message;

  const auto result = bridge.tick(backend, backend.readSnapshot(), 0.01);

  EXPECT_FALSE(result.runtime_ticked);
  EXPECT_GT(backend.apply_count, 0);
  EXPECT_TRUE(backend.last_command.has_effort);
  EXPECT_EQ(backend.controlOwner(), rt::ControlOwner::effort);
  const auto diag = context.diagnosticsState().snapshot();
  EXPECT_EQ(diag.rt_dispatch_mode, "independent_rt");
  EXPECT_EQ(diag.last_api_surface, "preferred");
}

TEST(RuntimeControlBridgeTest, DirectRtCartesianCommandRejectsConfiguredSafetyBoxViolation) {
  rt::RuntimeContext context;
  context.sessionState().setPowerOn(true);
  context.sessionState().setMotionMode(rt::kSessionMotionModeRt);
  context.sessionState().setRtControlMode(static_cast<int>(rokae::RtControllerMode::cartesianPosition));
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlSurface, "preferred");
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlDispatchMode, "independent_rt");
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlCartesianPosition,
                                         "seq=7;finished=0;values=2.00,0.00,0.00,0.00,0.00,0.00");
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kConfigCartesianLimit,
                                         "lengths=0.40,0.40,0.40;frame=1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1");
  FakeBackend backend;
  backend.snapshot.power_on = true;
  rt::RuntimeControlBridge bridge(context);

  const auto result = bridge.tick(backend, backend.readSnapshot(), 0.01);

  EXPECT_TRUE(result.control_cleared);
  EXPECT_EQ(backend.clear_count, 1);
  EXPECT_EQ(backend.apply_count, 0);
  const auto diag = context.diagnosticsState().snapshot();
  EXPECT_EQ(diag.last_result_source, "runtime_cartesian_limit_rejected");
  EXPECT_EQ(diag.rt_dispatch_mode, "direct_rt_rejected");
}

TEST(RuntimeControlBridgeTest, DirectRtLoadConfigurationAffectsCompensationEffort) {
  rt::RuntimeContext context;
  context.sessionState().setPowerOn(true);
  context.sessionState().setMotionMode(rt::kSessionMotionModeRt);
  context.sessionState().setRtControlMode(static_cast<int>(rokae::RtControllerMode::jointPosition));
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlSurface, "preferred");
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlDispatchMode, "independent_rt");
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlJointPosition,
                                         "seq=9;finished=0;values=0.40,0.30,0.20,0.10,0.05,0.02");
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kConfigLoad,
                                         "mass=4.5;cog=0.00,0.00,0.18;inertia=0.01,0.01,0.01");
  FakeBackend backend;
  backend.snapshot.power_on = true;
  backend.snapshot.joint_position = {0.40, 0.30, 0.20, 0.10, 0.05, 0.02};
  rt::RuntimeControlBridge bridge(context);

  const auto result = bridge.tick(backend, backend.readSnapshot(), 0.01);

  EXPECT_FALSE(result.control_cleared);
  ASSERT_EQ(backend.apply_count, 1);
  double effort_norm = 0.0;
  for (double value : backend.last_command.effort) {
    effort_norm += std::fabs(value);
  }
  EXPECT_GT(effort_norm, 0.1);
  EXPECT_EQ(backend.controlOwner(), rt::ControlOwner::effort);
}

TEST(RuntimeControlBridgeTest, DirectRtIsBlockedWhenRciClientCompatibilityIsRequested) {
  rt::RuntimeContext context;
  context.sessionState().setPowerOn(true);
  context.sessionState().setMotionMode(rt::kSessionMotionModeRt);
  context.sessionState().setRtControlMode(static_cast<int>(rokae::RtControllerMode::jointPosition));
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlSurface, "preferred");
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlDispatchMode, "independent_rt");
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kConfigUseRciClient, "1");
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlJointPosition,
                                         "seq=12;finished=0;values=0.10,0.05,0.02,0.00,0.00,0.00");
  FakeBackend backend;
  backend.snapshot.power_on = true;
  rt::RuntimeControlBridge bridge(context);

  const auto result = bridge.tick(backend, backend.readSnapshot(), 0.01);

  EXPECT_TRUE(result.control_cleared);
  EXPECT_EQ(backend.apply_count, 0);
  EXPECT_EQ(backend.clear_count, 1);
  const auto diag = context.diagnosticsState().snapshot();
  EXPECT_EQ(diag.last_result_source, "runtime_rci_client_enabled");
  EXPECT_EQ(diag.rt_dispatch_mode, "direct_rt_blocked_by_rci_client");
}

TEST(RuntimeControlBridgeTest, DirectRtTimeoutUsesConfiguredNetworkToleranceWindow) {
  rt::RuntimeContext context;
  context.sessionState().setPowerOn(true);
  context.sessionState().setMotionMode(rt::kSessionMotionModeRt);
  context.sessionState().setRtControlMode(static_cast<int>(rokae::RtControllerMode::jointPosition));
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlSurface, "preferred");
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlDispatchMode, "independent_rt");
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kConfigRtNetworkTolerance, "0");
  context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlJointPosition,
                                         "seq=15;finished=0;values=0.05,0.05,0.05,0.05,0.05,0.05");
  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  FakeBackend backend;
  backend.snapshot.power_on = true;
  rt::RuntimeControlBridge bridge(context);

  const auto result = bridge.tick(backend, backend.readSnapshot(), 0.01);

  EXPECT_TRUE(result.control_cleared);
  EXPECT_EQ(backend.apply_count, 0);
  const auto diag = context.diagnosticsState().snapshot();
  EXPECT_EQ(diag.last_result_source, "runtime_direct_command_timeout");
  EXPECT_EQ(diag.rt_dispatch_mode, "direct_rt_starved");
}

TEST(RuntimeControlBridgeTest, DirectRtCartesianImpedanceConsumesForceControlFrameTransform) {
  gazebo::xMate3Kinematics kinematics;
  std::vector<double> seed_joints;
  std::vector<double> flange_pose;
  const std::vector<std::vector<double>> ik_probe_candidates{
      {0.2, -0.5, 0.4, 0.1, 0.2, -0.1},
      {0.1, -0.2, 0.3, -0.1, 0.2, 0.1},
      {-0.2, -0.3, 0.5, 0.2, -0.1, 0.2},
      {0.3, -0.7, 0.6, -0.2, 0.4, -0.3},
  };
  for (const auto &candidate : ik_probe_candidates) {
    auto pose = kinematics.forwardKinematicsRPY(candidate);
    auto ik = kinematics.inverseKinematics(pose, candidate);
    if (ik.size() == 6) {
      seed_joints = candidate;
      flange_pose = std::move(pose);
      break;
    }
  }
  if (seed_joints.empty() || flange_pose.size() != 6) {
    GTEST_SKIP() << "IK backend could not find a stable seeded pose for cartesian impedance test";
  }

  rt::RuntimeContext base_context;
  base_context.sessionState().setPowerOn(true);
  base_context.sessionState().setMotionMode(rt::kSessionMotionModeRt);
  base_context.sessionState().setRtControlMode(static_cast<int>(rokae::RtControllerMode::cartesianImpedance));
  base_context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlSurface, "preferred");
  base_context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlDispatchMode, "independent_rt");
  base_context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kConfigCartesianDesiredWrench,
                                              "5.0,0.0,0.0,0.0,0.0,0.0");

  rt::RuntimeContext rotated_context;
  rotated_context.sessionState().setPowerOn(true);
  rotated_context.sessionState().setMotionMode(rt::kSessionMotionModeRt);
  rotated_context.sessionState().setRtControlMode(static_cast<int>(rokae::RtControllerMode::cartesianImpedance));
  rotated_context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlSurface, "preferred");
  rotated_context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlDispatchMode, "independent_rt");
  rotated_context.dataStoreState().setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kConfigCartesianDesiredWrench,
                                                 "5.0,0.0,0.0,0.0,0.0,0.0");
  rotated_context.dataStoreState().setCustomData(
      rokae_xmate3_ros2::runtime::rt_topics::kConfigForceControlFrame,
      "type=" + std::to_string(static_cast<int>(rokae::FrameType::world)) +
          ";values=0,-1,0,0,1,0,0,0,0,0,1,0,0,0,0,1");
  rokae_xmate3_ros2::runtime::RtFastCommandFrame cartesian_fast_frame;
  cartesian_fast_frame.sequence = 21;
  cartesian_fast_frame.kind = rokae_xmate3_ros2::runtime::RtFastCommandKind::cartesian_position;
  cartesian_fast_frame.transport = rokae_xmate3_ros2::runtime::RtFastTransport::ros_topic;
  cartesian_fast_frame.dispatch_mode = "independent_rt";
  cartesian_fast_frame.sent_at = std::chrono::steady_clock::now();
  std::copy_n(flange_pose.begin(), 6, cartesian_fast_frame.values.begin());
  base_context.dataStoreState().ingestRtFastCommand(cartesian_fast_frame, 0);
  rotated_context.dataStoreState().ingestRtFastCommand(cartesian_fast_frame, 0);

  FakeBackend base_backend;
  base_backend.snapshot.power_on = true;
  base_backend.snapshot.joint_position = {seed_joints[0], seed_joints[1], seed_joints[2],
                                          seed_joints[3], seed_joints[4], seed_joints[5]};
  FakeBackend rotated_backend = base_backend;

  rt::RuntimeControlBridge base_bridge(base_context);
  rt::RuntimeControlBridge rotated_bridge(rotated_context);

  const auto base_result = base_bridge.tick(base_backend, base_backend.readSnapshot(), 0.01);
  const auto rotated_result = rotated_bridge.tick(rotated_backend, rotated_backend.readSnapshot(), 0.01);
  const auto base_diag = base_context.diagnosticsState().snapshot();
  const auto rotated_diag = rotated_context.diagnosticsState().snapshot();

  EXPECT_FALSE(base_result.control_cleared)
      << base_diag.last_result_source << " / " << base_diag.rt_dispatch_mode;
  EXPECT_FALSE(rotated_result.control_cleared)
      << rotated_diag.last_result_source << " / " << rotated_diag.rt_dispatch_mode;
  ASSERT_EQ(base_backend.apply_count, 1);
  ASSERT_EQ(rotated_backend.apply_count, 1);

  bool any_effort_differs = false;
  for (std::size_t i = 0; i < 6; ++i) {
    if (std::fabs(base_backend.last_command.effort[i] - rotated_backend.last_command.effort[i]) > 1e-6) {
      any_effort_differs = true;
      break;
    }
  }
  EXPECT_TRUE(any_effort_differs);
}

TEST(RuntimeControlBridgeTest, PowerOffStopsRuntimeAndLocksBrake) {
  rt::RuntimeContext context;
  context.sessionState().setPowerOn(false);
  FakeBackend backend;
  rt::RuntimeControlBridge bridge(context);
  context.motionRuntime().reset();

  const auto result = bridge.tick(backend, backend.readSnapshot(), 0.01);

  EXPECT_TRUE(backend.brake_locked);
  EXPECT_GE(backend.brake_set_count, 1);
  EXPECT_TRUE(result.brake_locked);
  EXPECT_TRUE(result.status.state == rt::ExecutionState::idle ||
              result.status.state == rt::ExecutionState::stopped);
}

TEST(RuntimeControlBridgeTest, DragModeClearsControlWithoutTickingRuntime) {
  rt::RuntimeContext context;
  context.sessionState().setPowerOn(true);
  context.sessionState().setDragMode(true);
  FakeBackend backend;
  rt::RuntimeControlBridge bridge(context);
  context.motionRuntime().reset();

  const auto result = bridge.tick(backend, backend.readSnapshot(), 0.01);

  EXPECT_EQ(backend.clear_count, 1);
  EXPECT_TRUE(result.control_cleared);
  EXPECT_FALSE(result.runtime_ticked);
  EXPECT_EQ(backend.controlOwner(), rt::ControlOwner::none);
}

TEST(RuntimeControlBridgeTest, NormalTickAdvancesRuntime) {
  rt::RuntimeContext context;
  context.sessionState().setPowerOn(true);
  FakeBackend backend;
  rt::RuntimeControlBridge bridge(context);
  context.motionRuntime().reset();

  rt::MotionRequest request;
  request.request_id = "move_1";
  request.start_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  rt::MotionCommandSpec cmd;
  cmd.kind = rt::MotionKind::move_absj;
  cmd.target_joints = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  request.commands.push_back(cmd);
  std::string message;
  ASSERT_TRUE(context.motionRuntime().submit(request, message)) << message;

  const auto result = bridge.tick(backend, backend.readSnapshot(), 0.01);

  EXPECT_TRUE(result.runtime_ticked);
  EXPECT_TRUE(result.status.state == rt::ExecutionState::planning ||
              result.status.state == rt::ExecutionState::queued ||
              result.status.state == rt::ExecutionState::executing);
}

TEST(RuntimeControlBridgeTest, SoftLimitWatchdogStopsActiveRuntime) {
  rt::RuntimeContext context;
  context.sessionState().setPowerOn(true);
  FakeBackend backend;
  backend.snapshot.power_on = true;
  backend.snapshot.joint_position[0] = 0.6;
  std::array<std::array<double, 2>, 6> soft_limits{{
      {{-0.2, 0.2}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
  }};
  context.motionOptionsState().setSoftLimit(true, soft_limits);
  rt::RuntimeControlBridge bridge(context);
  context.motionRuntime().reset();

  rt::MotionRequest request;
  request.request_id = "soft_limit_trip";
  request.start_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  rt::MotionCommandSpec cmd;
  cmd.kind = rt::MotionKind::move_absj;
  cmd.target_joints = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  request.commands.push_back(cmd);
  std::string message;
  ASSERT_TRUE(context.motionRuntime().submit(request, message)) << message;

  const auto result = bridge.tick(backend, backend.readSnapshot(), 0.01);

  EXPECT_TRUE(result.control_cleared);
  EXPECT_EQ(result.status.state, rt::ExecutionState::stopped);
  EXPECT_EQ(backend.controlOwner(), rt::ControlOwner::none);
}

TEST(RuntimeControlBridgeTest, CollisionDetectionCanTriggerRetreatPulse) {
  rt::RuntimeContext context;
  context.sessionState().setPowerOn(true);
  context.sessionState().setCollisionDetectionEnabled(true);
  context.sessionState().setCollisionDetectionConfig({{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}}, 1, 0.8);
  FakeBackend backend;
  backend.snapshot.power_on = true;
  backend.snapshot.joint_velocity[1] = 0.3;
  backend.snapshot.joint_torque[1] = 200.0;
  rt::RuntimeControlBridge bridge(context);
  context.motionRuntime().reset();

  rt::MotionRequest request;
  request.request_id = "collision_trip";
  request.start_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  rt::MotionCommandSpec cmd;
  cmd.kind = rt::MotionKind::move_absj;
  cmd.target_joints = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  request.commands.push_back(cmd);
  std::string message;
  ASSERT_TRUE(context.motionRuntime().submit(request, message)) << message;

  const auto result = bridge.tick(backend, backend.readSnapshot(), 0.01);

  EXPECT_EQ(result.status.state, rt::ExecutionState::stopped);
  EXPECT_GT(backend.apply_count, 0);
  EXPECT_EQ(backend.controlOwner(), rt::ControlOwner::none);
}

TEST(RuntimeControlBridgeTest, CollisionStop2SlowsActiveRuntimeBeforeStopping) {
  rt::RuntimeContext context;
  context.sessionState().setPowerOn(true);
  context.sessionState().setCollisionDetectionEnabled(true);
  context.sessionState().setCollisionDetectionConfig({{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}}, 2, 0.0);
  FakeBackend backend;
  backend.snapshot.power_on = true;
  backend.snapshot.joint_velocity[2] = 0.25;
  backend.snapshot.joint_torque[2] = 220.0;
  rt::RuntimeControlBridge bridge(context);
  context.motionRuntime().reset();

  rt::MotionRequest request;
  request.request_id = "collision_stop2_trip";
  request.start_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  rt::MotionCommandSpec cmd;
  cmd.kind = rt::MotionKind::move_absj;
  cmd.target_joints = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  request.commands.push_back(cmd);
  std::string message;
  ASSERT_TRUE(context.motionRuntime().submit(request, message)) << message;
  context.motionRuntime().setActiveSpeedScale(1.0);

  const auto result = bridge.tick(backend, backend.readSnapshot(), 0.01);

  EXPECT_EQ(result.status.state, rt::ExecutionState::stopped);
  EXPECT_DOUBLE_EQ(context.motionRuntime().activeSpeedScale(), 0.25);
  EXPECT_EQ(backend.controlOwner(), rt::ControlOwner::none);
}
