#include <gtest/gtest.h>

#include "runtime/runtime_control_bridge.hpp"

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

TEST(RuntimeControlBridgeTest, PowerOffStopsRuntimeAndLocksBrake) {
  rt::RuntimeContext context;
  context.sessionState().setPowerOn(false);
  FakeBackend backend;
  rt::RuntimeControlBridge bridge(context);

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
