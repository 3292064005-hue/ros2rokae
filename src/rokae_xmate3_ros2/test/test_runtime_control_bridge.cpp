#include <gtest/gtest.h>

#include "runtime/runtime_control_bridge.hpp"

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
