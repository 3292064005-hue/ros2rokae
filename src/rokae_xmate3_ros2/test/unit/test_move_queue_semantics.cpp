#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <thread>

#include "runtime/motion_runtime.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

namespace {

class QueueBackend final : public rt::BackendInterface {
 public:
  QueueBackend() { snapshot_.power_on = true; }

  rt::RobotSnapshot readSnapshot() const override { return snapshot_; }
  void applyControl(const rt::ControlCommand &command) override { last_command_ = command; }
  void clearControl() override {
    snapshot_.joint_velocity.fill(0.0);
    snapshot_.joint_torque.fill(0.0);
  }

 private:
  rt::RobotSnapshot snapshot_{};
  rt::ControlCommand last_command_{};
};

rt::MotionRequest makeRequest(const std::string &request_id, double goal) {
  rt::MotionRequest request;
  request.request_id = request_id;
  request.start_joints.assign(6, 0.0);
  request.default_speed = 100.0;
  request.trajectory_dt = 0.01;

  rt::MotionCommandSpec command;
  command.kind = rt::MotionKind::move_absj;
  command.use_preplanned_trajectory = true;
  command.preplanned_dt = 0.01;
  command.preplanned_trajectory = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {goal, 0.0, 0.0, 0.0, 0.0, 0.0},
  };
  command.target_joints = command.preplanned_trajectory.back();
  request.commands.push_back(command);
  return request;
}

bool waitForQueued(rt::MotionRuntime &runtime, const std::string &request_id) {
  for (int i = 0; i < 200; ++i) {
    const auto status = runtime.status(request_id);
    if (status.state == rt::ExecutionState::queued) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

}  // namespace

TEST(MoveQueueSemantics, RejectsConcurrentSubmitUntilResetReopensRuntime) {
  rt::MotionRuntime runtime;
  QueueBackend backend;

  runtime.reset();

  std::string message;
  ASSERT_TRUE(runtime.submit(makeRequest("primary", 0.1), message)) << message;
  ASSERT_TRUE(waitForQueued(runtime, "primary"));
  EXPECT_FALSE(runtime.canAcceptRequest());

  EXPECT_FALSE(runtime.submit(makeRequest("secondary", 0.2), message));
  EXPECT_NE(message.find("busy"), std::string::npos);

  runtime.reset();
  EXPECT_TRUE(runtime.canAcceptRequest());
  EXPECT_TRUE(runtime.submit(makeRequest("secondary", 0.2), message)) << message;
  ASSERT_TRUE(waitForQueued(runtime, "secondary"));

  const auto facts = runtime.contractFacts();
  EXPECT_EQ(facts.active_request_count, 1u);
  EXPECT_EQ(facts.owner, rt::ControlOwner::none);
  EXPECT_EQ(runtime.runtimePhase(), rt::RuntimePhase::planning);
  (void)backend;
}
