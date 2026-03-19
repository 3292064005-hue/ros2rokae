#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "runtime/motion_runtime.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

class FakeBackend final : public rt::BackendInterface {
 public:
  explicit FakeBackend(const std::array<double, 6> &target) : target_(target) {
    snapshot_.power_on = true;
  }

  rt::RobotSnapshot readSnapshot() const override { return snapshot_; }

  void applyControl(const rt::ControlCommand &command) override {
    const auto previous_position = snapshot_.joint_position;
    for (size_t i = 0; i < 6; ++i) {
      const double error = target_[i] - snapshot_.joint_position[i];
      const double step = std::clamp(error * 0.30, -0.006, 0.006);
      snapshot_.joint_position[i] += step;
      snapshot_.joint_velocity[i] = step / 0.01;
      snapshot_.joint_torque[i] = command.has_effort ? command.effort[i] : 0.0;
      if (std::fabs(target_[i] - snapshot_.joint_position[i]) < 1e-4) {
        snapshot_.joint_position[i] = target_[i];
        snapshot_.joint_velocity[i] = 0.0;
      }
    }
    last_position_ = previous_position;
  }

  void clearControl() override {
    snapshot_.joint_velocity.fill(0.0);
    snapshot_.joint_torque.fill(0.0);
  }

 private:
  rt::RobotSnapshot snapshot_{};
  std::array<double, 6> target_{};
  std::array<double, 6> last_position_{};
};

TEST(MotionRuntimeStateTest, TransitionsPlanningToCompletedForPreplannedMotion) {
  rt::MotionRuntime runtime;
  const std::array<double, 6> target = {0.015, -0.012, 0.010, 0.0, 0.0, 0.0};
  FakeBackend backend(target);

  rt::MotionRequest request;
  request.request_id = "runtime_smoke";
  request.start_joints.assign(6, 0.0);
  request.default_speed = 20;
  request.trajectory_dt = 0.01;

  rt::MotionCommandSpec command;
  command.kind = rt::MotionKind::move_absj;
  command.use_preplanned_trajectory = true;
  command.preplanned_dt = 0.01;
  command.preplanned_trajectory = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {target[0], target[1], target[2], target[3], target[4], target[5]},
  };
  command.target_joints.assign(target.begin(), target.end());
  request.commands.push_back(command);

  std::string message;
  ASSERT_TRUE(runtime.submit(request, message)) << message;

  const auto initial_view = runtime.view();
  EXPECT_TRUE(initial_view.has_request);
  EXPECT_TRUE(initial_view.active_motion);
  EXPECT_FALSE(initial_view.can_accept_request);
  EXPECT_EQ(initial_view.status.state, rt::ExecutionState::planning);

  const auto first_update = runtime.waitForUpdate(request.request_id, 0, std::chrono::milliseconds(200));
  EXPECT_GT(first_update.revision, 0u);

  bool saw_planning = runtime.status(request.request_id).state == rt::ExecutionState::planning ||
                      first_update.state == rt::ExecutionState::planning;
  bool saw_queued = false;
  bool saw_executing = false;
  bool saw_settling = false;
  bool saw_completed = false;

  for (int i = 0; i < 200; ++i) {
    const auto status = runtime.status(request.request_id);
    if (status.state == rt::ExecutionState::queued) {
      saw_queued = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  for (int i = 0; i < 2000; ++i) {
    const auto status = runtime.tick(backend, 0.01);
    saw_planning = saw_planning || status.state == rt::ExecutionState::planning;
    saw_queued = saw_queued || status.state == rt::ExecutionState::queued;
    saw_executing = saw_executing || status.state == rt::ExecutionState::executing;
    saw_settling = saw_settling || status.state == rt::ExecutionState::settling;
    if (status.state == rt::ExecutionState::completed) {
      saw_completed = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  EXPECT_TRUE(saw_planning);
  EXPECT_TRUE(saw_queued);
  EXPECT_TRUE(saw_executing);
  EXPECT_TRUE(saw_settling);
  EXPECT_TRUE(saw_completed);

  const auto cached_status = runtime.status(request.request_id);
  EXPECT_EQ(cached_status.state, rt::ExecutionState::completed);
  EXPECT_TRUE(cached_status.terminal_success);

  const auto completed_view = runtime.view();
  EXPECT_FALSE(completed_view.has_request);
  EXPECT_TRUE(completed_view.can_accept_request);
  EXPECT_TRUE(completed_view.terminal);
  EXPECT_EQ(completed_view.status.state, rt::ExecutionState::completed);
}
