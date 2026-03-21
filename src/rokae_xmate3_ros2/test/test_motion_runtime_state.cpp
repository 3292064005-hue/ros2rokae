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
    if (status.state == rt::ExecutionState::completed ||
        status.state == rt::ExecutionState::completed_relaxed) {
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
  EXPECT_TRUE(cached_status.state == rt::ExecutionState::completed ||
              cached_status.state == rt::ExecutionState::completed_relaxed);
  EXPECT_EQ(cached_status.execution_backend, rt::ExecutionBackend::effort);
  if (cached_status.state == rt::ExecutionState::completed_relaxed) {
    EXPECT_FALSE(cached_status.terminal_success);
  } else {
    EXPECT_TRUE(cached_status.terminal_success);
  }

  const auto completed_view = runtime.view();
  EXPECT_FALSE(completed_view.has_request);
  EXPECT_TRUE(completed_view.can_accept_request);
  EXPECT_TRUE(completed_view.terminal);
  EXPECT_TRUE(completed_view.status.state == rt::ExecutionState::completed ||
              completed_view.status.state == rt::ExecutionState::completed_relaxed);
}


class StaticBackend final : public rt::BackendInterface {
 public:
  StaticBackend() { snapshot_.power_on = true; }

  rt::RobotSnapshot readSnapshot() const override { return snapshot_; }
  void applyControl(const rt::ControlCommand &command) override { last_command_ = command; }
  void clearControl() override {}

 private:
  rt::RobotSnapshot snapshot_{};
  rt::ControlCommand last_command_{};
};

TEST(MotionRuntimeStateTest, ActiveSpeedScaleChangesTrajectoryProgressRate) {
  auto make_request = []() {
    rt::MotionRequest request;
    request.request_id = "speed_scale_runtime";
    request.start_joints.assign(6, 0.0);
    request.default_speed = 200;
    request.trajectory_dt = 0.01;

    rt::MotionCommandSpec command;
    command.kind = rt::MotionKind::move_absj;
    command.use_preplanned_trajectory = true;
    command.preplanned_dt = 0.01;
    for (int i = 0; i <= 100; ++i) {
      const double s = static_cast<double>(i) / 100.0;
      command.preplanned_trajectory.push_back({0.20 * s, 0.10 * s, -0.08 * s, 0.0, 0.0, 0.0});
    }
    command.target_joints = command.preplanned_trajectory.back();
    request.commands.push_back(command);
    return request;
  };

  auto wait_for_plan_ready = [](rt::MotionRuntime &runtime, const std::string &request_id) {
    for (int i = 0; i < 200; ++i) {
      const auto status = runtime.status(request_id);
      if (status.state == rt::ExecutionState::queued) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
  };

  rt::MotionRuntime slow_runtime;
  StaticBackend slow_backend;
  std::string message;
  ASSERT_TRUE(slow_runtime.submit(make_request(), message)) << message;
  ASSERT_TRUE(wait_for_plan_ready(slow_runtime, "speed_scale_runtime"));
  slow_runtime.setActiveSpeedScale(0.25);
  for (int i = 0; i < 25; ++i) {
    (void)slow_runtime.tick(slow_backend, 0.01);
  }
  const auto slow_status = slow_runtime.status("speed_scale_runtime");
  EXPECT_EQ(slow_status.state, rt::ExecutionState::executing);

  rt::MotionRuntime fast_runtime;
  StaticBackend fast_backend;
  ASSERT_TRUE(fast_runtime.submit(make_request(), message)) << message;
  ASSERT_TRUE(wait_for_plan_ready(fast_runtime, "speed_scale_runtime"));
  fast_runtime.setActiveSpeedScale(2.0);
  for (int i = 0; i < 60; ++i) {
    (void)fast_runtime.tick(fast_backend, 0.01);
  }
  const auto fast_status = fast_runtime.status("speed_scale_runtime");
  EXPECT_NE(fast_status.state, rt::ExecutionState::executing);
}

TEST(MotionRuntimeStateTest, ActiveSpeedScaleChangesCompletionTickCount) {
  auto make_request = []() {
    rt::MotionRequest request;
    request.request_id = "speed_scale_completion_runtime";
    request.start_joints.assign(6, 0.0);
    request.default_speed = 300.0;
    request.trajectory_dt = 0.01;

    rt::MotionCommandSpec command;
    command.kind = rt::MotionKind::move_absj;
    command.use_preplanned_trajectory = true;
    command.preplanned_dt = 0.01;
    for (int i = 0; i <= 120; ++i) {
      const double s = static_cast<double>(i) / 120.0;
      command.preplanned_trajectory.push_back({0.30 * s, 0.18 * s, -0.12 * s, 0.0, 0.0, 0.0});
    }
    command.target_joints = command.preplanned_trajectory.back();
    request.commands.push_back(command);
    return request;
  };

  auto wait_for_plan_ready = [](rt::MotionRuntime &runtime, const std::string &request_id) {
    for (int i = 0; i < 200; ++i) {
      const auto status = runtime.status(request_id);
      if (status.state == rt::ExecutionState::queued) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
  };

  auto run_to_terminal = [&](double scale) {
    rt::MotionRuntime runtime;
    StaticBackend backend;
    std::string message;
    if (!runtime.submit(make_request(), message)) {
      ADD_FAILURE() << "submit failed: " << message;
      return 2000;
    }
    if (!wait_for_plan_ready(runtime, "speed_scale_completion_runtime")) {
      ADD_FAILURE() << "plan did not reach queued state";
      return 2000;
    }
    runtime.setActiveSpeedScale(scale);

    for (int tick = 1; tick <= 2000; ++tick) {
      const auto status = runtime.tick(backend, 0.01);
      if (status.terminal()) {
        return tick;
      }
    }
    ADD_FAILURE() << "runtime did not reach a terminal state";
    return 2000;
  };

  const int slow_ticks = run_to_terminal(0.25);
  const int nominal_ticks = run_to_terminal(1.0);
  const int fast_ticks = run_to_terminal(2.0);

  EXPECT_GT(slow_ticks, nominal_ticks);
  EXPECT_GT(nominal_ticks, fast_ticks);
  EXPECT_GE(slow_ticks - nominal_ticks, 200);
  EXPECT_GE(nominal_ticks - fast_ticks, 40);
}

class StalledBackend final : public rt::BackendInterface {
 public:
  StalledBackend() { snapshot_.power_on = true; }

  rt::RobotSnapshot readSnapshot() const override { return snapshot_; }
  void applyControl(const rt::ControlCommand &) override {}
  void clearControl() override {}

 private:
  rt::RobotSnapshot snapshot_{};
};

class TrajectoryBackend final : public rt::BackendInterface {
 public:
  TrajectoryBackend() { snapshot_.power_on = true; }

  void attach(rt::MotionRuntime &runtime) { runtime.attachBackend(this); }

  rt::RobotSnapshot readSnapshot() const override {
    if (state_.active && !goal_.points.empty()) {
      elapsed_sec_ = std::min(elapsed_sec_ + tick_sec_, goal_.points.back().time_from_start);
      updateSnapshotFromGoal();
      state_.desired_time_from_start = elapsed_sec_;
      if (elapsed_sec_ + 1e-9 >= goal_.points.back().time_from_start) {
        state_.active = false;
        state_.completed = true;
        state_.succeeded = true;
        state_.message = "trajectory completed";
      }
    }
    return snapshot_;
  }

  void applyControl(const rt::ControlCommand &) override { ++apply_control_count_; }
  void clearControl() override { ++clear_control_count_; }

  [[nodiscard]] bool supportsTrajectoryExecution() const override { return true; }

  bool startTrajectoryExecution(const rt::TrajectoryExecutionGoal &goal, std::string &message) override {
    goal_ = goal;
    elapsed_sec_ = 0.0;
    ++start_count_;
    state_ = rt::TrajectoryExecutionState{};
    state_.request_id = goal.request_id;
    state_.accepted = true;
    state_.active = true;
    state_.message = "trajectory accepted";
    if (!goal_.points.empty()) {
      for (std::size_t i = 0; i < 6 && i < goal_.points.front().position.size(); ++i) {
        snapshot_.joint_position[i] = goal_.points.front().position[i];
      }
    }
    message.clear();
    return !goal.points.empty();
  }

  void cancelTrajectoryExecution(const std::string &reason) override {
    state_.active = false;
    state_.completed = true;
    state_.canceled = true;
    state_.message = reason;
  }

  [[nodiscard]] rt::TrajectoryExecutionState readTrajectoryExecutionState() const override { return state_; }

  int startCount() const { return start_count_; }
  int applyControlCount() const { return apply_control_count_; }
  int clearControlCount() const { return clear_control_count_; }

 private:
  void updateSnapshotFromGoal() const {
    if (goal_.points.empty()) {
      return;
    }
    const auto *previous = &goal_.points.front();
    const auto *next = &goal_.points.back();
    for (const auto &point : goal_.points) {
      if (point.time_from_start + 1e-9 < elapsed_sec_) {
        previous = &point;
        continue;
      }
      next = &point;
      break;
    }
    const double dt = std::max(next->time_from_start - previous->time_from_start, 1e-9);
    const double alpha =
        std::clamp((elapsed_sec_ - previous->time_from_start) / dt, 0.0, 1.0);
    for (std::size_t i = 0; i < 6; ++i) {
      const double p0 = i < previous->position.size() ? previous->position[i] : 0.0;
      const double p1 = i < next->position.size() ? next->position[i] : p0;
      snapshot_.joint_position[i] = p0 + (p1 - p0) * alpha;
      snapshot_.joint_velocity[i] = (p1 - p0) / dt;
    }
  }

  mutable rt::RobotSnapshot snapshot_{};
  mutable rt::TrajectoryExecutionState state_{};
  mutable double elapsed_sec_ = 0.0;
  double tick_sec_ = 0.01;
  rt::TrajectoryExecutionGoal goal_{};
  int start_count_ = 0;
  int apply_control_count_ = 0;
  int clear_control_count_ = 0;
};

TEST(MotionRuntimeStateTest, SettleTimeoutCompletesRelaxedWithoutTerminalSuccess) {
  rt::MotionRuntime runtime;
  StalledBackend backend;

  rt::MotionRequest request;
  request.request_id = "relaxed_completion_runtime";
  request.start_joints.assign(6, 0.0);
  request.default_speed = 50;
  request.trajectory_dt = 0.01;

  rt::MotionCommandSpec command;
  command.kind = rt::MotionKind::move_absj;
  command.use_preplanned_trajectory = true;
  command.preplanned_dt = 0.01;
  command.preplanned_trajectory = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.4, -0.3, 0.2, 0.0, 0.0, 0.0},
  };
  command.target_joints = command.preplanned_trajectory.back();
  request.commands.push_back(command);

  std::string message;
  ASSERT_TRUE(runtime.submit(request, message)) << message;

  for (int i = 0; i < 3000; ++i) {
    const auto status = runtime.tick(backend, 0.01);
    if (status.state == rt::ExecutionState::completed_relaxed) {
      EXPECT_FALSE(status.terminal_success);
      EXPECT_EQ(status.execution_backend, rt::ExecutionBackend::effort);
      return;
    }
  }

  FAIL() << "expected completed_relaxed state";
}

TEST(MotionRuntimeStateTest, UsesTrajectoryBackendWhenAvailable) {
  rt::MotionRuntime runtime;
  TrajectoryBackend backend;
  backend.attach(runtime);

  rt::MotionRequest request;
  request.request_id = "trajectory_backend_runtime";
  request.start_joints.assign(6, 0.0);
  request.default_speed = 200.0;
  request.trajectory_dt = 0.01;

  rt::MotionCommandSpec command;
  command.kind = rt::MotionKind::move_absj;
  command.use_preplanned_trajectory = true;
  command.preplanned_dt = 0.01;
  for (int i = 0; i <= 80; ++i) {
    const double s = static_cast<double>(i) / 80.0;
    command.preplanned_trajectory.push_back({0.18 * s, -0.12 * s, 0.09 * s, 0.0, 0.0, 0.0});
  }
  command.target_joints = command.preplanned_trajectory.back();
  request.commands.push_back(command);

  std::string message;
  ASSERT_TRUE(runtime.submit(request, message)) << message;

  bool reached_terminal = false;
  for (int i = 0; i < 500; ++i) {
    const auto status = runtime.tick(backend, 0.01);
    if (status.terminal()) {
      EXPECT_EQ(status.state, rt::ExecutionState::completed);
      EXPECT_TRUE(status.terminal_success);
      EXPECT_EQ(status.execution_backend, rt::ExecutionBackend::jtc);
      reached_terminal = true;
      break;
    }
  }

  EXPECT_TRUE(reached_terminal);
  EXPECT_GE(backend.startCount(), 1);
  EXPECT_GT(backend.clearControlCount(), 0);
}

TEST(MotionRuntimeStateTest, TrajectoryBackendRetimesOnSpeedScaleChange) {
  rt::MotionRuntime runtime;
  TrajectoryBackend backend;
  backend.attach(runtime);

  rt::MotionRequest request;
  request.request_id = "trajectory_backend_retime_runtime";
  request.start_joints.assign(6, 0.0);
  request.default_speed = 220.0;
  request.trajectory_dt = 0.01;

  rt::MotionCommandSpec command;
  command.kind = rt::MotionKind::move_absj;
  command.use_preplanned_trajectory = true;
  command.preplanned_dt = 0.01;
  for (int i = 0; i <= 120; ++i) {
    const double s = static_cast<double>(i) / 120.0;
    command.preplanned_trajectory.push_back({0.24 * s, 0.16 * s, -0.14 * s, 0.0, 0.0, 0.0});
  }
  command.target_joints = command.preplanned_trajectory.back();
  request.commands.push_back(command);

  std::string message;
  ASSERT_TRUE(runtime.submit(request, message)) << message;

  for (int i = 0; i < 200 && backend.startCount() < 1; ++i) {
    (void)runtime.tick(backend, 0.01);
  }
  ASSERT_GE(backend.startCount(), 1);
  runtime.setActiveSpeedScale(2.0);
  for (int i = 0; i < 200; ++i) {
    const auto status = runtime.tick(backend, 0.01);
    if (status.terminal()) {
      EXPECT_EQ(status.execution_backend, rt::ExecutionBackend::jtc);
      break;
    }
  }

  EXPECT_GE(backend.startCount(), 2);
}
