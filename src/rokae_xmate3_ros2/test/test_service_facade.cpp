#include <gtest/gtest.h>

#include <array>

#include "runtime/runtime_state.hpp"
#include "runtime/motion_runtime.hpp"
#include "runtime/service_facade.hpp"

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
  rt::MotionRuntime motion_runtime;
  rt::MotionRequestCoordinator coordinator(motion_options_state, motion_runtime);
  FakeBackend backend;
  rt::ControlFacade facade(session_state, motion_options_state, &backend, &motion_runtime, &coordinator);

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
  ASSERT_TRUE(drag_res.success);
  EXPECT_TRUE(session_state.dragMode());
  EXPECT_EQ(backend.clear_count, 1);

  rokae_xmate3_ros2::srv::DisableDrag::Request undrag_req;
  rokae_xmate3_ros2::srv::DisableDrag::Response undrag_res;
  facade.handleDisableDrag(undrag_req, undrag_res);
  ASSERT_TRUE(undrag_res.success);
  EXPECT_FALSE(session_state.dragMode());
  EXPECT_EQ(backend.clear_count, 2);

  rokae_xmate3_ros2::srv::Stop::Request stop_req;
  rokae_xmate3_ros2::srv::Stop::Response stop_res;
  facade.handleStop(stop_req, stop_res);
  ASSERT_TRUE(stop_res.success);
  EXPECT_EQ(backend.clear_count, 3);
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

TEST(ServiceFacadeTest, PathFacadeSubmitsReplayRequestsThroughCoordinator) {
  rt::ProgramState program_state;
  rt::MotionOptionsState motion_options_state;
  rt::SessionState session_state;
  session_state.setPowerOn(true);
  rt::MotionRuntime motion_runtime;
  rt::MotionRequestCoordinator coordinator(motion_options_state, motion_runtime);

  auto joint_state_fetcher = [](std::array<double, 6> &position,
                                std::array<double, 6> &velocity,
                                std::array<double, 6> &torque) {
    position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    velocity.fill(0.0);
    torque.fill(0.0);
  };
  auto dt_provider = []() { return 0.01; };
  auto request_id_generator = [](const std::string &prefix) { return prefix + "_001"; };

  rt::PathFacade facade(program_state, &coordinator, joint_state_fetcher, dt_provider, request_id_generator);

  program_state.startRecordingPath();
  program_state.recordPathSample({0.0, 0.1, 0.2, 0.3, 0.4, 0.5});
  program_state.recordPathSample({0.2, 0.3, 0.4, 0.5, 0.6, 0.7});
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
