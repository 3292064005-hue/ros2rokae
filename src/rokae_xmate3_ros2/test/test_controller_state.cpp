#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "runtime/controller_state.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

TEST(ControllerStateTest, BuildsRequestAndOperationContextsFromSingleStateStore) {
  rt::ControllerState state;
  state.connect("192.168.0.1");
  state.setPowerOn(true);
  state.setDragMode(false);
  state.setDefaultSpeed(72);
  state.setDefaultZone(9);
  state.setDefaultConfOpt(true);
  state.setAvoidSingularity(false);
  state.setRtControlMode(3);

  std::array<std::array<double, 2>, 6> limits{{
      {{-1.0, 1.0}},
      {{-2.0, 2.0}},
      {{-3.0, 3.0}},
      {{-4.0, 4.0}},
      {{-5.0, 5.0}},
      {{-6.0, 6.0}},
  }};
  state.setSoftLimit(true, limits);

  const auto request = state.makeMotionRequestContext("req_1", {1, 2, 3, 4, 5, 6}, 0.02);
  EXPECT_EQ(request.request_id, "req_1");
  EXPECT_EQ(request.default_speed, 72);
  EXPECT_EQ(request.default_zone, 9);
  EXPECT_TRUE(request.strict_conf);
  EXPECT_FALSE(request.avoid_singularity);
  EXPECT_TRUE(request.soft_limit_enabled);
  EXPECT_EQ(request.soft_limits[4][0], -5.0);
  EXPECT_DOUBLE_EQ(request.trajectory_dt, 0.02);

  const auto op = state.makeOperationStateContext();
  EXPECT_TRUE(op.connected);
  EXPECT_TRUE(op.power_on);
  EXPECT_FALSE(op.drag_mode);
  EXPECT_EQ(op.rt_control_mode, 3);
}

TEST(ControllerStateTest, StoresPathsRegistersAndLogsThreadSafelyAtApiBoundary) {
  rt::ControllerState state;
  state.startRecordingPath();
  state.recordPathSample({0.0, 0.1, 0.2, 0.3, 0.4, 0.5});
  state.recordPathSample({1.0, 1.1, 1.2, 1.3, 1.4, 1.5});
  state.saveRecordedPath("demo");

  std::vector<std::vector<double>> path;
  ASSERT_TRUE(state.getSavedPath("demo", path));
  ASSERT_EQ(path.size(), 2u);
  EXPECT_DOUBLE_EQ(path.back()[5], 1.5);

  state.setRegister("r/test", "42");
  EXPECT_EQ(state.registerValue("r/test"), "42");

  state.setCustomData("register/runtime", "hello");
  EXPECT_EQ(state.registerValue("runtime"), "hello");

  rokae_xmate3_ros2::msg::LogInfo log;
  log.timestamp = "1";
  log.content = "runtime event";
  log.level = 1;
  state.appendLog(log);
  const auto logs = state.queryLogs(4);
  ASSERT_EQ(logs.size(), 1u);
  EXPECT_EQ(logs.front().content, "runtime event");
}
