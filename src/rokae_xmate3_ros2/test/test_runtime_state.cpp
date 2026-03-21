#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

#include "runtime/runtime_state.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

TEST(RuntimeStateTest, SessionAndMotionOptionsExposeStableSnapshots) {
  rt::SessionState session;
  session.connect("192.168.1.10");
  session.setPowerOn(true);
  session.setDragMode(true);
  session.setRtControlMode(2);

  rt::MotionOptionsState options;
  options.setDefaultSpeed(66);
  options.setDefaultZone(7);
  options.setDefaultConfOpt(true);
  options.setAvoidSingularity(false);

  std::array<std::array<double, 2>, 6> limits{{
      {{-1.0, 1.0}},
      {{-2.0, 2.0}},
      {{-3.0, 3.0}},
      {{-4.0, 4.0}},
      {{-5.0, 5.0}},
      {{-6.0, 6.0}},
  }};
  options.setSoftLimit(true, limits);

  const auto request = options.makeMotionRequestContext("req_state", {0, 1, 2, 3, 4, 5}, 0.02);
  EXPECT_EQ(request.request_id, "req_state");
  EXPECT_DOUBLE_EQ(request.default_speed, 66.0);
  EXPECT_EQ(request.default_zone, 7);
  EXPECT_TRUE(request.strict_conf);
  EXPECT_FALSE(request.avoid_singularity);
  EXPECT_TRUE(request.soft_limit_enabled);
  EXPECT_DOUBLE_EQ(request.soft_limits[5][1], 6.0);

  const auto operation = session.makeOperationStateContext(false);
  EXPECT_TRUE(operation.connected);
  EXPECT_TRUE(operation.power_on);
  EXPECT_TRUE(operation.drag_mode);
  EXPECT_EQ(operation.rt_control_mode, 2);
}

TEST(RuntimeStateTest, ProgramAndDataStoresRetainPathRegisterAndIoData) {
  rt::ProgramState program;
  program.loadRlProject("/tmp/demo.rlproj", "demo");
  program.setRlProjectRunning(true, 3);
  EXPECT_TRUE(program.rlProjectLoaded());
  EXPECT_TRUE(program.rlProjectRunning());
  EXPECT_EQ(program.loadedRlProjectName(), "demo");
  EXPECT_EQ(program.rlCurrentEpisode(), 3);

  program.startRecordingPath();
  program.recordPathSample({0.0, 0.1, 0.2, 0.3, 0.4, 0.5});
  program.recordPathSample({1.0, 1.1, 1.2, 1.3, 1.4, 1.5});
  program.stopRecordingPath();
  program.saveRecordedPath("demo_path");

  std::vector<std::vector<double>> saved_path;
  ASSERT_TRUE(program.getSavedPath("demo_path", saved_path));
  ASSERT_EQ(saved_path.size(), 2u);
  EXPECT_DOUBLE_EQ(saved_path.back()[4], 1.4);

  rt::DataStoreState data;
  data.setRegister("r/int0", "42");
  data.setCustomData("register/runtime_mode", "servo");
  data.registerCallback("cb_1", "joint_state");
  data.setDI(0, 1, true);
  data.setDO(0, 2, false);
  data.setAI(0, 3, 3.5);
  data.setAO(0, 4, 5.0);

  rokae_xmate3_ros2::msg::LogInfo log;
  log.timestamp = "100";
  log.content = "runtime log";
  log.level = 1;
  data.appendLog(log);

  EXPECT_EQ(data.registerValue("r/int0"), "42");
  EXPECT_EQ(data.registerValue("runtime_mode"), "servo");
  EXPECT_TRUE(data.hasCallback("cb_1"));
  EXPECT_TRUE(data.getDI(0, 1));
  EXPECT_FALSE(data.getDO(0, 2));
  EXPECT_DOUBLE_EQ(data.getAI(0, 3), 3.5);
  EXPECT_DOUBLE_EQ(data.getAO(0, 4), 5.0);

  const auto logs = data.queryLogs(8);
  ASSERT_EQ(logs.size(), 1u);
  EXPECT_EQ(logs.front().content, "runtime log");
}
