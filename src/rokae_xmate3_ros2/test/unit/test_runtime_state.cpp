#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "runtime/runtime_state.hpp"
#include "rokae_xmate3_ros2/runtime/rt_fast_command.hpp"
#include "rokae_xmate3_ros2/runtime/rt_semantic_topics.hpp"

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
  auto warn_log = log;
  warn_log.timestamp = "200";
  warn_log.content = "warn log";
  warn_log.level = 2;
  data.appendLog(warn_log);

  EXPECT_EQ(data.registerValue("r/int0"), "42");
  EXPECT_EQ(data.registerValue("runtime_mode"), "servo");
  EXPECT_TRUE(data.hasCallback("cb_1"));
  EXPECT_TRUE(data.getDI(0, 1));
  EXPECT_FALSE(data.getDO(0, 2));
  EXPECT_DOUBLE_EQ(data.getAI(0, 3), 3.5);
  EXPECT_DOUBLE_EQ(data.getAO(0, 4), 5.0);

  const auto logs = data.queryLogs(8);
  ASSERT_EQ(logs.size(), 2u);
  EXPECT_EQ(logs.front().content, "warn log");
  EXPECT_EQ(logs.back().content, "runtime log");
  const auto filtered_logs = data.queryLogs(8, static_cast<std::uint8_t>(1));
  ASSERT_EQ(filtered_logs.size(), 1u);
  EXPECT_EQ(filtered_logs.front().content, "runtime log");
}


TEST(DataStoreStateTest, RtSemanticSnapshotTracksHotPathMetadata) {
  rokae_xmate3_ros2::runtime::DataStoreState state;
  state.setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlSurface, "web_bridge");
  state.setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlDispatchMode, "joint_position");
  state.setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kCatalogProvenance, "runtime_cache");

  const auto snapshot = state.rtSemanticSnapshot();
  EXPECT_EQ(snapshot.control_surface, "web_bridge");
  EXPECT_EQ(snapshot.dispatch_mode, "joint_position");
  EXPECT_EQ(snapshot.catalog_provenance, "runtime_cache");
}

TEST(DataStoreStateTest, RtFastIngressPrefersHigherPriorityTransport) {
  rt::DataStoreState state;
  state.setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlDispatchMode, "independent_rt");
  state.setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlJointPosition,
                      "seq=3;finished=0;values=0.1,0.2,0.3,0.4,0.5,0.6");

  auto ingress = state.rtFastIngressSnapshot();
  ASSERT_TRUE(ingress.present);
  EXPECT_EQ(ingress.sequence, 3u);
  EXPECT_EQ(ingress.transport, rokae_xmate3_ros2::runtime::RtFastTransport::legacy_custom_data);

  rokae_xmate3_ros2::runtime::RtFastCommandFrame topic_frame;
  topic_frame.sequence = 3;
  topic_frame.kind = rokae_xmate3_ros2::runtime::RtFastCommandKind::joint_position;
  topic_frame.values = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5};
  topic_frame.dispatch_mode = "independent_rt";
  topic_frame.transport = rokae_xmate3_ros2::runtime::RtFastTransport::ros_topic;
  topic_frame.sent_at = std::chrono::steady_clock::now();
  state.ingestRtFastCommand(topic_frame, 2);

  ingress = state.rtFastIngressSnapshot();
  ASSERT_TRUE(ingress.present);
  EXPECT_EQ(ingress.sequence, 3u);
  EXPECT_EQ(ingress.transport, rokae_xmate3_ros2::runtime::RtFastTransport::ros_topic);
  EXPECT_EQ(ingress.queue_depth, 2u);
  EXPECT_DOUBLE_EQ(ingress.values[0], 1.0);

  state.setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlJointPosition,
                      "seq=3;finished=0;values=9,9,9,9,9,9");
  ingress = state.rtFastIngressSnapshot();
  EXPECT_EQ(ingress.transport, rokae_xmate3_ros2::runtime::RtFastTransport::ros_topic);
  EXPECT_DOUBLE_EQ(ingress.values[0], 1.0);

  rokae_xmate3_ros2::runtime::RtFastCommandFrame shm_frame = topic_frame;
  shm_frame.values = {2.0, 2.1, 2.2, 2.3, 2.4, 2.5};
  shm_frame.transport = rokae_xmate3_ros2::runtime::RtFastTransport::shm_ring;
  shm_frame.sent_at = std::chrono::steady_clock::now();
  state.ingestRtFastCommand(shm_frame, 7);

  ingress = state.rtFastIngressSnapshot();
  EXPECT_EQ(ingress.transport, rokae_xmate3_ros2::runtime::RtFastTransport::shm_ring);
  EXPECT_EQ(ingress.queue_depth, 7u);
  EXPECT_DOUBLE_EQ(ingress.values[0], 2.0);

  const auto rt_snapshot = state.rtControlSnapshot();
  EXPECT_TRUE(rt_snapshot.joint_position_command.present);
  EXPECT_TRUE(rt_snapshot.joint_position_command.valid);
  EXPECT_EQ(rt_snapshot.joint_position_command.sequence, 3u);
  EXPECT_DOUBLE_EQ(rt_snapshot.joint_position_command.values[0], 2.0);
}

TEST(DataStoreStateTest, RtFastStopCommandSetsStopRequested) {
  rt::DataStoreState state;
  rokae_xmate3_ros2::runtime::RtFastCommandFrame stop_frame;
  stop_frame.sequence = 11;
  stop_frame.kind = rokae_xmate3_ros2::runtime::RtFastCommandKind::stop;
  stop_frame.dispatch_mode = "independent_rt";
  stop_frame.transport = rokae_xmate3_ros2::runtime::RtFastTransport::ros_topic;
  stop_frame.sent_at = std::chrono::steady_clock::now();
  state.ingestRtFastCommand(stop_frame, 0);

  const auto ingress = state.rtFastIngressSnapshot();
  EXPECT_TRUE(ingress.present);
  EXPECT_EQ(ingress.sequence, 11u);
  EXPECT_EQ(ingress.kind, rokae_xmate3_ros2::runtime::RtFastCommandKind::stop);
  EXPECT_EQ(ingress.transport, rokae_xmate3_ros2::runtime::RtFastTransport::ros_topic);

  const auto rt_snapshot = state.rtControlSnapshot();
  EXPECT_TRUE(rt_snapshot.stop_requested);
}
