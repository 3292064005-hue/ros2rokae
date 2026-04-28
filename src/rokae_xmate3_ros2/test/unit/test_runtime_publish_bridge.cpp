#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include "runtime/runtime_publish_bridge.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

TEST(RuntimePublishBridgeTest, PublisherTickBuildsMessagesLogsAndPathSamplesFromRuntimeContext) {
  rt::RuntimeContext context;
  context.motionRuntime().reset();
  context.sessionState().connect("127.0.0.1");
  context.sessionState().setPowerOn(true);

  rt::RuntimePublishBridge bridge(context);

  rt::MotionRequest request;
  request.request_id = "move_1";
  request.start_joints = {0.0, 0.1, 1.5, 0.0, 1.2, 3.14};
  rt::MotionCommandSpec cmd;
  cmd.kind = rt::MotionKind::move_absj;
  cmd.target_joints = {0.1, 0.2, 1.4, 0.0, 1.1, 3.10};
  request.commands.push_back(cmd);
  std::string submit_message;
  ASSERT_TRUE(context.motionRuntime().submit(request, submit_message)) << submit_message;

  rt::PublisherTickInput tick_input;
  tick_input.stamp = rclcpp::Time(123456789);
  tick_input.frame_id = "base_link";
  const std::vector<std::string> joint_names = {"j1", "j2", "j3", "j4", "j5", "j6"};
  tick_input.joint_names = &joint_names;
  tick_input.position = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  tick_input.velocity = {0.5, 0.4, 0.3, 0.2, 0.1, 0.0};
  tick_input.torque = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5};
  tick_input.min_publish_period_sec = 0.0;
  tick_input.joint_state_publish_period_sec = 0.001;
  tick_input.operation_state_publish_period_sec = 0.001;
  tick_input.diagnostics_publish_period_sec = 0.001;

  const auto first_tick = bridge.buildPublisherTick(tick_input);
  EXPECT_TRUE(first_tick.publish_operation_state);
  EXPECT_TRUE(first_tick.publish_joint_state);
  EXPECT_TRUE(first_tick.publish_runtime_diagnostics);
  EXPECT_FALSE(first_tick.recorded_path_sample);
  EXPECT_EQ(first_tick.operation_state.state, rokae_xmate3_ros2::msg::OperationState::MOVING);
  ASSERT_EQ(first_tick.joint_state.name.size(), 6u);
  EXPECT_EQ(first_tick.joint_state.header.frame_id, "base_link");
  EXPECT_DOUBLE_EQ(first_tick.joint_state.position[3], 0.3);
  EXPECT_DOUBLE_EQ(first_tick.joint_state.effort[5], 1.5);

  tick_input.stamp = rclcpp::Time(123456789 + 100000);
  const auto throttled_tick = bridge.buildPublisherTick(tick_input);
  EXPECT_FALSE(throttled_tick.publish_operation_state);
  EXPECT_FALSE(throttled_tick.publish_joint_state);
  EXPECT_FALSE(throttled_tick.publish_runtime_diagnostics);

  context.programState().startRecordingPath();
  tick_input.stamp = rclcpp::Time(123456789 + 2000000);
  const auto recording_tick = bridge.buildPublisherTick(tick_input);
  EXPECT_TRUE(recording_tick.publish_operation_state);
  EXPECT_TRUE(recording_tick.publish_joint_state);
  EXPECT_TRUE(recording_tick.publish_runtime_diagnostics);
  EXPECT_TRUE(recording_tick.recorded_path_sample);
  context.programState().stopRecordingPath();
  context.programState().saveRecordedPath("capture");

  std::vector<std::vector<double>> saved_path;
  ASSERT_TRUE(context.programState().getSavedPath("capture", saved_path));
  ASSERT_EQ(saved_path.size(), 1u);
  ASSERT_EQ(saved_path.front().size(), 6u);
  EXPECT_DOUBLE_EQ(saved_path.front()[4], 0.4);

  rt::RuntimeStatus status;
  status.request_id = "move_1";
  status.state = rt::ExecutionState::executing;
  status.message = "tracking";
  status.total_segments = 2;
  status.completed_segments = 1;
  status.current_segment_index = 1;
  status.revision = 3;
  bridge.emitRuntimeStatus(status, rclcpp::Time(456789123), rclcpp::get_logger("runtime_publish_bridge_test"));

  const auto logs = context.dataStoreState().queryLogs(10);
  ASSERT_EQ(logs.size(), 1u);
  EXPECT_NE(logs.front().content.find("request=move_1"), std::string::npos);
}

TEST(RuntimePublishBridgeTest, PublisherTickCanProjectAuthorityJointStateOverInputSnapshot) {
  rt::RuntimeContext context;
  context.motionRuntime().reset();
  context.sessionState().connect("127.0.0.1");
  context.sessionState().setPowerOn(true);

  rt::MotionRequest request;
  request.request_id = "move_authority";
  request.start_joints = {0.11, 0.22, 0.33, 0.44, 0.55, 0.66};
  rt::MotionCommandSpec cmd;
  cmd.kind = rt::MotionKind::move_absj;
  cmd.target_joints = {0.12, 0.23, 0.34, 0.45, 0.56, 0.67};
  request.commands.push_back(cmd);
  std::string submit_message;
  ASSERT_TRUE(context.motionRuntime().submit(request, submit_message)) << submit_message;

  rt::RuntimePublishBridge bridge(context);
  const std::vector<std::string> joint_names = {"j1", "j2", "j3", "j4", "j5", "j6"};
  rt::PublisherTickInput tick_input;
  tick_input.stamp = rclcpp::Time(223456789);
  tick_input.frame_id = "base_link";
  tick_input.joint_names = &joint_names;
  tick_input.position = {9.0, 9.0, 9.0, 9.0, 9.0, 9.0};
  tick_input.velocity = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0};
  tick_input.torque = {7.0, 7.0, 7.0, 7.0, 7.0, 7.0};
  tick_input.joint_state_publish_period_sec = 0.001;
  tick_input.operation_state_publish_period_sec = 0.001;
  tick_input.diagnostics_publish_period_sec = 0.001;
  tick_input.prefer_authority_joint_state = true;
  tick_input.allow_input_joint_state_fallback = false;

  const auto tick = bridge.buildPublisherTick(tick_input);
  ASSERT_TRUE(tick.publish_joint_state);
  ASSERT_EQ(tick.joint_state.position.size(), 6u);
  EXPECT_DOUBLE_EQ(tick.joint_state.position[0], 0.11);
  EXPECT_DOUBLE_EQ(tick.joint_state.position[5], 0.66);
  EXPECT_DOUBLE_EQ(tick.joint_state.velocity[0], 0.0);
  EXPECT_DOUBLE_EQ(tick.joint_state.effort[0], 0.0);
}


TEST(RuntimePublishBridgeTest, AuthorityPreferredTickSuppressesPublishWhenNoAuthorityAndNoFallback) {
  rt::RuntimeContext context;
  context.motionRuntime().reset();
  context.sessionState().connect("127.0.0.1");
  context.sessionState().setPowerOn(true);

  rt::RuntimePublishBridge bridge(context);
  const std::vector<std::string> joint_names = {"j1", "j2", "j3", "j4", "j5", "j6"};
  rt::PublisherTickInput tick_input;
  tick_input.stamp = rclcpp::Time(323456789);
  tick_input.frame_id = "base_link";
  tick_input.joint_names = &joint_names;
  tick_input.position = {9.0, 9.0, 9.0, 9.0, 9.0, 9.0};
  tick_input.velocity = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0};
  tick_input.torque = {7.0, 7.0, 7.0, 7.0, 7.0, 7.0};
  tick_input.joint_state_publish_period_sec = 0.001;
  tick_input.operation_state_publish_period_sec = 0.001;
  tick_input.diagnostics_publish_period_sec = 0.001;
  tick_input.prefer_authority_joint_state = true;
  tick_input.allow_input_joint_state_fallback = false;

  const auto tick = bridge.buildPublisherTick(tick_input);
  EXPECT_FALSE(tick.publish_joint_state);
  EXPECT_FALSE(tick.recorded_path_sample);
}

TEST(RuntimePublishBridgeTest, AuthorityPreferredTickReusesLastGoodAuthoritySampleWhenLiveAuthorityDrops) {
  rt::RuntimeContext context;
  context.motionRuntime().reset();
  context.sessionState().connect("127.0.0.1");
  context.sessionState().setPowerOn(true);

  rt::MotionRequest request;
  request.request_id = "move_authority_cache";
  request.start_joints = {0.21, 0.22, 0.23, 0.24, 0.25, 0.26};
  rt::MotionCommandSpec cmd;
  cmd.kind = rt::MotionKind::move_absj;
  cmd.target_joints = {0.31, 0.32, 0.33, 0.34, 0.35, 0.36};
  request.commands.push_back(cmd);
  std::string submit_message;
  ASSERT_TRUE(context.motionRuntime().submit(request, submit_message)) << submit_message;

  rt::RuntimePublishBridge bridge(context);
  const std::vector<std::string> joint_names = {"j1", "j2", "j3", "j4", "j5", "j6"};
  rt::PublisherTickInput tick_input;
  tick_input.stamp = rclcpp::Time(423456789);
  tick_input.frame_id = "base_link";
  tick_input.joint_names = &joint_names;
  tick_input.position = {9.0, 9.0, 9.0, 9.0, 9.0, 9.0};
  tick_input.velocity = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0};
  tick_input.torque = {7.0, 7.0, 7.0, 7.0, 7.0, 7.0};
  tick_input.joint_state_publish_period_sec = 0.001;
  tick_input.operation_state_publish_period_sec = 0.001;
  tick_input.diagnostics_publish_period_sec = 0.001;
  tick_input.prefer_authority_joint_state = true;
  tick_input.allow_input_joint_state_fallback = false;

  const auto live_tick = bridge.buildPublisherTick(tick_input);
  ASSERT_TRUE(live_tick.publish_joint_state);
  EXPECT_DOUBLE_EQ(live_tick.joint_state.position[0], 0.21);
  EXPECT_DOUBLE_EQ(live_tick.joint_state.position[5], 0.26);

  context.motionRuntime().reset();
  tick_input.stamp = rclcpp::Time(423456789 + 2000000);
  const auto cached_tick = bridge.buildPublisherTick(tick_input);
  ASSERT_TRUE(cached_tick.publish_joint_state);
  EXPECT_DOUBLE_EQ(cached_tick.joint_state.position[0], 0.21);
  EXPECT_DOUBLE_EQ(cached_tick.joint_state.position[5], 0.26);
}

TEST(RuntimePublishBridgeTest, BuildsMoveAppendFeedbackFromRuntimeStatus) {
  rt::RuntimeStatus status;
  status.state = rt::ExecutionState::settling;
  status.total_segments = 4;
  status.completed_segments = 2;
  status.current_segment_index = 2;
  status.message = "settling";

  const auto feedback = rt::buildMoveAppendFeedback(status, 1, "executing");
  EXPECT_TRUE(feedback.should_publish);
  EXPECT_DOUBLE_EQ(feedback.progress, 0.5);
  EXPECT_EQ(feedback.current_state, "settling");
  EXPECT_EQ(feedback.current_cmd_index, 2);
}

TEST(RuntimePublishBridgeTest, BuildsMoveAppendQueueAcceptedAndTerminalResults) {
  rt::RuntimeContext context;
  rt::RuntimePublishBridge bridge(context);

  rt::RuntimeStatus completed;
  completed.state = rt::ExecutionState::completed;
  completed.terminal_success = true;
  completed.message = "done";
  const auto queued_result = bridge.buildMoveAppendQueuedResult("move_queue", "queued awaiting moveStart");
  EXPECT_TRUE(queued_result->success);
  EXPECT_EQ(queued_result->cmd_id, "move_queue");
  EXPECT_EQ(queued_result->message, "queued awaiting moveStart");

  const auto ok_result = bridge.buildMoveAppendTerminalResult("move_ok", completed);
  EXPECT_TRUE(ok_result->success);
  EXPECT_EQ(ok_result->cmd_id, "move_ok");
  EXPECT_EQ(ok_result->message, "done");

  rt::RuntimeStatus relaxed;
  relaxed.state = rt::ExecutionState::completed_relaxed;
  relaxed.terminal_success = false;
  relaxed.message = "completed_with_relaxed_settle";
  const auto relaxed_result = bridge.buildMoveAppendTerminalResult("move_relaxed", relaxed);
  EXPECT_FALSE(relaxed_result->success);
  EXPECT_EQ(relaxed_result->cmd_id, "move_relaxed");
  EXPECT_EQ(relaxed_result->message, "completed_with_relaxed_settle");

  rt::RuntimeStatus failed;
  failed.state = rt::ExecutionState::failed;
  failed.terminal_success = false;
  failed.message = "planning failed";
  const auto fail_result = bridge.buildMoveAppendTerminalResult("move_fail", failed);
  EXPECT_FALSE(fail_result->success);
  EXPECT_EQ(fail_result->cmd_id, "move_fail");
  EXPECT_EQ(fail_result->message, "planning failed");
}
