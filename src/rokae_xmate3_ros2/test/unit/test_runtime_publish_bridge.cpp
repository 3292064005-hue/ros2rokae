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

TEST(RuntimePublishBridgeTest, BuildsMoveAppendResultFromTerminalStatus) {
  rt::RuntimeContext context;
  rt::RuntimePublishBridge bridge(context);

  rt::RuntimeStatus completed;
  completed.state = rt::ExecutionState::completed;
  completed.terminal_success = true;
  completed.message = "done";
  const auto ok_result = bridge.buildMoveAppendResult("move_ok", completed);
  EXPECT_TRUE(ok_result->success);
  EXPECT_EQ(ok_result->cmd_id, "move_ok");
  EXPECT_EQ(ok_result->message, "done");

  rt::RuntimeStatus relaxed;
  relaxed.state = rt::ExecutionState::completed_relaxed;
  relaxed.terminal_success = false;
  relaxed.message = "completed_with_relaxed_settle";
  const auto relaxed_result = bridge.buildMoveAppendResult("move_relaxed", relaxed);
  EXPECT_FALSE(relaxed_result->success);
  EXPECT_EQ(relaxed_result->cmd_id, "move_relaxed");
  EXPECT_EQ(relaxed_result->message, "completed_with_relaxed_settle");

  rt::RuntimeStatus failed;
  failed.state = rt::ExecutionState::failed;
  failed.terminal_success = false;
  failed.message = "planning failed";
  const auto fail_result = bridge.buildMoveAppendResult("move_fail", failed);
  EXPECT_FALSE(fail_result->success);
  EXPECT_EQ(fail_result->cmd_id, "move_fail");
  EXPECT_EQ(fail_result->message, "planning failed");
}
