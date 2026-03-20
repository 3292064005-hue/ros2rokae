#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include "runtime/runtime_publish_bridge.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

TEST(RuntimePublishBridgeTest, BuildsMessagesLogsAndPathSamplesFromRuntimeContext) {
  rt::RuntimeContext context;
  context.sessionState().connect("127.0.0.1");
  context.sessionState().setPowerOn(true);

  bool recording = false;
  std::vector<std::array<double, 6>> samples;
  rt::RuntimePublishBridge bridge(
      context,
      [&recording]() { return recording; },
      [&samples](const std::array<double, 6> &joint_position) { samples.push_back(joint_position); });

  rt::MotionRequest request;
  request.request_id = "move_1";
  request.start_joints = {0.0, 0.1, 1.5, 0.0, 1.2, 3.14};
  rt::MotionCommandSpec cmd;
  cmd.kind = rt::MotionKind::move_absj;
  cmd.target_joints = {0.1, 0.2, 1.4, 0.0, 1.1, 3.10};
  request.commands.push_back(cmd);
  std::string submit_message;
  ASSERT_TRUE(context.motionRuntime().submit(request, submit_message)) << submit_message;

  const auto operation_state = bridge.buildOperationStateMessage();
  EXPECT_EQ(operation_state.state, rokae_xmate3_ros2::msg::OperationState::MOVING);

  const auto joint_message = bridge.buildJointStateMessage(
      rclcpp::Time(123456789),
      "base_link",
      {"j1", "j2", "j3", "j4", "j5", "j6"},
      {0.0, 0.1, 0.2, 0.3, 0.4, 0.5},
      {0.5, 0.4, 0.3, 0.2, 0.1, 0.0},
      {1.0, 1.1, 1.2, 1.3, 1.4, 1.5});
  ASSERT_EQ(joint_message.name.size(), 6u);
  EXPECT_EQ(joint_message.header.frame_id, "base_link");
  EXPECT_DOUBLE_EQ(joint_message.position[3], 0.3);
  EXPECT_DOUBLE_EQ(joint_message.effort[5], 1.5);

  bridge.maybeRecordPathSample({0.0, 0.1, 0.2, 0.3, 0.4, 0.5});
  EXPECT_TRUE(samples.empty());
  recording = true;
  bridge.maybeRecordPathSample({0.0, 0.1, 0.2, 0.3, 0.4, 0.5});
  ASSERT_EQ(samples.size(), 1u);
  EXPECT_DOUBLE_EQ(samples.back()[4], 0.4);

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
