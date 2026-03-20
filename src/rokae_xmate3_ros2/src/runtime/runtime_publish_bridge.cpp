#include "runtime/runtime_publish_bridge.hpp"

#include <algorithm>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

namespace rokae_xmate3_ros2::runtime {

RuntimePublishBridge::RuntimePublishBridge(RuntimeContext &runtime_context,
                                           PathRecordingStateProvider path_recording_state_provider,
                                           PathSampleRecorder path_sample_recorder)
    : runtime_context_(runtime_context),
      path_recording_state_provider_(std::move(path_recording_state_provider)),
      path_sample_recorder_(std::move(path_sample_recorder)) {}

RuntimeView RuntimePublishBridge::currentView() const {
  return runtime_context_.currentRuntimeView();
}

rokae_xmate3_ros2::msg::OperationState RuntimePublishBridge::buildOperationStateMessage() const {
  rokae_xmate3_ros2::msg::OperationState msg;
  msg.state = resolve_operation_state(currentView(), runtime_context_.operationStateContext());
  return msg;
}

sensor_msgs::msg::JointState RuntimePublishBridge::buildJointStateMessage(
    const rclcpp::Time &stamp,
    const std::string &frame_id,
    const std::vector<std::string> &joint_names,
    const std::array<double, 6> &position,
    const std::array<double, 6> &velocity,
    const std::array<double, 6> &torque) const {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  const auto joint_count = std::min<std::size_t>(joint_names.size(), position.size());
  msg.name.reserve(joint_count);
  msg.position.reserve(joint_count);
  msg.velocity.reserve(joint_count);
  msg.effort.reserve(joint_count);
  for (std::size_t i = 0; i < joint_count; ++i) {
    msg.name.push_back(joint_names[i]);
    msg.position.push_back(position[i]);
    msg.velocity.push_back(velocity[i]);
    msg.effort.push_back(torque[i]);
  }
  return msg;
}

void RuntimePublishBridge::emitRuntimeStatus(const RuntimeStatus &status,
                                             const rclcpp::Time &stamp,
                                             const rclcpp::Logger &logger) {
  const auto event = build_runtime_log_event(status, last_runtime_logged_revision_);
  if (!event.should_log) {
    return;
  }

  last_runtime_logged_revision_ = event.revision;
  rokae_xmate3_ros2::msg::LogInfo log;
  log.timestamp = std::to_string(stamp.nanoseconds());
  log.content = event.text;
  log.repair.clear();
  log.level = event.warning ? 2 : 1;
  runtime_context_.dataStoreState().appendLog(log);

  if (event.warning) {
    RCLCPP_WARN(logger, "%s", event.text.c_str());
  } else {
    RCLCPP_INFO(logger, "%s", event.text.c_str());
  }
}

bool RuntimePublishBridge::shouldRecordPathSample() const {
  return static_cast<bool>(path_recording_state_provider_) && path_recording_state_provider_();
}

void RuntimePublishBridge::maybeRecordPathSample(const std::array<double, 6> &joint_position) const {
  if (!shouldRecordPathSample() || !path_sample_recorder_) {
    return;
  }
  path_sample_recorder_(joint_position);
}

FeedbackSnapshot buildMoveAppendFeedback(const RuntimeStatus &status,
                                         std::size_t last_completed_segments,
                                         const std::string &last_message) {
  FeedbackSnapshot snapshot;
  if (status.state == ExecutionState::idle) {
    return snapshot;
  }
  if (status.completed_segments == last_completed_segments && status.message == last_message) {
    return snapshot;
  }

  snapshot.should_publish = true;
  snapshot.progress = status.total_segments == 0
                          ? 0.0
                          : static_cast<double>(status.completed_segments) /
                                static_cast<double>(status.total_segments);
  snapshot.current_state = to_string(status.state);
  snapshot.current_cmd_index = static_cast<int>(status.current_segment_index);
  return snapshot;
}

}  // namespace rokae_xmate3_ros2::runtime
