#include "runtime/runtime_publish_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace rokae_xmate3_ros2::runtime {

namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

}  // namespace

RuntimePublishBridge::RuntimePublishBridge(RuntimeContext &runtime_context)
    : runtime_context_(runtime_context) {}

RuntimeView RuntimePublishBridge::currentView() const {
  return runtime_context_.readView().runtime;
}

rokae_xmate3_ros2::msg::OperationState RuntimePublishBridge::buildOperationStateMessage() const {
  const auto view = runtime_context_.readView();
  rokae_xmate3_ros2::msg::OperationState msg;
  msg.state = resolve_operation_state(view.runtime, view.operation_state);
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

PublisherTickOutput RuntimePublishBridge::buildPublisherTick(const PublisherTickInput &input) {
  PublisherTickOutput output;
  const auto view = runtime_context_.readView();

  bool publish_due = true;
  if (input.min_publish_period_sec > 0.0 && last_publisher_tick_ns_ > 0) {
    const auto publish_period_ns =
        static_cast<std::int64_t>(input.min_publish_period_sec * static_cast<double>(kNanosecondsPerSecond));
    publish_due = (input.stamp.nanoseconds() - last_publisher_tick_ns_) >= publish_period_ns;
  }

  if (publish_due) {
    output.publish_operation_state = true;
    output.operation_state.state = resolve_operation_state(view.runtime, view.operation_state);
    if (input.joint_names != nullptr) {
      output.publish_joint_state = true;
      output.joint_state = buildJointStateMessage(
          input.stamp, input.frame_id, *input.joint_names, input.position, input.velocity, input.torque);
    }
    last_publisher_tick_ns_ = input.stamp.nanoseconds();
  }

  if (view.program.recording_path) {
    runtime_context_.programState().recordPathSample(input.position);
    output.recorded_path_sample = true;
  }

  return output;
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

RuntimeStatus RuntimePublishBridge::waitForRequestUpdate(const std::string &request_id,
                                                         std::uint64_t last_revision,
                                                         std::chrono::milliseconds timeout) const {
  return runtime_context_.requestCoordinator().waitForUpdate(request_id, last_revision, timeout);
}

std::shared_ptr<rokae_xmate3_ros2::action::MoveAppend::Feedback>
RuntimePublishBridge::buildMoveAppendFeedbackMessage(const FeedbackSnapshot &snapshot) const {
  auto feedback = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Feedback>();
  feedback->progress = snapshot.progress;
  feedback->current_state = snapshot.current_state;
  feedback->current_cmd_index = snapshot.current_cmd_index;
  return feedback;
}

std::shared_ptr<rokae_xmate3_ros2::action::MoveAppend::Result>
RuntimePublishBridge::buildMoveAppendResult(const std::string &request_id,
                                            const RuntimeStatus &status) const {
  auto result = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Result>();
  result->success = (status.state == ExecutionState::completed ||
                     status.state == ExecutionState::completed_relaxed) &&
                    status.terminal_success;
  result->cmd_id = request_id;
  result->message = status.message.empty() ? to_string(status.state) : status.message;
  return result;
}

void RuntimePublishBridge::driveMoveAppendGoal(
    const std::shared_ptr<MoveAppendGoalHandle> &goal_handle,
    const std::string &request_id) {
  std::string last_state;
  std::size_t last_completed = std::numeric_limits<std::size_t>::max();
  std::uint64_t last_revision = 0;

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      runtime_context_.requestCoordinator().stop("move append canceled");
      auto result = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Result>();
      result->success = false;
      result->cmd_id = request_id;
      result->message = "Canceled";
      goal_handle->canceled(result);
      return;
    }

    const auto status = waitForRequestUpdate(request_id, last_revision, std::chrono::milliseconds(100));
    if (status.revision > last_revision) {
      last_revision = status.revision;
    }

    const auto feedback_snapshot = buildMoveAppendFeedback(status, last_completed, last_state);
    if (feedback_snapshot.should_publish) {
      goal_handle->publish_feedback(buildMoveAppendFeedbackMessage(feedback_snapshot));
      last_completed = status.completed_segments;
      last_state = status.message;
    }

    if (status.terminal()) {
      auto result = buildMoveAppendResult(request_id, status);
      if (result->success) {
        goal_handle->succeed(result);
      } else if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
      return;
    }
  }

  auto result = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Result>();
  result->success = false;
  result->cmd_id = request_id;
  result->message = "MoveAppend interrupted";
  goal_handle->abort(result);
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
