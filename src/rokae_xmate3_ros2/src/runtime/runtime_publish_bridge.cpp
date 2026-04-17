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

rokae_xmate3_ros2::msg::RuntimeDiagnostics RuntimePublishBridge::buildRuntimeDiagnosticsMessage() const {
  const auto snapshot = runtime_context_.diagnosticsState().snapshot();
  rokae_xmate3_ros2::msg::RuntimeDiagnostics msg;
  msg.backend_mode = snapshot.backend_mode;
  msg.control_owner = snapshot.control_owner;
  msg.runtime_phase = snapshot.runtime_phase;
  msg.shutdown_phase = snapshot.shutdown_phase;
  msg.active_request_count = snapshot.active_request_count;
  msg.active_goal_count = snapshot.active_goal_count;
  msg.active_request_id = snapshot.active_request_id;
  msg.active_execution_backend = snapshot.active_execution_backend;
  msg.last_runtime_event = snapshot.last_runtime_event;
  msg.last_plan_summary = snapshot.last_plan_summary;
  msg.last_selected_candidate = snapshot.last_selected_candidate;
  msg.last_plan_failure = snapshot.last_plan_failure;
  msg.last_retimer_note = snapshot.last_retimer_note;
  msg.last_servo_dt = snapshot.last_servo_dt;
  msg.capability_flags = snapshot.capability_flags;
  msg.motion_mode = snapshot.motion_mode;
  msg.rt_mode = snapshot.rt_mode;
  msg.active_profile = snapshot.active_profile;
  msg.loop_hz = snapshot.loop_hz;
  msg.state_stream_hz = snapshot.state_stream_hz;
  msg.command_latency_ms = snapshot.command_latency_ms;
  msg.rt_subscription_plan = snapshot.rt_subscription_plan;
  msg.rt_prearm_status = snapshot.rt_prearm_status;
  msg.rt_watchdog_summary = snapshot.rt_watchdog_summary;
  msg.rt_late_cycle_count = snapshot.rt_late_cycle_count;
  msg.rt_max_gap_ms = snapshot.rt_max_gap_ms;
  msg.rt_avg_gap_ms = snapshot.rt_avg_gap_ms;
  msg.rt_consecutive_late_cycles = snapshot.rt_consecutive_late_cycles;
  msg.rt_stale_state_count = snapshot.rt_stale_state_count;
  msg.rt_command_starvation_windows = snapshot.rt_command_starvation_windows;
  msg.rt_last_trigger_reason = snapshot.rt_last_trigger_reason;
  msg.rt_transport_source = snapshot.rt_transport_source;
  msg.rt_scheduler_state = snapshot.rt_scheduler_state;
  msg.rt_deadline_miss = snapshot.rt_deadline_miss;
  msg.rt_rx_latency_us = snapshot.rt_rx_latency_us;
  msg.rt_queue_depth = snapshot.rt_queue_depth;
  msg.recent_runtime_events = snapshot.recent_runtime_events;
  msg.event_bus_summary = snapshot.event_bus_summary;
  msg.runtime_event_count = snapshot.runtime_event_count;
  msg.planning_rejection_count = snapshot.planning_rejection_count;
  msg.watchdog_trigger_count = snapshot.watchdog_trigger_count;
  msg.profile_capability_summary = snapshot.profile_capability_summary;
  msg.planning_capability_summary = snapshot.planning_capability_summary;
  msg.runtime_option_summary = snapshot.runtime_option_summary;
  msg.last_api_surface = snapshot.last_api_surface;
  msg.last_result_source = snapshot.last_result_source;
  msg.rt_dispatch_mode = snapshot.rt_dispatch_mode;
  msg.rt_state_source = snapshot.rt_state_source;
  msg.model_exactness_summary = snapshot.model_exactness_summary;
  msg.model_primary_backend = snapshot.model_primary_backend;
  msg.model_fallback_used = snapshot.model_fallback_used;
  msg.catalog_provenance_summary = snapshot.catalog_provenance_summary;
  msg.tool_catalog_size = snapshot.tool_catalog_size;
  msg.wobj_catalog_size = snapshot.wobj_catalog_size;
  msg.project_catalog_size = snapshot.project_catalog_size;
  msg.register_catalog_size = snapshot.register_catalog_size;
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

  const double legacy_period_sec = input.min_publish_period_sec > 0.0 ? input.min_publish_period_sec : 0.0;
  const double joint_state_period_sec =
      input.joint_state_publish_period_sec > 0.0 ? input.joint_state_publish_period_sec : legacy_period_sec;
  const double operation_state_period_sec =
      input.operation_state_publish_period_sec > 0.0 ? input.operation_state_publish_period_sec : legacy_period_sec;
  const double diagnostics_period_sec =
      input.diagnostics_publish_period_sec > 0.0 ? input.diagnostics_publish_period_sec : operation_state_period_sec;

  const auto period_due = [&](const double period_sec, std::int64_t last_tick_ns) {
    if (period_sec <= 0.0 || last_tick_ns <= 0) {
      return true;
    }
    const auto publish_period_ns =
        static_cast<std::int64_t>(period_sec * static_cast<double>(kNanosecondsPerSecond));
    return (input.stamp.nanoseconds() - last_tick_ns) >= publish_period_ns;
  };

  if (input.joint_names != nullptr && period_due(joint_state_period_sec, last_joint_state_publish_ns_)) {
    output.publish_joint_state = true;
    output.joint_state = buildJointStateMessage(
        input.stamp, input.frame_id, *input.joint_names, input.position, input.velocity, input.torque);
    last_joint_state_publish_ns_ = input.stamp.nanoseconds();
  }

  if (period_due(operation_state_period_sec, last_operation_state_publish_ns_)) {
    output.publish_operation_state = true;
    output.operation_state.state = resolve_operation_state(view.runtime, view.operation_state);
    last_operation_state_publish_ns_ = input.stamp.nanoseconds();
  }

  if (period_due(diagnostics_period_sec, last_diagnostics_publish_ns_)) {
    output.publish_runtime_diagnostics = true;
    last_diagnostics_publish_ns_ = input.stamp.nanoseconds();
  }

  if (view.program.recording_path) {
    runtime_context_.programState().recordPathSample(input.stamp.seconds(), input.position, input.velocity);
    output.recorded_path_sample = true;
  }

  return output;
}

void RuntimePublishBridge::emitRuntimeStatus(const RuntimeStatus &status,
                                             const rclcpp::Time &stamp,
                                             const rclcpp::Logger &logger) {
  runtime_context_.diagnosticsState().updateRuntimeStatus(status);
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
RuntimePublishBridge::buildMoveAppendQueuedResult(const std::string &request_id,
                                                  const std::string &message) const {
  auto result = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Result>();
  result->success = true;
  result->cmd_id = request_id;
  result->message = message.empty() ? std::string{"queued awaiting moveStart"} : message;
  return result;
}

std::shared_ptr<rokae_xmate3_ros2::action::MoveAppend::Result>
RuntimePublishBridge::buildMoveAppendTerminalResult(const std::string &request_id,
                                                    const RuntimeStatus &status) const {
  auto result = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Result>();
  result->success = (status.state == ExecutionState::completed ||
                     status.state == ExecutionState::completed_relaxed) &&
                    status.terminal_success;
  result->cmd_id = request_id;
  result->message = status.message.empty() ? to_string(status.state) : status.message;
  return result;
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
