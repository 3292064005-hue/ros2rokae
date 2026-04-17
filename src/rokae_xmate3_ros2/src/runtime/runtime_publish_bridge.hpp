#ifndef ROKAE_XMATE3_ROS2_RUNTIME_PUBLISH_BRIDGE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_PUBLISH_BRIDGE_HPP

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "runtime/runtime_context.hpp"
#include "rokae_xmate3_ros2/action/move_append.hpp"
#include "rokae_xmate3_ros2/msg/operation_state.hpp"
#include "rokae_xmate3_ros2/msg/runtime_diagnostics.hpp"

namespace rokae_xmate3_ros2::runtime {

struct FeedbackSnapshot {
  bool should_publish = false;
  double progress = 0.0;
  std::string current_state;
  int current_cmd_index = 0;
};

struct PublisherTickInput {
  rclcpp::Time stamp;
  std::string frame_id;
  const std::vector<std::string> *joint_names = nullptr;
  std::array<double, 6> position{};
  std::array<double, 6> velocity{};
  std::array<double, 6> torque{};
  double min_publish_period_sec = 0.0;
  double joint_state_publish_period_sec = 0.0;
  double operation_state_publish_period_sec = 0.0;
  double diagnostics_publish_period_sec = 0.0;
};

struct PublisherTickOutput {
  bool publish_joint_state = false;
  sensor_msgs::msg::JointState joint_state;
  bool publish_operation_state = false;
  rokae_xmate3_ros2::msg::OperationState operation_state;
  bool publish_runtime_diagnostics = false;
  bool recorded_path_sample = false;
};

class RuntimePublishBridge {
 public:
  using MoveAppendGoalHandle =
      rclcpp_action::ServerGoalHandle<rokae_xmate3_ros2::action::MoveAppend>;

  explicit RuntimePublishBridge(RuntimeContext &runtime_context);

  [[nodiscard]] RuntimeView currentView() const;
  [[nodiscard]] rokae_xmate3_ros2::msg::OperationState buildOperationStateMessage() const;
  [[nodiscard]] rokae_xmate3_ros2::msg::RuntimeDiagnostics buildRuntimeDiagnosticsMessage() const;
  [[nodiscard]] sensor_msgs::msg::JointState buildJointStateMessage(
      const rclcpp::Time &stamp,
      const std::string &frame_id,
      const std::vector<std::string> &joint_names,
      const std::array<double, 6> &position,
      const std::array<double, 6> &velocity,
      const std::array<double, 6> &torque) const;
  [[nodiscard]] PublisherTickOutput buildPublisherTick(const PublisherTickInput &input);

  void emitRuntimeStatus(const RuntimeStatus &status,
                         const rclcpp::Time &stamp,
                         const rclcpp::Logger &logger);

  [[nodiscard]] RuntimeStatus waitForRequestUpdate(const std::string &request_id,
                                                   std::uint64_t last_revision,
                                                   std::chrono::milliseconds timeout) const;
  [[nodiscard]] std::shared_ptr<rokae_xmate3_ros2::action::MoveAppend::Feedback>
  buildMoveAppendFeedbackMessage(const FeedbackSnapshot &snapshot) const;
  /**
   * @brief Build the public xMate6 MoveAppend result for a successfully queued request.
   * @param request_id Stable queued request identifier returned to SDK callers.
   * @param message Queue-acceptance detail. Empty input is normalized to the canonical
   *        "queued awaiting moveStart" text so install-facing clients observe one contract.
   * @return Action result marking queue acceptance as success.
   * @throws None.
   */
  [[nodiscard]] std::shared_ptr<rokae_xmate3_ros2::action::MoveAppend::Result>
  buildMoveAppendQueuedResult(const std::string &request_id, const std::string &message) const;
  /**
   * @brief Build a terminal MoveAppend result for internal/runtime status projections.
   * @param request_id Stable queued request identifier.
   * @param status Terminal runtime status.
   * @return Action result representing the terminal runtime outcome.
   * @throws None.
   * @note Public xMate6 action clients no longer wait for this result before returning from
   *       MoveAppend; terminal outcomes are consumed through runtime state / events after moveStart().
   */
  [[nodiscard]] std::shared_ptr<rokae_xmate3_ros2::action::MoveAppend::Result>
  buildMoveAppendTerminalResult(const std::string &request_id, const RuntimeStatus &status) const;

 private:
  RuntimeContext &runtime_context_;
  std::uint64_t last_runtime_logged_revision_ = 0;
  std::int64_t last_joint_state_publish_ns_ = 0;
  std::int64_t last_operation_state_publish_ns_ = 0;
  std::int64_t last_diagnostics_publish_ns_ = 0;
};

[[nodiscard]] FeedbackSnapshot buildMoveAppendFeedback(const RuntimeStatus &status,
                                                       std::size_t last_completed_segments,
                                                       const std::string &last_message);

}  // namespace rokae_xmate3_ros2::runtime

#endif
