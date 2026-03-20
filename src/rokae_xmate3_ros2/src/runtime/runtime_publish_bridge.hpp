#ifndef ROKAE_XMATE3_ROS2_RUNTIME_PUBLISH_BRIDGE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_PUBLISH_BRIDGE_HPP

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "runtime/runtime_context.hpp"
#include "rokae_xmate3_ros2/msg/operation_state.hpp"

namespace rokae_xmate3_ros2::runtime {

struct FeedbackSnapshot {
  bool should_publish = false;
  double progress = 0.0;
  std::string current_state;
  int current_cmd_index = 0;
};

class RuntimePublishBridge {
 public:
  using PathRecordingStateProvider = std::function<bool()>;
  using PathSampleRecorder = std::function<void(const std::array<double, 6> &)>;

  RuntimePublishBridge(RuntimeContext &runtime_context,
                       PathRecordingStateProvider path_recording_state_provider = {},
                       PathSampleRecorder path_sample_recorder = {});

  [[nodiscard]] RuntimeView currentView() const;
  [[nodiscard]] rokae_xmate3_ros2::msg::OperationState buildOperationStateMessage() const;
  [[nodiscard]] sensor_msgs::msg::JointState buildJointStateMessage(
      const rclcpp::Time &stamp,
      const std::string &frame_id,
      const std::vector<std::string> &joint_names,
      const std::array<double, 6> &position,
      const std::array<double, 6> &velocity,
      const std::array<double, 6> &torque) const;

  void emitRuntimeStatus(const RuntimeStatus &status,
                         const rclcpp::Time &stamp,
                         const rclcpp::Logger &logger);

  [[nodiscard]] bool shouldRecordPathSample() const;
  void maybeRecordPathSample(const std::array<double, 6> &joint_position) const;

 private:
  RuntimeContext &runtime_context_;
  PathRecordingStateProvider path_recording_state_provider_;
  PathSampleRecorder path_sample_recorder_;
  std::uint64_t last_runtime_logged_revision_ = 0;
};

[[nodiscard]] FeedbackSnapshot buildMoveAppendFeedback(const RuntimeStatus &status,
                                                       std::size_t last_completed_segments,
                                                       const std::string &last_message);

}  // namespace rokae_xmate3_ros2::runtime

#endif
