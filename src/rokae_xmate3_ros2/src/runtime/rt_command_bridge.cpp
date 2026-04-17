#include "runtime/rt_command_bridge.hpp"

#include <chrono>
#include <sstream>

#include "rokae_xmate3_ros2/robot.hpp"
#include "rokae_xmate3_ros2/runtime/rt_fast_command.hpp"
#include "rokae_xmate3_ros2/runtime/rt_semantic_topics.hpp"

namespace rokae_xmate3_ros2::runtime::rt_command_bridge {
namespace {

std::string boolText(const bool value) {
  return value ? "1" : "0";
}

std::string serializeValues(const std::array<double, 6> &values) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      oss << ',';
    }
    oss << values[i];
  }
  return oss.str();
}

std::string buildPayload(const std::uint64_t sequence,
                         const std::array<double, 6> &values,
                         const bool finished) {
  std::ostringstream oss;
  oss << "seq=" << sequence << ";finished=" << boolText(finished)
      << ";values=" << serializeValues(values);
  return oss.str();
}

const char *topicForKind(const CommandKind kind) {
  switch (kind) {
    case CommandKind::JointPosition:
      return rt_topics::kControlJointPosition;
    case CommandKind::CartesianPosition:
      return rt_topics::kControlCartesianPosition;
    case CommandKind::Torque:
      return rt_topics::kControlTorque;
  }
  return rt_topics::kControlJointPosition;
}

RtFastCommandKind toFastKind(const CommandKind kind) noexcept {
  switch (kind) {
    case CommandKind::JointPosition:
      return RtFastCommandKind::joint_position;
    case CommandKind::CartesianPosition:
      return RtFastCommandKind::cartesian_position;
    case CommandKind::Torque:
      return RtFastCommandKind::torque;
  }
  return RtFastCommandKind::joint_position;
}

bool send(rokae::ros2::xMateRobot &robot,
          const std::string &topic,
          const std::string &payload,
          std::error_code &ec) noexcept {
  robot.sendCustomData(topic, payload, ec);
  return !ec;
}

}  // namespace

bool publishMetadata(rokae::ros2::xMateRobot &robot,
                     const std::string &dispatch_mode,
                     std::error_code &ec) noexcept {
  if (!send(robot, rt_topics::kControlSurface, "xCoreSDK_compat", ec)) {
    return false;
  }
  if (!send(robot, rt_topics::kControlDispatchMode, dispatch_mode, ec)) {
    return false;
  }
  return send(robot, rt_topics::kControlStop, "0", ec);
}

bool publishCommand(rokae::ros2::xMateRobot &robot,
                    std::atomic<std::uint64_t> &sequence,
                    const CommandKind kind,
                    const std::array<double, 6> &values,
                    const bool finished,
                    std::error_code &ec,
                    const std::string &dispatch_mode,
                    const int rt_mode) noexcept {
  const auto seq = sequence.fetch_add(1);

  RtFastCommandFrame frame;
  frame.sequence = seq;
  frame.rt_mode = rt_mode;
  frame.kind = toFastKind(kind);
  frame.values = values;
  frame.finished = finished;
  frame.dispatch_mode = dispatch_mode;
  frame.sent_at = std::chrono::steady_clock::now();
  frame.transport = RtFastTransport::unknown;
  std::uint32_t queue_depth = 0;
  if (robot.publishRtFastCommand(frame, queue_depth, ec)) {
    return true;
  }

  if (!publishMetadata(robot, dispatch_mode, ec)) {
    return false;
  }
  if (!send(robot, rt_topics::kControlSequence, std::to_string(seq), ec)) {
    return false;
  }
  return send(robot, topicForKind(kind), buildPayload(seq, values, finished), ec);
}

bool publishStop(rokae::ros2::xMateRobot &robot, std::error_code &ec) noexcept {
  RtFastCommandFrame frame;
  frame.sequence = 0;
  frame.rt_mode = -1;
  frame.kind = RtFastCommandKind::stop;
  frame.finished = true;
  frame.dispatch_mode = "independent_rt";
  frame.sent_at = std::chrono::steady_clock::now();
  frame.transport = RtFastTransport::unknown;
  std::uint32_t queue_depth = 0;
  if (robot.publishRtFastCommand(frame, queue_depth, ec)) {
    return true;
  }
  return send(robot, rt_topics::kControlStop, "1", ec);
}

}  // namespace rokae_xmate3_ros2::runtime::rt_command_bridge
