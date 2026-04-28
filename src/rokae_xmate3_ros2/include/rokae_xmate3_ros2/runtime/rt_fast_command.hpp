#ifndef ROKAE_XMATE3_ROS2_RUNTIME_RT_FAST_COMMAND_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_RT_FAST_COMMAND_HPP

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

namespace rokae_xmate3_ros2::runtime {

enum class RtFastCommandKind : std::uint8_t {
  joint_position = 0,
  cartesian_position = 1,
  torque = 2,
  stop = 3,
};

enum class RtFastTransport : std::uint8_t {
  unknown = 0,
  shm_ring = 1,
  ros_topic = 2,
  legacy_custom_data = 3,
};

struct RtFastCommandFrame {
  std::uint64_t sequence = 0;
  int rt_mode = -1;
  RtFastCommandKind kind = RtFastCommandKind::joint_position;
  std::array<double, 6> values{};
  bool finished = false;
  std::string dispatch_mode{"idle"};
  std::chrono::steady_clock::time_point sent_at{};
  RtFastTransport transport = RtFastTransport::unknown;
};

inline const char *toString(const RtFastTransport transport) noexcept {
  switch (transport) {
    case RtFastTransport::shm_ring:
      return "shm_ring";
    case RtFastTransport::ros_topic:
      return "ros_rt_topic";
    case RtFastTransport::legacy_custom_data:
      return "legacy_custom_data";
    case RtFastTransport::unknown:
    default:
      return "unknown";
  }
}

inline const char *toString(const RtFastCommandKind kind) noexcept {
  switch (kind) {
    case RtFastCommandKind::joint_position:
      return "joint_position";
    case RtFastCommandKind::cartesian_position:
      return "cartesian_position";
    case RtFastCommandKind::torque:
      return "torque";
    case RtFastCommandKind::stop:
      return "stop";
    default:
      return "joint_position";
  }
}

}  // namespace rokae_xmate3_ros2::runtime

#endif
