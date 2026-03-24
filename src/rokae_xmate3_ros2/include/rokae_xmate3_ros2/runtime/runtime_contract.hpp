#ifndef ROKAE_XMATE3_ROS2_RUNTIME_RUNTIME_CONTRACT_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_RUNTIME_CONTRACT_HPP

#include <cstddef>
#include <cstdint>
#include <string>

namespace rokae_xmate3_ros2::runtime {

enum class ControlOwner : std::uint8_t {
  none,
  effort,
  trajectory,
};

enum class RuntimePhase : std::uint8_t {
  idle,
  planning,
  executing,
  faulted,
};

enum class ShutdownPhase : std::uint8_t {
  running,
  draining,
  backend_detached,
  safe_to_delete,
  safe_to_stop_world,
  finished,
  faulted,
};

struct RuntimeContractView {
  ControlOwner owner = ControlOwner::none;
  RuntimePhase runtime_phase = RuntimePhase::idle;
  ShutdownPhase shutdown_phase = ShutdownPhase::running;
  std::size_t active_request_count = 0;
  std::size_t active_goal_count = 0;
  bool backend_quiescent = false;
  bool safe_to_delete = false;
  bool safe_to_stop_world = false;
  std::string message;
};

template <typename Enum>
constexpr std::uint8_t to_u8(Enum value) noexcept {
  return static_cast<std::uint8_t>(value);
}

inline const char *to_string(ControlOwner owner) noexcept {
  switch (owner) {
    case ControlOwner::effort:
      return "effort";
    case ControlOwner::trajectory:
      return "trajectory";
    default:
      return "none";
  }
}

inline const char *to_string(RuntimePhase phase) noexcept {
  switch (phase) {
    case RuntimePhase::planning:
      return "planning";
    case RuntimePhase::executing:
      return "executing";
    case RuntimePhase::faulted:
      return "faulted";
    default:
      return "idle";
  }
}

inline const char *to_string(ShutdownPhase phase) noexcept {
  switch (phase) {
    case ShutdownPhase::draining:
      return "draining";
    case ShutdownPhase::backend_detached:
      return "backend_detached";
    case ShutdownPhase::safe_to_delete:
      return "safe_to_delete";
    case ShutdownPhase::safe_to_stop_world:
      return "safe_to_stop_world";
    case ShutdownPhase::finished:
      return "finished";
    case ShutdownPhase::faulted:
      return "faulted";
    default:
      return "running";
  }
}

}  // namespace rokae_xmate3_ros2::runtime

#endif  // ROKAE_XMATE3_ROS2_RUNTIME_RUNTIME_CONTRACT_HPP
