#ifndef ROKAE_XMATE3_ROS2_RUNTIME_OPERATION_STATE_ADAPTER_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_OPERATION_STATE_ADAPTER_HPP

#include <cstdint>
#include <string>

#include "runtime/runtime_types.hpp"

namespace rokae_xmate3_ros2::runtime {

struct OperationStateContext {
  bool connected = false;
  bool power_on = false;
  bool drag_mode = false;
  bool rl_project_running = false;
  int rt_control_mode = -1;
};

struct RuntimeLogEvent {
  bool should_log = false;
  bool warning = false;
  std::uint64_t revision = 0;
  std::string text;
};

[[nodiscard]] uint8_t resolve_operation_state(const RuntimeView &view,
                                              const OperationStateContext &context) noexcept;

[[nodiscard]] RuntimeLogEvent build_runtime_log_event(const RuntimeStatus &status,
                                                      std::uint64_t last_revision);

}  // namespace rokae_xmate3_ros2::runtime

#endif
