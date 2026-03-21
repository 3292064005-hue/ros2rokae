#ifndef ROKAE_XMATE3_ROS2_RUNTIME_CONTROL_BRIDGE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_CONTROL_BRIDGE_HPP

#include "runtime/runtime_context.hpp"

namespace rokae_xmate3_ros2::runtime {

struct ControlTickResult {
  RuntimeStatus status;
  bool control_cleared = false;
  bool brake_locked = false;
  bool brake_released = false;
  bool runtime_ticked = false;
};

class RuntimeControlBridge {
 public:
  explicit RuntimeControlBridge(RuntimeContext &runtime_context);

  [[nodiscard]] ControlTickResult tick(BackendInterface &backend,
                                       const RobotSnapshot &snapshot,
                                       double dt);

 private:
  RuntimeContext &runtime_context_;
  double servo_accumulator_sec_ = 0.0;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
