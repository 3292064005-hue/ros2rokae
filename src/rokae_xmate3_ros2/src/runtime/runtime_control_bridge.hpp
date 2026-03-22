#ifndef ROKAE_XMATE3_ROS2_RUNTIME_CONTROL_BRIDGE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_CONTROL_BRIDGE_HPP

#include "runtime/runtime_context.hpp"

namespace rokae_xmate3_ros2::runtime {

struct RuntimeControlBridgeConfig {
  std::array<double, 6> collision_nominal_thresholds{{35.0, 35.0, 30.0, 18.0, 12.0, 8.0}};
  double collision_slow_scale = 0.25;
  double collision_retreat_distance = 0.04;
};

struct ControlTickResult {
  RuntimeStatus status;
  bool control_cleared = false;
  bool brake_locked = false;
  bool brake_released = false;
  bool runtime_ticked = false;
};

class RuntimeControlBridge {
 public:
  explicit RuntimeControlBridge(RuntimeContext &runtime_context,
                                RuntimeControlBridgeConfig config = {});

  [[nodiscard]] ControlTickResult tick(BackendInterface &backend,
                                       const RobotSnapshot &snapshot,
                                       double dt);

 private:
  RuntimeContext &runtime_context_;
  RuntimeControlBridgeConfig config_;
  double servo_accumulator_sec_ = 0.0;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
