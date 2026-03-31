#ifndef ROKAE_XMATE3_ROS2_RUNTIME_CONTROL_BRIDGE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_CONTROL_BRIDGE_HPP

#include <vector>

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "runtime/rt_subscription_plan.hpp"
#include "runtime/rt_watchdog.hpp"
#include "runtime/runtime_context.hpp"

namespace rokae_xmate3_ros2::runtime {

struct RuntimeControlBridgeConfig {
  std::array<double, 6> collision_nominal_thresholds{{35.0, 35.0, 30.0, 18.0, 12.0, 8.0}};
  double collision_slow_scale = 0.25;
  double collision_retreat_distance = 0.04;
  double collision_hysteresis_ratio = 0.15;
  double collision_confirm_window_sec = 0.01;
  double collision_debounce_sec = 0.10;
  double servo_lag_warning_sec = 0.004;
  std::vector<std::string> rt_default_fields{"q_m", "dq_m", "tau_m"};
  bool rt_use_state_data_in_loop = true;
  bool rt_network_tolerance_configured = true;
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
  gazebo::xMate3Kinematics kinematics_;
  double servo_accumulator_sec_ = 0.0;
  double collision_candidate_time_sec_ = 0.0;
  double collision_debounce_remaining_sec_ = 0.0;
  std::size_t collision_candidate_axis_ = 0;
  bool collision_candidate_active_ = false;
  std::size_t servo_lag_warning_count_ = 0;
  RtSubscriptionPlan default_rt_plan_{};
  RtWatchdog rt_watchdog_{};
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
