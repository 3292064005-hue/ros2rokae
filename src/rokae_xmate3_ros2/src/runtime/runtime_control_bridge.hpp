#ifndef ROKAE_XMATE3_ROS2_RUNTIME_CONTROL_BRIDGE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_CONTROL_BRIDGE_HPP

#include <chrono>
#include <vector>

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"
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
  double rt_command_timeout_sec = 0.25;
  std::array<double, 6> joint_position_gain{{220.0, 220.0, 180.0, 90.0, 60.0, 40.0}};
  std::array<double, 6> joint_damping_gain{{28.0, 28.0, 22.0, 12.0, 8.0, 6.0}};
  bool authoritative_servo_clock = false;
  double authoritative_servo_period_sec = rokae_xmate3_ros2::spec::xmate3::kServoTickSec;
  int max_servo_substeps_per_update = 8;
  bool allow_topic_rt_transport = true;
  bool allow_legacy_rt_custom_data = true;
  bool require_shm_rt_transport = false;
  bool fail_on_rt_deadline_miss = false;
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

  /**
   * @brief Execute one runtime servo update against the active backend.
   *
   * The bridge is the only place where runtime ownership arbitration, RT ingress
   * selection, watchdog updates, and backend control application are combined. The
   * call is intentionally synchronous so the host thread can be the single owner of
   * the authoritative servo clock.
   *
   * @param backend Active runtime backend that receives control commands.
   * @param snapshot Latest robot state snapshot sampled by the host.
   * @param dt Wall/sim time elapsed since the previous host tick, in seconds.
   * @return Per-tick control outcome including brake/control side effects.
   */
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
  std::uint64_t last_rt_sequence_ = 0;
  std::array<double, 6> last_torque_command_{};
  std::array<double, 6> last_joint_target_{};
  std::array<double, 6> last_cartesian_target_{};
  bool has_last_torque_command_ = false;
  bool has_last_joint_target_ = false;
  bool has_last_cartesian_target_ = false;
  std::chrono::steady_clock::time_point last_rt_fast_received_at_{};
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
