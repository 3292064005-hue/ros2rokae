#ifndef ROKAE_XMATE3_ROS2_RUNTIME_RT_RUNTIME_PROFILE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_RT_RUNTIME_PROFILE_HPP

#include <string>

namespace rokae_xmate3_ros2::runtime {

enum class RuntimeHostKind {
  daemonized_runtime,
  gazebo_plugin,
};

struct RuntimePublishRateConfig {
  double joint_state_period_sec = 0.004;
  double operation_state_period_sec = 0.020;
  double diagnostics_period_sec = 0.020;
};

struct RuntimeRtProfileConfig {
  std::string requested_profile{"nrt_strict_parity"};
  std::string effective_profile{"nrt_strict_parity"};
  RuntimeHostKind host_kind = RuntimeHostKind::daemonized_runtime;
  bool authoritative_servo_clock = false;
  bool allow_topic_rt_ingress = true;
  bool allow_legacy_rt_custom_data = true;
  bool require_shm_transport = false;
  bool fail_on_degraded_scheduler = false;
  bool fail_on_rt_deadline_miss = false;
  bool supported = true;
  int executor_threads = 2;
  double servo_period_sec = 0.001;
  double servo_deadline_warn_sec = 0.0012;
  RuntimePublishRateConfig publish_rates{};
  std::string diagnostics_note{"compat profile"};
  std::string startup_error;
};

/**
 * @brief Resolve the runtime-host profile into concrete execution and publishing rules.
 *
 * The returned structure is the single source of truth for runtime-side RT policy:
 * authoritative tick ownership, ingress transport constraints, scheduler strictness,
 * executor width, and observability publish periods.
 *
 * @param requested_profile User/config requested runtime profile name.
 * @param host_kind Hosting shape that will own the runtime (daemon or Gazebo plugin).
 * @return Fully resolved runtime profile contract.
 */
[[nodiscard]] RuntimeRtProfileConfig resolveRuntimeRtProfile(
    const std::string &requested_profile,
    RuntimeHostKind host_kind);

/**
 * @brief Summarize the resolved runtime profile for diagnostics and logs.
 *
 * @param profile Resolved runtime profile contract.
 * @return Stable semicolon-delimited summary string.
 */
[[nodiscard]] std::string summarizeRuntimeRtProfile(const RuntimeRtProfileConfig &profile);

}  // namespace rokae_xmate3_ros2::runtime

#endif
