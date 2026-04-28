#include "runtime/rt_runtime_profile.hpp"

#include <sstream>

namespace rokae_xmate3_ros2::runtime {
namespace {

RuntimeRtProfileConfig makeBaseProfile(const std::string &requested_profile,
                                       const std::string &effective_profile,
                                       const RuntimeHostKind host_kind) {
  RuntimeRtProfileConfig config;
  config.requested_profile = requested_profile;
  config.effective_profile = effective_profile;
  config.host_kind = host_kind;
  return config;
}

}  // namespace

RuntimeRtProfileConfig resolveRuntimeRtProfile(const std::string &requested_profile,
                                               const RuntimeHostKind host_kind) {
  if (requested_profile == "hard_1khz") {
    auto config = makeBaseProfile(requested_profile, requested_profile, host_kind);
    config.authoritative_servo_clock = true;
    config.allow_topic_rt_ingress = false;
    config.allow_legacy_rt_custom_data = false;
    config.require_shm_transport = true;
    config.fail_on_degraded_scheduler = true;
    config.fail_on_rt_deadline_miss = true;
    config.executor_threads = 1;
    config.publish_rates = {0.004, 0.020, 0.020};
    config.diagnostics_note = "single authoritative 1ms servo; shm-only RT ingress; fail-fast scheduler contract";
    if (host_kind != RuntimeHostKind::daemonized_runtime) {
      config.supported = false;
      config.startup_error = "hard_1khz is only supported by the daemonized runtime host";
    }
    return config;
  }

  if (requested_profile == "rt_hardened") {
    auto config = makeBaseProfile(requested_profile, requested_profile, host_kind);
    config.authoritative_servo_clock = true;
    config.allow_topic_rt_ingress = true;
    config.allow_legacy_rt_custom_data = false;
    config.require_shm_transport = false;
    config.fail_on_degraded_scheduler = false;
    config.executor_threads = 2;
    config.publish_rates = {0.004, 0.020, 0.020};
    config.diagnostics_note = "authoritative servo; decoupled observability; legacy RT custom-data disabled";
    return config;
  }

  if (requested_profile == "rt_sim_experimental_best_effort") {
    auto config = makeBaseProfile(requested_profile, requested_profile, host_kind);
    config.authoritative_servo_clock = true;
    config.allow_topic_rt_ingress = true;
    config.allow_legacy_rt_custom_data = true;
    config.require_shm_transport = false;
    config.fail_on_degraded_scheduler = false;
    config.executor_threads = 2;
    config.publish_rates = {0.004, 0.020, 0.020};
    config.diagnostics_note = "best-effort RT with authoritative servo and decoupled observability";
    return config;
  }

  if (requested_profile == "hybrid_bridge") {
    auto config = makeBaseProfile(requested_profile, requested_profile, host_kind);
    config.authoritative_servo_clock = false;
    config.allow_topic_rt_ingress = true;
    config.allow_legacy_rt_custom_data = true;
    config.require_shm_transport = false;
    config.fail_on_degraded_scheduler = false;
    config.executor_threads = 2;
    config.publish_rates = {0.008, 0.020, 0.020};
    config.diagnostics_note = "hybrid bridge profile; simulation-step servo accumulation";
    return config;
  }

  auto config = makeBaseProfile(requested_profile, "nrt_strict_parity", host_kind);
  config.authoritative_servo_clock = false;
  config.allow_topic_rt_ingress = true;
  config.allow_legacy_rt_custom_data = true;
  config.require_shm_transport = false;
  config.fail_on_degraded_scheduler = false;
  config.executor_threads = 2;
  config.publish_rates = {0.020, 0.050, 0.050};
  config.diagnostics_note = "strict NRT parity profile";
  return config;
}

std::string summarizeRuntimeRtProfile(const RuntimeRtProfileConfig &profile) {
  std::ostringstream stream;
  stream << "requested=" << profile.requested_profile
         << ";effective=" << profile.effective_profile
         << ";host="
         << (profile.host_kind == RuntimeHostKind::daemonized_runtime ? "daemonized_runtime" : "gazebo_plugin")
         << ";authoritative_servo=" << (profile.authoritative_servo_clock ? "true" : "false")
         << ";allow_topic_rt_ingress=" << (profile.allow_topic_rt_ingress ? "true" : "false")
         << ";allow_legacy_rt_custom_data=" << (profile.allow_legacy_rt_custom_data ? "true" : "false")
         << ";require_shm_transport=" << (profile.require_shm_transport ? "true" : "false")
         << ";fail_on_degraded_scheduler=" << (profile.fail_on_degraded_scheduler ? "true" : "false")
         << ";fail_on_rt_deadline_miss=" << (profile.fail_on_rt_deadline_miss ? "true" : "false")
         << ";executor_threads=" << profile.executor_threads
         << ";joint_state_period_sec=" << profile.publish_rates.joint_state_period_sec
         << ";operation_state_period_sec=" << profile.publish_rates.operation_state_period_sec
         << ";diagnostics_period_sec=" << profile.publish_rates.diagnostics_period_sec
         << ";note=" << profile.diagnostics_note;
  if (!profile.startup_error.empty()) {
    stream << ";startup_error=" << profile.startup_error;
  }
  return stream.str();
}

}  // namespace rokae_xmate3_ros2::runtime
