#include "runtime/backend_contract_catalog.hpp"

#include <algorithm>

namespace rokae_xmate3_ros2::runtime {
namespace {

BackendContractDescriptor makeDescriptor(std::string backend_mode,
                                         std::string provider_class,
                                         std::string owner_rule,
                                         std::string required_controller,
                                         std::string default_runtime_profile,
                                         bool supports_trajectory_execution,
                                         bool supports_effort_owner,
                                         bool supports_rt_fast_path,
                                         std::vector<std::string> baseline_capability_flags) {
  BackendContractDescriptor descriptor;
  descriptor.backend_mode = std::move(backend_mode);
  descriptor.provider_class = std::move(provider_class);
  descriptor.owner_rule = std::move(owner_rule);
  descriptor.required_controller = std::move(required_controller);
  descriptor.default_runtime_profile = std::move(default_runtime_profile);
  descriptor.supports_trajectory_execution = supports_trajectory_execution;
  descriptor.supports_effort_owner = supports_effort_owner;
  descriptor.supports_rt_fast_path = supports_rt_fast_path;
  descriptor.baseline_capability_flags = std::move(baseline_capability_flags);
  descriptor.authority_scope = supports_trajectory_execution ? "runtime_request_coordinator" : "motion_runtime_view";
  descriptor.fidelity_class = supports_trajectory_execution ? "controller_semantic_parity" : "simulation_grade";
  return descriptor;
}

void appendUnique(std::vector<std::string> &dst, const std::vector<std::string> &src) {
  for (const auto &value : src) {
    if (std::find(dst.begin(), dst.end(), value) == dst.end()) {
      dst.push_back(value);
    }
  }
}

}  // namespace

std::string normalizeBackendModeKey(const std::string &backend_mode) {
  if (backend_mode == "daemonized_headless_sim") {
    return "headless_sim";
  }
  if (backend_mode == "sim_runtime_headless" || backend_mode == "headless") {
    return "headless_sim";
  }
  return backend_mode;
}

BackendContractDescriptor describeBackendMode(const std::string &backend_mode) {
  const auto normalized_backend_mode = normalizeBackendModeKey(backend_mode);
  if (normalized_backend_mode == "jtc") {
    return makeDescriptor("jtc",
                          "trajectory_executor",
                          "trajectory_owner",
                          "joint_trajectory_controller",
                          "nrt_strict_parity",
                          true,
                          false,
                          false,
                          {"simulation.gazebo11", "ros2.humble", "backend.jtc", "trajectory_executor",
                           "diagnostics.runtime_status", "diagnostics.get_runtime_diagnostics",
                           "planning.validate_motion", "profile.nrt_strict_parity", "profile.jtc_profile"});
  }
  if (normalized_backend_mode == "hybrid") {
    return makeDescriptor("hybrid",
                          "hybrid_executor",
                          "runtime",
                          "trajectory+effort",
                          "nrt_strict_parity",
                          true,
                          true,
                          true,
                          {"simulation.gazebo11", "ros2.humble", "backend.hybrid", "trajectory_executor",
                           "effort_owner", "rt.experimental", "rt.best_effort_non_controller_grade",
                           "rt.transport.ros_topic", "profile.nrt_strict_parity", "profile.hybrid_bridge",
                           "profile.rt_hardened", "diagnostics.runtime_status",
                           "diagnostics.get_runtime_diagnostics", "planning.validate_motion"});
  }
  if (normalized_backend_mode == "effort") {
    return makeDescriptor("effort",
                          "effort_owner",
                          "effort_owner",
                          "effort_controller",
                          "rt_hardened",
                          false,
                          true,
                          true,
                          {"simulation.gazebo11", "ros2.humble", "backend.effort", "effort_owner",
                           "rt.experimental", "rt.best_effort_non_controller_grade",
                           "rt.transport.shm_ring", "rt.transport.ros_topic",
                           "profile.rt_sim_experimental_best_effort", "profile.rt_hardened",
                           "profile.hard_1khz", "diagnostics.runtime_status",
                           "diagnostics.get_runtime_diagnostics", "planning.validate_motion"});
  }
  if (normalized_backend_mode == "headless_sim") {
    return makeDescriptor("headless_sim",
                          "headless_sim_provider",
                          "runtime",
                          "headless_sim_servo",
                          "rt_hardened",
                          false,
                          true,
                          true,
                          {"backend.headless_sim", "runtime.daemonized", "rt.experimental",
                           "rt.best_effort_non_controller_grade", "rt.transport.shm_ring",
                           "rt.transport.ros_topic", "profile.nrt_strict_parity",
                           "profile.rt_sim_experimental_best_effort", "profile.rt_hardened",
                           "profile.hard_1khz", "diagnostics.runtime_status",
                           "diagnostics.get_runtime_diagnostics", "planning.validate_motion"});
  }
  return makeDescriptor("unknown",
                        "unknown_provider",
                        "runtime",
                        "none",
                        "nrt_strict_parity",
                        false,
                        false,
                        false,
                        {"backend.unknown"});
}

std::vector<std::string> mergeBackendCapabilityFlags(const BackendContractDescriptor &descriptor,
                                                     const std::vector<std::string> &extra_flags) {
  std::vector<std::string> merged = descriptor.baseline_capability_flags;
  appendUnique(merged, extra_flags);
  return merged;
}

}  // namespace rokae_xmate3_ros2::runtime
