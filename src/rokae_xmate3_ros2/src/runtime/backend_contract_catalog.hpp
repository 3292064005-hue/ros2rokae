#ifndef ROKAE_XMATE3_ROS2_RUNTIME_BACKEND_CONTRACT_CATALOG_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_BACKEND_CONTRACT_CATALOG_HPP

#include <string>
#include <vector>

namespace rokae_xmate3_ros2::runtime {

struct BackendContractDescriptor {
  std::string backend_mode{"unknown"};
  std::string provider_class{"unknown_provider"};
  std::string owner_rule{"runtime"};
  std::string required_controller{"none"};
  std::string default_runtime_profile{"nrt_strict_parity"};
  std::string public_rt_policy{"best_effort_non_controller_grade"};
  std::string authority_scope{"runtime_request_coordinator"};
  std::string fidelity_class{"simulation_grade"};
  std::vector<std::string> baseline_capability_flags{};
  bool supports_trajectory_execution{false};
  bool supports_effort_owner{false};
  bool supports_rt_fast_path{false};
};

[[nodiscard]] std::string normalizeBackendModeKey(const std::string &backend_mode);
[[nodiscard]] BackendContractDescriptor describeBackendMode(const std::string &backend_mode);
[[nodiscard]] std::vector<std::string> mergeBackendCapabilityFlags(
    const BackendContractDescriptor &descriptor,
    const std::vector<std::string> &extra_flags = {});

}  // namespace rokae_xmate3_ros2::runtime

#endif
