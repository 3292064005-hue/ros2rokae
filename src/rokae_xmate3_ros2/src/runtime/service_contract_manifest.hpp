#ifndef ROKAE_XMATE3_ROS2_RUNTIME_SERVICE_CONTRACT_MANIFEST_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_SERVICE_CONTRACT_MANIFEST_HPP

#include <vector>

#include "runtime/service_registration.hpp"

namespace rokae_xmate3_ros2::runtime {

[[nodiscard]] std::vector<ServiceRegistrationDescriptor> buildPublicPrimaryServiceContractManifest();
[[nodiscard]] std::vector<ServiceRegistrationDescriptor> buildPublicCompatibilityAliasContractManifest();
#if ROKAE_ENABLE_INTERNAL_SURFACE
[[nodiscard]] std::vector<ServiceRegistrationDescriptor> buildInternalPrimaryServiceContractManifest();
[[nodiscard]] std::vector<ServiceRegistrationDescriptor> buildInternalCompatibilityAliasContractManifest();
#endif

}  // namespace rokae_xmate3_ros2::runtime

#endif
