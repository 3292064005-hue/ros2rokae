#include "runtime/ros_bindings.hpp"
#include "runtime/ros_service_registry.hpp"

#include <stdexcept>
#include <vector>

#include "runtime/service_contract_manifest.hpp"
#include "runtime/service_registration.hpp"

namespace rokae_xmate3_ros2::runtime {

std::vector<ServiceRegistrationDescriptor> buildPrimaryServiceDescriptors(ServiceExposureProfile profile) {
  auto descriptors = buildPublicPrimaryServiceContractManifest();
#if ROKAE_ENABLE_INTERNAL_SURFACE
  if (profile == ServiceExposureProfile::internal_full) {
    auto internal = buildInternalPrimaryServiceContractManifest();
    descriptors.insert(descriptors.end(), internal.begin(), internal.end());
  }
#else
  (void)profile;
#endif
  return descriptors;
}

void RosBindings::initServices() {
  const auto descriptors = buildPrimaryServiceDescriptors(service_exposure_profile_);
  const auto aliases = buildCompatibilityAliasDescriptors(service_exposure_profile_);
  const bool register_primary = publishesCanonicalAliases(compatibility_alias_policy_);
  const bool register_aliases = publishesCompatibilityAliases(compatibility_alias_policy_);
  std::string error_message;
  if (!validateServiceDescriptorSets(register_primary ? descriptors : std::vector<ServiceRegistrationDescriptor>{},
                                     register_aliases ? aliases : std::vector<ServiceRegistrationDescriptor>{},
                                     error_message)) {
    throw std::runtime_error(error_message);
  }
  if (!register_primary) {
    return;
  }
  services_.reserve(descriptors.size());
  for (const auto &descriptor : descriptors) {
    services_.push_back(descriptor.create(*this));
  }
}

}  // namespace rokae_xmate3_ros2::runtime
