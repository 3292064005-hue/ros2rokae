#include "runtime/ros_bindings.hpp"
#include "runtime/ros_service_registry.hpp"

#include <stdexcept>
#include <vector>

#include "runtime/service_contract_manifest.hpp"
#include "runtime/service_registration.hpp"

namespace rokae_xmate3_ros2::runtime {

std::vector<ServiceRegistrationDescriptor> buildCompatibilityAliasDescriptors(ServiceExposureProfile profile) {
  auto descriptors = buildPublicCompatibilityAliasContractManifest();
#if ROKAE_ENABLE_INTERNAL_SURFACE
  if (profile == ServiceExposureProfile::internal_full) {
    auto internal = buildInternalCompatibilityAliasContractManifest();
    descriptors.insert(descriptors.end(), internal.begin(), internal.end());
  }
#else
  (void)profile;
#endif
  return descriptors;
}

void RosBindings::registerCompatibilityAliases() {
  if (!publishesCompatibilityAliases(compatibility_alias_policy_)) {
    compatibility_services_.clear();
    return;
  }
  const auto descriptors = buildCompatibilityAliasDescriptors(service_exposure_profile_);
  const auto primary = publishesCanonicalAliases(compatibility_alias_policy_)
                           ? buildPrimaryServiceDescriptors(service_exposure_profile_)
                           : std::vector<ServiceRegistrationDescriptor>{};
  std::string error_message;
  if (!validateServiceDescriptorSets(primary, descriptors, error_message)) {
    throw std::runtime_error(error_message);
  }
  compatibility_services_.reserve(descriptors.size());
  for (const auto &descriptor : descriptors) {
    compatibility_services_.push_back(descriptor.create(*this));
  }
}

}  // namespace rokae_xmate3_ros2::runtime
