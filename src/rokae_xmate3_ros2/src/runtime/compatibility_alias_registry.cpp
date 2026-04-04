#include "runtime/ros_bindings.hpp"
#include "runtime/ros_service_registry.hpp"

#include <stdexcept>
#include <vector>

#include "runtime/service_contract_manifest.hpp"
#include "runtime/service_registration.hpp"

namespace rokae_xmate3_ros2::runtime {

std::vector<ServiceRegistrationDescriptor> buildCompatibilityAliasDescriptors() {
  std::vector<ServiceRegistrationDescriptor> descriptors;
#define ROKAE_ADD_ALIAS(ServiceT, domain, name, facade_accessor, method) \
  descriptors.push_back(makeServiceRegistrationDescriptor<ServiceT>(domain, name, true, facade_accessor, method));
  ROKAE_COMPATIBILITY_ALIAS_CONTRACTS(ROKAE_ADD_ALIAS)
#undef ROKAE_ADD_ALIAS
  return descriptors;
}

void RosBindings::registerCompatibilityAliases() {
  const auto descriptors = buildCompatibilityAliasDescriptors();
  const auto primary = buildPrimaryServiceDescriptors();
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
