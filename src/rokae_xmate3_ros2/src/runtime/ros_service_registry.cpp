#include "runtime/ros_bindings.hpp"
#include "runtime/ros_service_registry.hpp"

#include <stdexcept>
#include <vector>

#include "runtime/service_contract_manifest.hpp"
#include "runtime/service_registration.hpp"

namespace rokae_xmate3_ros2::runtime {

std::vector<ServiceRegistrationDescriptor> buildPrimaryServiceDescriptors() {
  std::vector<ServiceRegistrationDescriptor> descriptors;
#define ROKAE_ADD_PRIMARY(ServiceT, domain, name, facade_accessor, method) \
  descriptors.push_back(makeServiceRegistrationDescriptor<ServiceT>(domain, name, false, facade_accessor, method));
  ROKAE_PRIMARY_SERVICE_CONTRACTS(ROKAE_ADD_PRIMARY)
#undef ROKAE_ADD_PRIMARY
  return descriptors;
}

void RosBindings::initServices() {
  const auto descriptors = buildPrimaryServiceDescriptors();
  const auto aliases = buildCompatibilityAliasDescriptors();
  std::string error_message;
  if (!validateServiceDescriptorSets(descriptors, aliases, error_message)) {
    throw std::runtime_error(error_message);
  }
  services_.reserve(descriptors.size());
  for (const auto &descriptor : descriptors) {
    services_.push_back(descriptor.create(*this));
  }
}

}  // namespace rokae_xmate3_ros2::runtime
