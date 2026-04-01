#include "runtime/ros_bindings.hpp"

#include <stdexcept>
#include <vector>

#include "runtime/service_registration.hpp"

namespace rokae_xmate3_ros2::runtime {

namespace {
std::vector<ServiceRegistrationDescriptor> makeCompatibilityAliasDescriptors() {
  std::vector<ServiceRegistrationDescriptor> descriptors;
#define ROKAE_ADD_ALIAS(ServiceT, domain, name, facade_accessor, method)   descriptors.push_back(makeServiceRegistrationDescriptor<ServiceT>(domain, name, true, facade_accessor, method))
  ROKAE_ADD_ALIAS(rokae_xmate3_ros2::srv::GetDI, "compatibility", "/xmate3/cobot/get_di", &RosBindings::ioProgramFacade, &IoProgramFacade::handleGetDI);
  ROKAE_ADD_ALIAS(rokae_xmate3_ros2::srv::GetDO, "compatibility", "/xmate3/cobot/get_do", &RosBindings::ioProgramFacade, &IoProgramFacade::handleGetDO);
  ROKAE_ADD_ALIAS(rokae_xmate3_ros2::srv::SetDI, "compatibility", "/xmate3/cobot/set_di", &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetDI);
  ROKAE_ADD_ALIAS(rokae_xmate3_ros2::srv::SetDO, "compatibility", "/xmate3/cobot/set_do", &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetDO);
  ROKAE_ADD_ALIAS(rokae_xmate3_ros2::srv::GetAI, "compatibility", "/xmate3/cobot/get_ai", &RosBindings::ioProgramFacade, &IoProgramFacade::handleGetAI);
  ROKAE_ADD_ALIAS(rokae_xmate3_ros2::srv::SetAO, "compatibility", "/xmate3/cobot/set_ao", &RosBindings::ioProgramFacade, &IoProgramFacade::handleSetAO);
  ROKAE_ADD_ALIAS(rokae_xmate3_ros2::srv::SetSimulationMode, "compatibility", "/xmate3/cobot/set_simulation_mode", &RosBindings::controlFacade, &ControlFacade::handleSetSimulationMode);
#undef ROKAE_ADD_ALIAS
  return descriptors;
}
}  // namespace

void RosBindings::registerCompatibilityAliases() {
  const auto descriptors = makeCompatibilityAliasDescriptors();
  std::string error_message;
  if (!validateServiceDescriptors(descriptors, error_message)) {
    throw std::runtime_error(error_message);
  }
  compatibility_services_.reserve(descriptors.size());
  for (const auto &descriptor : descriptors) {
    compatibility_services_.push_back(descriptor.create(*this));
  }
}

}  // namespace rokae_xmate3_ros2::runtime
