#ifndef ROKAE_XMATE3_ROS2_RUNTIME_SERVICE_REGISTRATION_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_SERVICE_REGISTRATION_HPP

#include <functional>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "runtime/ros_bindings.hpp"
#include "runtime/ros_service_factory.hpp"

namespace rokae_xmate3_ros2::runtime {

struct ServiceRegistrationDescriptor {
  const char *domain = "unknown";
  const char *name = "";
  bool compatibility_alias = false;
  std::function<rclcpp::ServiceBase::SharedPtr(RosBindings &)> create;
};

template <typename ServiceT, typename FacadeT>
ServiceRegistrationDescriptor makeServiceRegistrationDescriptor(
    const char *domain,
    const char *name,
    bool compatibility_alias,
    FacadeT *(RosBindings::*facade_accessor)() const,
    void (FacadeT::*method)(const typename ServiceT::Request &, typename ServiceT::Response &) const) {
  return ServiceRegistrationDescriptor{
      domain,
      name,
      compatibility_alias,
      [facade_accessor, method, name](RosBindings &bindings) {
        auto *facade = (bindings.*facade_accessor)();
        return detail::CreateFacadeService<ServiceT>(bindings.node(), name, facade, method);
      }};
}

inline bool validateServiceDescriptors(const std::vector<ServiceRegistrationDescriptor> &descriptors,
                                       std::string &error_message) {
  std::unordered_set<std::string> seen;
  for (const auto &descriptor : descriptors) {
    if (descriptor.name == nullptr || *descriptor.name == '\0') {
      error_message = "service descriptor with empty name";
      return false;
    }
    const std::string name(descriptor.name);
    if (!seen.insert(name).second) {
      error_message = "duplicate service descriptor: " + name;
      return false;
    }
  }
  return true;
}

}  // namespace rokae_xmate3_ros2::runtime

#endif
