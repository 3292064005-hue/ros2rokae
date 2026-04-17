#ifndef ROKAE_XMATE3_ROS2_RUNTIME_ROS_SERVICE_REGISTRY_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_ROS_SERVICE_REGISTRY_HPP

#include <vector>

#include "runtime/service_exposure_profile.hpp"
#include "runtime/service_registration.hpp"

namespace rokae_xmate3_ros2::runtime {

[[nodiscard]] std::vector<ServiceRegistrationDescriptor> buildPrimaryServiceDescriptors(ServiceExposureProfile profile);
[[nodiscard]] std::vector<ServiceRegistrationDescriptor> buildCompatibilityAliasDescriptors(ServiceExposureProfile profile);

}  // namespace rokae_xmate3_ros2::runtime

#endif
