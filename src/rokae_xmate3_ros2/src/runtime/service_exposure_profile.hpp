#ifndef ROKAE_XMATE3_ROS2_RUNTIME_SERVICE_EXPOSURE_PROFILE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_SERVICE_EXPOSURE_PROFILE_HPP

#include <string>

namespace rokae_xmate3_ros2::runtime {

enum class ServiceExposureProfile {
  public_xmate6_only,
  internal_full,
};

inline const char *to_string(ServiceExposureProfile profile) noexcept {
  switch (profile) {
    case ServiceExposureProfile::internal_full:
      return "internal_full";
    case ServiceExposureProfile::public_xmate6_only:
    default:
      return "public_xmate6_only";
  }
}

inline ServiceExposureProfile defaultServiceExposureProfile() noexcept {
#ifdef ROKAE_DEFAULT_SERVICE_EXPOSURE_PROFILE
  constexpr const char *kDefaultProfile = ROKAE_DEFAULT_SERVICE_EXPOSURE_PROFILE;
  return (std::string{kDefaultProfile} == "internal_full") ? ServiceExposureProfile::internal_full
                                                            : ServiceExposureProfile::public_xmate6_only;
#else
  return ServiceExposureProfile::public_xmate6_only;
#endif
}

inline ServiceExposureProfile parseServiceExposureProfile(const std::string &profile_name) noexcept {
  if (profile_name == "internal_full" || profile_name == "full" || profile_name == "internal") {
    return ServiceExposureProfile::internal_full;
  }
  return ServiceExposureProfile::public_xmate6_only;
}

}  // namespace rokae_xmate3_ros2::runtime

#endif
