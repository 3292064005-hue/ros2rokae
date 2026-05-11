#ifndef ROKAE_XMATE3_ROS2_RUNTIME_SERVICE_EXPOSURE_PROFILE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_SERVICE_EXPOSURE_PROFILE_HPP

#include <string>

namespace rokae_xmate3_ros2::runtime {

enum class ServiceExposureProfile {
  public_xmate_er3_only,
  public_xmate_er3_experimental,
  internal_full,
};

/**
 * @brief Converts a service exposure profile enum to the launch/config token.
 * @param profile Profile enum selected by launch or runtime parameters.
 * @return Stable profile token used by logs, diagnostics, and generated metadata.
 * @throws Never throws.
 * @details The public_xmate_er3_only profile is the default release surface: xMateER3 six-axis
 * NRT motion, state/query, toolset, kinematics, and field-tagged simulation diagnostics. RT,
 * drag, and path replay services require public_xmate_er3_experimental or internal_full.
 */
inline const char *to_string(ServiceExposureProfile profile) noexcept {
  switch (profile) {
    case ServiceExposureProfile::internal_full:
      return "internal_full";
    case ServiceExposureProfile::public_xmate_er3_experimental:
      return "public_xmate_er3_experimental";
    case ServiceExposureProfile::public_xmate_er3_only:
    default:
      return "public_xmate_er3_only";
  }
}

inline ServiceExposureProfile defaultServiceExposureProfile() noexcept {
#ifdef ROKAE_DEFAULT_SERVICE_EXPOSURE_PROFILE
  constexpr const char *kDefaultProfile = ROKAE_DEFAULT_SERVICE_EXPOSURE_PROFILE;
  const std::string profile{kDefaultProfile};
  if (profile == "internal_full") {
    return ServiceExposureProfile::internal_full;
  }
  if (profile == "public_xmate_er3_experimental" || profile == "experimental") {
    return ServiceExposureProfile::public_xmate_er3_experimental;
  }
  return ServiceExposureProfile::public_xmate_er3_only;
#else
  return ServiceExposureProfile::public_xmate_er3_only;
#endif
}

/**
 * @brief Parses a service exposure profile token from launch parameters.
 * @param profile_name Raw launch/runtime parameter value.
 * @return Parsed profile; unknown values intentionally fall back to public_xmate_er3_only.
 * @throws Never throws.
 * @details Boundary behavior is conservative: mistyped profile names cannot accidentally expose
 * internal, RT, drag, or path replay services.
 */
inline ServiceExposureProfile parseServiceExposureProfile(const std::string &profile_name) noexcept {
  if (profile_name == "internal_full" || profile_name == "full" || profile_name == "internal") {
    return ServiceExposureProfile::internal_full;
  }
  if (profile_name == "public_xmate_er3_experimental" || profile_name == "experimental" ||
      profile_name == "experimental_rt") {
    return ServiceExposureProfile::public_xmate_er3_experimental;
  }
  return ServiceExposureProfile::public_xmate_er3_only;
}

inline bool includesExperimentalServices(ServiceExposureProfile profile) noexcept {
  return profile == ServiceExposureProfile::public_xmate_er3_experimental ||
         profile == ServiceExposureProfile::internal_full;
}

}  // namespace rokae_xmate3_ros2::runtime

#endif
