#ifndef ROKAE_XMATE3_ROS2_RUNTIME_COMPATIBILITY_ALIAS_POLICY_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_COMPATIBILITY_ALIAS_POLICY_HPP

#include <stdexcept>
#include <string>

namespace rokae_xmate3_ros2::runtime {

enum class CompatibilityAliasPolicy {
  canonical_plus_compat,
  canonical_only,
  legacy_only,
};

inline const char *to_string(CompatibilityAliasPolicy policy) noexcept {
  switch (policy) {
    case CompatibilityAliasPolicy::canonical_only:
      return "canonical_only";
    case CompatibilityAliasPolicy::legacy_only:
      return "legacy_only";
    case CompatibilityAliasPolicy::canonical_plus_compat:
    default:
      return "canonical_plus_compat";
  }
}

inline CompatibilityAliasPolicy parseCompatibilityAliasPolicy(const std::string &policy_name) {
  if (policy_name == "canonical_only" || policy_name == "canonical") {
    return CompatibilityAliasPolicy::canonical_only;
  }
  if (policy_name == "legacy_only" || policy_name == "compatibility_only" || policy_name == "legacy") {
    return CompatibilityAliasPolicy::legacy_only;
  }
  if (policy_name.empty() || policy_name == "default") {
    return CompatibilityAliasPolicy::canonical_only;
  }
  if (policy_name == "canonical_plus_compat" || policy_name == "canonical+compat") {
    return CompatibilityAliasPolicy::canonical_plus_compat;
  }
  throw std::invalid_argument("unsupported compatibility alias policy: " + policy_name);
}

inline CompatibilityAliasPolicy defaultCompatibilityAliasPolicy() {
#ifdef ROKAE_DEFAULT_COMPATIBILITY_ALIAS_POLICY
  constexpr const char *kDefaultPolicy = ROKAE_DEFAULT_COMPATIBILITY_ALIAS_POLICY;
  return parseCompatibilityAliasPolicy(kDefaultPolicy);
#else
  return CompatibilityAliasPolicy::canonical_only;
#endif
}

inline bool publishesCanonicalAliases(CompatibilityAliasPolicy policy) noexcept {
  return policy != CompatibilityAliasPolicy::legacy_only;
}

inline bool publishesCompatibilityAliases(CompatibilityAliasPolicy policy) noexcept {
  return policy != CompatibilityAliasPolicy::canonical_only;
}

}  // namespace rokae_xmate3_ros2::runtime

#endif
