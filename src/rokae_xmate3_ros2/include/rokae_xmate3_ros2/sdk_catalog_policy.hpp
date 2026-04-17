#ifndef ROKAE_XMATE3_ROS2_SDK_CATALOG_POLICY_HPP
#define ROKAE_XMATE3_ROS2_SDK_CATALOG_POLICY_HPP

namespace rokae::ros2 {

struct SdkCatalogConsistencyPolicy {
  bool strict_runtime_authority = true;
  bool allow_legacy_catalog_fallback = false;
};

constexpr SdkCatalogConsistencyPolicy strictRuntimeCatalogPolicy() noexcept {
  return SdkCatalogConsistencyPolicy{true, false};
}

constexpr SdkCatalogConsistencyPolicy legacySdkCompatibilityCatalogPolicy() noexcept {
  return SdkCatalogConsistencyPolicy{false, true};
}

}  // namespace rokae::ros2

#endif
