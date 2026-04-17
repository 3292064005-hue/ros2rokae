#include <gtest/gtest.h>

#include <system_error>

#include "rokae_xmate3_ros2/sdk_catalog_policy.hpp"

TEST(SdkCatalogPolicyTest, StrictModeKeepsFailureVisible) {
  rokae::ros2::SdkCatalogConsistencyPolicy policy;
  policy.strict_runtime_authority = true;
  policy.allow_legacy_catalog_fallback = false;
  EXPECT_TRUE(policy.strict_runtime_authority);
  EXPECT_FALSE(policy.allow_legacy_catalog_fallback);
}

TEST(SdkCatalogPolicyTest, LegacyFallbackFlagCanBeEnabledSeparately) {
  rokae::ros2::SdkCatalogConsistencyPolicy policy;
  policy.strict_runtime_authority = true;
  policy.allow_legacy_catalog_fallback = true;
  EXPECT_TRUE(policy.strict_runtime_authority);
  EXPECT_TRUE(policy.allow_legacy_catalog_fallback);
}


TEST(SdkCatalogPolicyTest, StrictFactoryDefaultsToAuthoritativeRuntime) {
  constexpr auto policy = rokae::ros2::strictRuntimeCatalogPolicy();
  EXPECT_TRUE(policy.strict_runtime_authority);
  EXPECT_FALSE(policy.allow_legacy_catalog_fallback);
}

TEST(SdkCatalogPolicyTest, LegacyCompatibilityFactoryKeepsFallbackEnabled) {
  constexpr auto policy = rokae::ros2::legacySdkCompatibilityCatalogPolicy();
  EXPECT_FALSE(policy.strict_runtime_authority);
  EXPECT_TRUE(policy.allow_legacy_catalog_fallback);
}
