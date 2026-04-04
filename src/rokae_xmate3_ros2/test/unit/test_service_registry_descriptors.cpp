#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "runtime/ros_service_registry.hpp"
#include "runtime/service_registration.hpp"

using rokae_xmate3_ros2::runtime::ServiceRegistrationDescriptor;
using rokae_xmate3_ros2::runtime::buildCompatibilityAliasDescriptors;
using rokae_xmate3_ros2::runtime::buildPrimaryServiceDescriptors;
using rokae_xmate3_ros2::runtime::validateServiceDescriptors;
using rokae_xmate3_ros2::runtime::validateServiceDescriptorSets;

TEST(ServiceRegistrationDescriptorTest, RejectsDuplicateNames) {
  std::vector<ServiceRegistrationDescriptor> descriptors{
      {"query", "/xmate3/test", false, {}},
      {"compatibility", "/xmate3/test", true, {}}};
  std::string error;
  EXPECT_FALSE(validateServiceDescriptors(descriptors, error));
  EXPECT_NE(error.find("duplicate service descriptor"), std::string::npos);
}

TEST(ServiceRegistrationDescriptorTest, AcceptsUniqueNames) {
  std::vector<ServiceRegistrationDescriptor> descriptors{
      {"query", "/xmate3/test_a", false, {}},
      {"compatibility", "/xmate3/test_b", true, {}}};
  std::string error;
  EXPECT_TRUE(validateServiceDescriptors(descriptors, error));
  EXPECT_TRUE(error.empty());
}

TEST(ServiceRegistrationDescriptorTest, RejectsEmptyNames) {
  std::vector<ServiceRegistrationDescriptor> descriptors{
      {"query", "", false, {}}};
  std::string error;
  EXPECT_FALSE(validateServiceDescriptors(descriptors, error));
  EXPECT_NE(error.find("empty name"), std::string::npos);
}

TEST(ServiceRegistrationDescriptorTest, PublishesControlQueryManifestForPrimaryAndCompatibilitySurfaces) {
  auto primary = buildPrimaryServiceDescriptors();
  auto aliases = buildCompatibilityAliasDescriptors();
  ASSERT_FALSE(primary.empty());
  ASSERT_FALSE(aliases.empty());

  std::string error;
  EXPECT_TRUE(validateServiceDescriptors(primary, error)) << error;
  error.clear();
  EXPECT_TRUE(validateServiceDescriptors(aliases, error)) << error;

  const auto set_toolset = std::find_if(primary.begin(), primary.end(), [](const auto &descriptor) {
    return std::string(descriptor.name) == "/xmate3/cobot/set_toolset";
  });
  ASSERT_NE(set_toolset, primary.end());
  EXPECT_STREQ(set_toolset->domain, "control");
  EXPECT_FALSE(set_toolset->compatibility_alias);

  const auto get_toolset = std::find_if(primary.begin(), primary.end(), [](const auto &descriptor) {
    return std::string(descriptor.name) == "/xmate3/cobot/get_toolset";
  });
  ASSERT_NE(get_toolset, primary.end());
  EXPECT_STREQ(get_toolset->domain, "query");

  const auto runtime_snapshot = std::find_if(primary.begin(), primary.end(), [](const auto &descriptor) {
    return std::string(descriptor.name) == "/xmate3/internal/get_runtime_state_snapshot";
  });
  ASSERT_NE(runtime_snapshot, primary.end());
  EXPECT_STREQ(runtime_snapshot->domain, "query");

  const auto sim_alias = std::find_if(aliases.begin(), aliases.end(), [](const auto &descriptor) {
    return std::string(descriptor.name) == "/xmate3/cobot/set_simulation_mode";
  });
  ASSERT_NE(sim_alias, aliases.end());
  EXPECT_TRUE(sim_alias->compatibility_alias);
  EXPECT_STREQ(sim_alias->domain, "compatibility");
}


TEST(ServiceRegistrationDescriptorTest, RejectsCrossSurfaceDuplicateNames) {
  std::vector<ServiceRegistrationDescriptor> primary{{"query", "/xmate3/test_shared", false, {}}};
  std::vector<ServiceRegistrationDescriptor> aliases{{"compatibility", "/xmate3/test_shared", true, {}}};
  std::string error;
  EXPECT_FALSE(validateServiceDescriptorSets(primary, aliases, error));
  EXPECT_NE(error.find("duplicate service descriptor"), std::string::npos);
}
