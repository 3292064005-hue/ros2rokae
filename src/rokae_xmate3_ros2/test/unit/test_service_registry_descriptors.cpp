#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "runtime/service_registration.hpp"

using rokae_xmate3_ros2::runtime::ServiceRegistrationDescriptor;
using rokae_xmate3_ros2::runtime::validateServiceDescriptors;

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
