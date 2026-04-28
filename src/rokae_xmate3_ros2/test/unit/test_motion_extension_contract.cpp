#include <gtest/gtest.h>

#include "runtime/motion_extension_contract.hpp"

namespace rokae_xmate3_ros2::runtime {

TEST(MotionExtensionContract, CatalogCoversPublicMotionKinds) {
  const auto contracts = buildMotionExtensionContracts();
  ASSERT_GE(contracts.size(), 6u);
  EXPECT_NE(summarizeMotionExtensionContracts(contracts).find("MoveAbsJ"), std::string::npos);
  EXPECT_NE(summarizeMotionExtensionContracts(contracts).find("MoveSP"), std::string::npos);
  std::string error;
  EXPECT_TRUE(validateMotionExtensionContracts(contracts, error)) << error;
  EXPECT_NE(summarizeMotionExtensionContracts(contracts).find("MoveSP=surface"), std::string::npos);
  EXPECT_NE(summarizeMotionExtensionContracts(contracts).find("|public:false|experimental:true"), std::string::npos);
}

}  // namespace rokae_xmate3_ros2::runtime
