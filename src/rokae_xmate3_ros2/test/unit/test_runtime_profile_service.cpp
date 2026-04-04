#include <gtest/gtest.h>

#include "runtime/runtime_profile_service.hpp"
#include "runtime/planning_capability_service.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

TEST(RuntimeProfileServiceTest, ActiveProfileAndSummaryAreStable) {
  const auto profiles = buildRuntimeProfileCatalog("hybrid", "nrt_strict_parity",
                                                   {"rt.experimental", "trajectory_executor", "effort_owner"});
  ASSERT_FALSE(profiles.empty());
  EXPECT_TRUE(profiles.front().active);
  const auto summary = summarizeRuntimeProfileCatalog(profiles);
  EXPECT_NE(summary.find("nrt_strict_parity=active"), std::string::npos);
  EXPECT_NE(summary.find("hybrid_bridge"), std::string::npos);
  EXPECT_NE(summary.find("rt_sim_experimental_best_effort"), std::string::npos);
  EXPECT_FALSE(profiles.front().owner_rule.empty());
  EXPECT_FALSE(profiles.front().preferred_contract.empty());
}

}  // namespace
}  // namespace rokae_xmate3_ros2::runtime

namespace rokae_xmate3_ros2::runtime {
namespace {

TEST(RuntimeProfileServiceTest, PlanningCapabilityCatalogsStayStructured) {
  const auto kinematics = buildKinematicsBackendCatalog("kdl");
  const auto retimers = buildRetimerPolicyCatalog("nominal");
  const auto planners = buildPlannerSelectionCatalog("risk_weighted");
  ASSERT_FALSE(kinematics.empty());
  ASSERT_FALSE(retimers.empty());
  ASSERT_FALSE(planners.empty());
  EXPECT_TRUE(kinematics.front().active);
  const auto summary = summarizePlanningCapabilityCatalog(kinematics, retimers, planners);
  EXPECT_NE(summary.find("kinematics[kdl=active"), std::string::npos);
  EXPECT_NE(summary.find("retimer[nominal=active"), std::string::npos);
  EXPECT_NE(summary.find("planner[risk_weighted=active"), std::string::npos);
}

}  // namespace
}  // namespace rokae_xmate3_ros2::runtime
