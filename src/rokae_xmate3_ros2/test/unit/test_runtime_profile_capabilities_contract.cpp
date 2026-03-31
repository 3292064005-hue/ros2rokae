#include <gtest/gtest.h>

#include "runtime/runtime_catalog_service.hpp"
#include "runtime/runtime_profile_service.hpp"
#include "runtime/planning_capability_service.hpp"
#include "runtime/session_state.hpp"
#include "runtime/motion_options_state.hpp"

namespace rokae_xmate3_ros2::runtime {

TEST(RuntimeProfileCapabilitiesContract, SummariesStayStructured) {
  SessionState session;
  MotionOptionsState options;
  session.setSimulationMode(true);
  session.setMotionMode(3);
  session.setRtControlMode(1);
  options.setDefaultSpeed(250.0);
  options.setDefaultZone(20);
  const auto profiles = buildRuntimeProfileCatalog("hybrid", "rt_simulated", {"rt.experimental", "trajectory_executor"});
  const auto summary = summarizeRuntimeProfileCatalog(profiles);
  const auto option_entries = buildRuntimeOptionCatalog(options, session);
  const auto option_summary = summarizeRuntimeOptionCatalog(option_entries);
  const auto planning_summary = summarizePlanningCapabilityCatalog(
      buildKinematicsBackendCatalog("kdl"),
      buildRetimerPolicyCatalog("nominal"),
      buildPlannerSelectionCatalog("risk_weighted"));
  EXPECT_NE(summary.find("rt_simulated=active"), std::string::npos);
  EXPECT_NE(option_summary.find("default_speed=250.000"), std::string::npos);
  EXPECT_NE(option_summary.find("simulation_mode=true"), std::string::npos);
  EXPECT_NE(planning_summary.find("kinematics[kdl=active"), std::string::npos);
}

}  // namespace rokae_xmate3_ros2::runtime
