#include <gtest/gtest.h>

#include "runtime/runtime_catalog_service.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

TEST(RuntimeOptionCatalogTest, SummarizesMotionAndSessionOptions) {
  MotionOptionsState motion_options;
  SessionState session_state;
  motion_options.setDefaultSpeed(120.0);
  motion_options.setDefaultZone(20);
  motion_options.setDefaultConfOpt(true);
  motion_options.setAvoidSingularity(false);
  motion_options.setSoftLimit(true, rokae_xmate3_ros2::spec::xmate3::kDefaultSoftLimits);
  session_state.setSimulationMode(false);
  session_state.setMotionMode(2);
  session_state.setRtControlMode(1);

  const auto entries = buildRuntimeOptionCatalog(motion_options, session_state);
  EXPECT_GE(entries.size(), 8u);
  const auto summary = summarizeRuntimeOptionCatalog(entries);
  EXPECT_NE(summary.find("default_speed=120.000"), std::string::npos);
  EXPECT_NE(summary.find("simulation_mode=false"), std::string::npos);
  EXPECT_NE(summary.find("rt_control_mode=1"), std::string::npos);
}

}  // namespace
}  // namespace rokae_xmate3_ros2::runtime
