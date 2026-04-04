#include <gtest/gtest.h>

#include "runtime/runtime_catalog_service.hpp"
#include "rokae_xmate3_ros2/runtime/rt_semantic_topics.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

TEST(RuntimeOptionCatalogTest, SummarizesMotionAndSessionOptions) {
  MotionOptionsState motion_options;
  SessionState session_state;
  DataStoreState data_store;
  motion_options.setDefaultSpeed(120.0);
  motion_options.setDefaultZone(20);
  motion_options.setDefaultConfOpt(true);
  motion_options.setAvoidSingularity(false);
  motion_options.setSoftLimit(true, rokae_xmate3_ros2::spec::xmate3::kDefaultSoftLimits);
  session_state.setSimulationMode(false);
  session_state.setMotionMode(2);
  session_state.setRtControlMode(1);
  data_store.setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kConfigFilterLimit,
                           "enabled=0;cutoff_frequency=42.0");
  data_store.setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kConfigCollisionBehaviourThresholds,
                           "1,2,3,4,5,6");

  const auto entries = buildRuntimeOptionCatalog(motion_options, session_state, data_store);
  EXPECT_GE(entries.size(), 8u);
  const auto summary = summarizeRuntimeOptionCatalog(entries);
  EXPECT_NE(summary.find("default_speed=120.000"), std::string::npos);
  EXPECT_NE(summary.find("simulation_mode=false"), std::string::npos);
  EXPECT_NE(summary.find("rt_control_mode=1"), std::string::npos);
  EXPECT_NE(summary.find("filter_limit=enabled=false,cutoff=42.000"), std::string::npos);
  EXPECT_NE(summary.find("collision_behaviour_thresholds=1.000,2.000,3.000,4.000,5.000,6.000"), std::string::npos);
}

}  // namespace
}  // namespace rokae_xmate3_ros2::runtime
