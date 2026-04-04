#include <gtest/gtest.h>

#include "runtime/data_store_state.hpp"
#include "rokae_xmate3_ros2/runtime/rt_semantic_topics.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

TEST(CompatStateDataTest, FilterLimitAndCollisionThresholdTopicsPopulateTypedSnapshot) {
  rt::DataStoreState state;
  state.setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kConfigFilterLimit,
                      "enabled=0;cutoff_frequency=42.0");
  state.setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kConfigCollisionBehaviourThresholds,
                      "10,20,30,40,50,60");

  const auto snapshot = state.rtControlSnapshot();
  EXPECT_TRUE(snapshot.filter_limit_configured);
  EXPECT_FALSE(snapshot.filter_limit_enabled);
  EXPECT_DOUBLE_EQ(snapshot.filter_limit_cutoff_frequency, 42.0);
  EXPECT_TRUE(snapshot.collision_thresholds_configured);
  EXPECT_DOUBLE_EQ(snapshot.collision_thresholds[0], 10.0);
  EXPECT_DOUBLE_EQ(snapshot.collision_thresholds[5], 60.0);
}

TEST(CompatStateDataTest, ForceControlFrameAcceptsValuesFieldPayload) {
  rt::DataStoreState state;
  state.setCustomData(
      rokae_xmate3_ros2::runtime::rt_topics::kConfigForceControlFrame,
      "type=0;values=1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1");

  const auto snapshot = state.rtControlSnapshot();
  EXPECT_TRUE(snapshot.force_control_frame.configured);
  EXPECT_EQ(snapshot.force_control_frame.type, rokae::FrameType::world);
  EXPECT_DOUBLE_EQ(snapshot.force_control_frame.frame[0], 1.0);
  EXPECT_DOUBLE_EQ(snapshot.force_control_frame.frame[15], 1.0);
}
