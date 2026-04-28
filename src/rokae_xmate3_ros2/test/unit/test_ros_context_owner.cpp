#include <gtest/gtest.h>

#include "rokae_xmate3_ros2/runtime/ros_context_owner.hpp"

TEST(RosContextOwnerTest, TracksLeaseCountWithoutAutoShutdown) {
  const auto initial = rokae_xmate3_ros2::runtime::RosContextOwner::activeLeaseCount();
  auto lease_a = rokae_xmate3_ros2::runtime::RosContextOwner::acquire("test_a");
  auto lease_b = rokae_xmate3_ros2::runtime::RosContextOwner::acquire("test_b");
  EXPECT_EQ(rokae_xmate3_ros2::runtime::RosContextOwner::activeLeaseCount(), initial + 2u);
  lease_a.reset();
  EXPECT_EQ(rokae_xmate3_ros2::runtime::RosContextOwner::activeLeaseCount(), initial + 1u);
  lease_b.reset();
  EXPECT_EQ(rokae_xmate3_ros2::runtime::RosContextOwner::activeLeaseCount(), initial);
}
