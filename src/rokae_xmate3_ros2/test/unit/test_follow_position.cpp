#include <gtest/gtest.h>

#include <array>

#include <Eigen/Geometry>

#include "rokae/exception.h"
#include "rokae/planner.h"

TEST(FollowPositionTest, StartRequiresInitializationAndStopIsIdempotent) {
  rokae::FollowPosition<6> follow;
  EXPECT_THROW(follow.start(std::array<double, 6>{}), rokae::RealtimeControlException);
  EXPECT_THROW(follow.start(Eigen::Transform<double, 3, Eigen::Isometry>::Identity()), rokae::RealtimeControlException);
  EXPECT_NO_THROW(follow.stop());
  EXPECT_NO_THROW(follow.stop());
}
