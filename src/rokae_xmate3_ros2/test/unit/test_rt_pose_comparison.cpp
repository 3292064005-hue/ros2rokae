#include <gtest/gtest.h>

#include <array>

#include "compat/rt_motion_primitives.hpp"

TEST(RtPoseComparison, EquivalentWrappedOrientationIsAcceptedWithinTolerance) {
  const std::array<double, 6> lhs{0.1, -0.2, 0.3, 0.0, 0.0, 3.1315926535897933};
  const std::array<double, 6> rhs{0.1, -0.2, 0.3, 0.0, 0.0, -3.1315926535897933};
  EXPECT_TRUE(rokae::compat_rt::cartesianPoseWithinTolerance(lhs, rhs, 1e-6, 0.05));
}

TEST(RtPoseComparison, LargeTranslationStillFailsEvenWhenOrientationMatches) {
  const std::array<double, 6> lhs{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<double, 6> rhs{0.06, 0.0, 0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(rokae::compat_rt::cartesianPoseWithinTolerance(lhs, rhs, 0.04, 0.1));
}
