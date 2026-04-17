#include <gtest/gtest.h>

#include <array>
#include <system_error>

#include "compat/rt_motion_primitives.hpp"

TEST(RtMoveCGeometry, FitCircularArcRejectsCollinearTriples) {
  const std::array<double, 6> start{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<double, 6> aux{0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<double, 6> target{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::error_code ec;
  const auto geometry = rokae::compat_rt::fitCircularArc(start, aux, target, ec);
  EXPECT_FALSE(geometry.has_value());
  EXPECT_EQ(ec, std::make_error_code(std::errc::invalid_argument));
}

TEST(RtMoveCGeometry, InterpolationDensityScalesWithRadiusAndSpeed) {
  const int small_radius_steps = rokae::compat_rt::estimateArcInterpolationSteps(0.3, 0.10, 1.0);
  const int large_radius_steps = rokae::compat_rt::estimateArcInterpolationSteps(0.3, 1.50, 1.0);
  const int slow_steps = rokae::compat_rt::estimateArcInterpolationSteps(0.1, 0.50, 1.0);
  const int fast_steps = rokae::compat_rt::estimateArcInterpolationSteps(1.0, 0.50, 1.0);
  EXPECT_GT(large_radius_steps, small_radius_steps);
  EXPECT_GT(slow_steps, 0);
  EXPECT_GT(fast_steps, 0);
}
