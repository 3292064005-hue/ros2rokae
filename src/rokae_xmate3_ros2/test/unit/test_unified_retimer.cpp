#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "runtime/unified_retimer.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

// ============================================================================
// Basic Retiming Tests
// ============================================================================

TEST(UnifiedRetimerTest, RetimePointToPointProducesNonEmptyResult) {
  const std::vector<double> start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> target = {0.3, 0.2, 0.5, -0.1, 0.4, -0.2};

  const auto result = rt::retimeJointWithUnifiedConfig(
      start, target, 0.01, 1.0, 2.0, 0.1);

  ASSERT_FALSE(result.empty()) << result.samples.error_message;
  EXPECT_GT(result.samples.total_time, 0.0);
  EXPECT_NEAR(result.samples.sample_dt, 0.01, 1e-9);
  EXPECT_EQ(result.samples.positions.front(), start);
  EXPECT_EQ(result.samples.positions.back(), target);
}

TEST(UnifiedRetimerTest, RetimeEndpointVelocitiesAreZero) {
  const std::vector<double> start = {0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
  const std::vector<double> target = {0.06, 0.20, 1.48, 0.02, 1.28, 3.08};

  const auto result = rt::retimeJointWithUnifiedConfig(
      start, target, 0.01, 1.0, 2.0, 0.1);

  ASSERT_FALSE(result.empty()) << result.samples.error_message;
  ASSERT_FALSE(result.samples.velocities.empty());

  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(result.samples.velocities.front()[i], 0.0, 1e-3)
        << "Non-zero start velocity at joint " << i;
    EXPECT_NEAR(result.samples.velocities.back()[i], 0.0, 1e-3)
        << "Non-zero end velocity at joint " << i;
  }
}

TEST(UnifiedRetimerTest, RetimeIdenticalStartTargetIsDegenerate) {
  const std::vector<double> same = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

  const auto result = rt::retimeJointWithUnifiedConfig(
      same, same, 0.01, 1.0, 2.0, 0.1);

  // Should succeed, with minimal or single-point trajectory
  ASSERT_FALSE(result.empty()) << result.samples.error_message;
  EXPECT_LE(result.samples.positions.size(), 2u);
}

// ============================================================================
// Metadata Tests
// ============================================================================

TEST(UnifiedRetimerTest, MetadataSourceFamilyIsCorrect) {
  const std::vector<double> start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> target = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

  const auto result = rt::retimeJointWithUnifiedConfig(
      start, target, 0.01, 1.0, 2.0, 0.1, rt::RetimerSourceFamily::joint);

  ASSERT_FALSE(result.empty());
  EXPECT_EQ(result.metadata.source_family, rt::RetimerSourceFamily::joint);
}

TEST(UnifiedRetimerTest, MetadataDurationMatchesSamples) {
  const std::vector<double> start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> target = {0.5, 0.3, 0.8, -0.2, 0.6, 1.0};

  const auto result = rt::retimeJointWithUnifiedConfig(
      start, target, 0.01, 1.0, 2.0, 0.1);

  ASSERT_FALSE(result.empty());
  EXPECT_DOUBLE_EQ(result.metadata.total_duration, result.samples.total_time);
  EXPECT_DOUBLE_EQ(result.metadata.sample_dt, result.samples.sample_dt);
}

// ============================================================================
// Path Retiming Tests
// ============================================================================

TEST(UnifiedRetimerTest, PathRetimingMultipleWaypointsPreservesEndpoints) {
  const std::vector<std::vector<double>> waypoints = {
      {0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926},
      {0.02, 0.18, 1.50, 0.01, 1.30, 3.10},
      {0.05, 0.22, 1.42, 0.03, 1.24, 3.02},
  };

  const auto result = rt::retimeJointPathWithUnifiedSpeed(waypoints, 0.01, 220.0);

  ASSERT_FALSE(result.empty()) << result.samples.error_message;
  EXPECT_GT(result.samples.positions.size(), waypoints.size());
  EXPECT_EQ(result.samples.positions.front(), waypoints.front());
  EXPECT_EQ(result.samples.positions.back(), waypoints.back());
}

TEST(UnifiedRetimerTest, PathRetimingSizeConsistency) {
  const std::vector<std::vector<double>> waypoints = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.1, 0.1, 0.1, 0.1, 0.1, 0.1},
      {0.2, 0.2, 0.2, 0.2, 0.2, 0.2},
      {0.3, 0.3, 0.3, 0.3, 0.3, 0.3},
  };

  const auto result = rt::retimeJointPathWithUnifiedSpeed(waypoints, 0.01, 300.0);

  ASSERT_FALSE(result.empty());
  EXPECT_EQ(result.samples.velocities.size(), result.samples.positions.size());
  EXPECT_EQ(result.samples.accelerations.size(), result.samples.positions.size());
}

// ============================================================================
// Speed Scaling Tests
// ============================================================================

TEST(UnifiedRetimerTest, ScaledVelocityLimitsAllPositive) {
  const auto limits = rt::scaledUnifiedVelocityLimits(200.0);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_GT(limits[i], 0.0) << "Non-positive velocity limit at joint " << i;
    EXPECT_TRUE(std::isfinite(limits[i]));
  }
}

TEST(UnifiedRetimerTest, ScaledAccelerationLimitsAllPositive) {
  const auto limits = rt::scaledUnifiedAccelerationLimits(200.0);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_GT(limits[i], 0.0) << "Non-positive acceleration limit at joint " << i;
    EXPECT_TRUE(std::isfinite(limits[i]));
  }
}

TEST(UnifiedRetimerTest, HigherSpeedProducesHigherVelocityLimits) {
  const auto slow = rt::scaledUnifiedVelocityLimits(100.0);
  const auto fast = rt::scaledUnifiedVelocityLimits(1000.0);

  for (size_t i = 0; i < 6; ++i) {
    EXPECT_GE(fast[i], slow[i])
        << "Higher speed did not produce higher velocity limit at joint " << i;
  }
}

// ============================================================================
// Unified Limits Factory Test
// ============================================================================

TEST(UnifiedRetimerTest, MakeUnifiedRetimerLimitsProducesValidValues) {
  const auto limits = rt::makeUnifiedRetimerLimits(1.0, 2.0, 0.1);

  for (size_t i = 0; i < 6; ++i) {
    EXPECT_GT(limits.velocity_limits[i], 0.0);
    EXPECT_GT(limits.acceleration_limits[i], 0.0);
  }
}

// ============================================================================
// Unified Retimer Config Factory Test
// ============================================================================

TEST(UnifiedRetimerTest, MakeUnifiedRetimerConfigSampleDtIsPreserved) {
  const auto config = rt::makeUnifiedRetimerConfig(0.005);
  EXPECT_NEAR(config.sample_dt, 0.005, 1e-9);
}

// ============================================================================
// to_string Tests
// ============================================================================

TEST(UnifiedRetimerTest, SourceFamilyToStringIsNonEmpty) {
  EXPECT_GT(std::string(rt::to_string(rt::RetimerSourceFamily::joint)).size(), 0u);
  EXPECT_GT(std::string(rt::to_string(rt::RetimerSourceFamily::cartesian)).size(), 0u);
  EXPECT_GT(std::string(rt::to_string(rt::RetimerSourceFamily::replay)).size(), 0u);
  EXPECT_GT(std::string(rt::to_string(rt::RetimerSourceFamily::s_trajectory)).size(), 0u);
}

TEST(UnifiedRetimerTest, RetimerNoteToStringIsNonEmpty) {
  EXPECT_GT(std::string(rt::to_string(rt::RetimerNote::nominal)).size(), 0u);
  EXPECT_GT(std::string(rt::to_string(rt::RetimerNote::limits_clamped)).size(), 0u);
  EXPECT_GT(std::string(rt::to_string(rt::RetimerNote::speed_scale_applied)).size(), 0u);
}

TEST(UnifiedRetimerTest, DescribeRetimerMetadataProducesNonEmptyString) {
  rt::RetimerMetadata metadata;
  metadata.source_family = rt::RetimerSourceFamily::joint;
  metadata.total_duration = 2.5;
  metadata.sample_dt = 0.01;
  metadata.note = rt::RetimerNote::nominal;

  const auto description = rt::describeRetimerMetadata(metadata);
  EXPECT_FALSE(description.empty());
}

// ============================================================================
// WithUnifiedLimits variant test
// ============================================================================

TEST(UnifiedRetimerTest, RetimeWithUnifiedLimitsMatchesConfigVariant) {
  const std::vector<double> start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> target = {0.2, 0.1, 0.3, -0.1, 0.2, 0.5};

  const std::array<double, 6> vel_lim = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  const std::array<double, 6> acc_lim = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

  const auto result = rt::retimeJointWithUnifiedLimits(
      start, target, 0.01, vel_lim, acc_lim);

  ASSERT_FALSE(result.empty()) << result.samples.error_message;
  EXPECT_GT(result.samples.total_time, 0.0);
  EXPECT_EQ(result.samples.positions.front(), start);
  EXPECT_EQ(result.samples.positions.back(), target);
}
