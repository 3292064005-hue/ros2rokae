#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "runtime/unified_retimer.hpp"
#include "rokae_xmate3_ros2/runtime/strict_jerk_profile.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

namespace {

void expectFiniteVectorSamples(const std::vector<std::vector<double>> &samples,
                               const char *label) {
  ASSERT_FALSE(samples.empty()) << label << " samples must not be empty";
  for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
    ASSERT_EQ(samples[sample_index].size(), 6u) << label << " sample width mismatch at " << sample_index;
    for (std::size_t axis = 0; axis < samples[sample_index].size(); ++axis) {
      EXPECT_TRUE(std::isfinite(samples[sample_index][axis]))
          << label << " non-finite at sample " << sample_index << " axis " << axis;
    }
  }
}

void expectSmoothCanonicalSamples(const rt::CanonicalTrajectorySamples &samples,
                                  const std::array<double, 6> &velocity_limits,
                                  const std::array<double, 6> &acceleration_limits,
                                  double endpoint_tolerance,
                                  double acceleration_jump_scale) {
  expectFiniteVectorSamples(samples.positions, "position");
  expectFiniteVectorSamples(samples.velocities, "velocity");
  expectFiniteVectorSamples(samples.accelerations, "acceleration");
  ASSERT_EQ(samples.velocities.size(), samples.positions.size());
  ASSERT_EQ(samples.accelerations.size(), samples.positions.size());

  for (std::size_t axis = 0; axis < 6; ++axis) {
    EXPECT_NEAR(samples.velocities.front()[axis], 0.0, endpoint_tolerance)
        << "start velocity not smooth at joint " << axis;
    EXPECT_NEAR(samples.velocities.back()[axis], 0.0, endpoint_tolerance)
        << "end velocity not smooth at joint " << axis;
    EXPECT_NEAR(samples.accelerations.front()[axis], 0.0, endpoint_tolerance)
        << "start acceleration not smooth at joint " << axis;
    EXPECT_NEAR(samples.accelerations.back()[axis], 0.0, endpoint_tolerance)
        << "end acceleration not smooth at joint " << axis;
  }

  for (std::size_t sample_index = 0; sample_index < samples.positions.size(); ++sample_index) {
    for (std::size_t axis = 0; axis < 6; ++axis) {
      EXPECT_LE(std::fabs(samples.velocities[sample_index][axis]),
                velocity_limits[axis] * 1.05 + 1e-9)
          << "velocity limit regression at sample " << sample_index << " axis " << axis;
      EXPECT_LE(std::fabs(samples.accelerations[sample_index][axis]),
                acceleration_limits[axis] * 1.10 + 1e-9)
          << "acceleration limit regression at sample " << sample_index << " axis " << axis;
    }
  }

  for (std::size_t sample_index = 1; sample_index < samples.accelerations.size(); ++sample_index) {
    for (std::size_t axis = 0; axis < 6; ++axis) {
      const double jump = std::fabs(samples.accelerations[sample_index][axis] -
                                   samples.accelerations[sample_index - 1][axis]);
      EXPECT_LE(jump, acceleration_limits[axis] * acceleration_jump_scale + 1e-9)
          << "acceleration jump spike at sample " << sample_index << " axis " << axis;
    }
  }
}

}  // namespace

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

TEST(UnifiedRetimerTest, StrictJerkScalarProfileIsFiniteAndEndpointSmooth) {
  constexpr double kVelocityLimit = 1.0;
  constexpr double kAccelerationLimit = 2.0;
  rt::StrictJerkLimitedScalarProfile profile;
  profile.configure(0.42, kVelocityLimit, kAccelerationLimit, kAccelerationLimit);

  ASSERT_GT(profile.total_time(), 0.0);
  constexpr int kSamples = 200;
  for (int index = 0; index <= kSamples; ++index) {
    const double t = profile.total_time() * static_cast<double>(index) / static_cast<double>(kSamples);
    const auto sample = profile.sample(t);
    EXPECT_TRUE(std::isfinite(sample.position));
    EXPECT_TRUE(std::isfinite(sample.velocity));
    EXPECT_TRUE(std::isfinite(sample.acceleration));
    EXPECT_GE(sample.position, -1e-12);
    EXPECT_LE(sample.position, profile.distance() + 1e-12);
    EXPECT_LE(std::fabs(sample.velocity), kVelocityLimit * 1.05);
    EXPECT_LE(std::fabs(sample.acceleration), kAccelerationLimit * 1.05);
  }

  const auto start = profile.sample(0.0);
  const auto end = profile.sample(profile.total_time());
  EXPECT_NEAR(start.velocity, 0.0, 1e-12);
  EXPECT_NEAR(start.acceleration, 0.0, 1e-12);
  EXPECT_NEAR(end.velocity, 0.0, 1e-12);
  EXPECT_NEAR(end.acceleration, 0.0, 1e-12);
  EXPECT_NEAR(end.position, profile.distance(), 1e-12);
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

TEST(UnifiedRetimerTest, PointToPointRetimingPassesStrictSmoothnessGate) {
  const std::vector<double> start = {0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
  const std::vector<double> target = {0.12, 0.25, 1.40, 0.04, 1.20, 3.00};
  const std::array<double, 6> velocity_limits = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  const std::array<double, 6> acceleration_limits = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

  const auto result = rt::retimeJointWithUnifiedLimits(
      start, target, 0.01, velocity_limits, acceleration_limits);

  ASSERT_FALSE(result.empty()) << result.samples.error_message;
  EXPECT_TRUE(result.metadata.jerk_constrained);
  expectSmoothCanonicalSamples(result.samples, velocity_limits, acceleration_limits, 1e-3, 1.25);
}

TEST(UnifiedRetimerTest, MultiWaypointPathRetimingPassesStrictSmoothnessGate) {
  const std::vector<std::vector<double>> waypoints = {
      {0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926},
      {0.03, 0.19, 1.50, 0.01, 1.30, 3.10},
      {0.07, 0.24, 1.43, 0.03, 1.23, 3.04},
      {0.10, 0.28, 1.37, 0.05, 1.18, 2.98},
  };
  const std::array<double, 6> velocity_limits = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  const std::array<double, 6> acceleration_limits = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

  const auto result = rt::retimeJointPathWithUnifiedLimits(
      waypoints, 0.01, velocity_limits, acceleration_limits);

  ASSERT_FALSE(result.empty()) << result.samples.error_message;
  EXPECT_TRUE(result.metadata.jerk_constrained);
  expectSmoothCanonicalSamples(result.samples, velocity_limits, acceleration_limits, 1e-3, 2.10);
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


TEST(UnifiedRetimerTest, MetadataFlagsExposeScalingAndClampState) {
  const std::vector<std::vector<double>> waypoints = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.4, 0.3, 0.2, 0.1, 0.2, 0.5},
  };

  const auto result = rt::retimeJointPathWithUnifiedSpeed(
      waypoints, 0.01, 50.0, rt::RetimerSourceFamily::joint, 1.5);

  ASSERT_FALSE(result.empty()) << result.samples.error_message;
  EXPECT_TRUE(result.metadata.jerk_constrained);
  EXPECT_TRUE(result.metadata.effective_speed_scale > 1.0);
  EXPECT_FALSE(result.metadata.detail.empty());
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


TEST(UnifiedRetimerTest, ValidationRejectsNonFiniteWaypointValues) {
  const std::vector<std::vector<double>> waypoints = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.1, 0.2, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0},
  };
  const std::array<double, 6> vel = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  const std::array<double, 6> acc = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

  const auto report = rt::validateUnifiedRetimerInput(
      waypoints, 0.01, vel, acc, rt::RetimerPolicy::nominal);
  EXPECT_FALSE(report.ok);
  EXPECT_NE(report.error_message.find("non-finite"), std::string::npos);
}

TEST(UnifiedRetimerTest, ConservativePolicyIsReflectedInMetadataDescription) {
  const std::vector<std::vector<double>> waypoints = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.4, 0.3, 0.2, 0.1, 0.2, 0.5},
  };

  const auto result = rt::retimeJointPathWithUnifiedSpeed(
      waypoints, 0.01, 200.0, rt::RetimerSourceFamily::joint, 1.0, rt::RetimerPolicy::conservative);

  ASSERT_FALSE(result.empty()) << result.samples.error_message;
  EXPECT_EQ(result.metadata.policy, rt::RetimerPolicy::conservative);
  const auto description = rt::describeRetimerMetadata(result.metadata);
  EXPECT_NE(description.find("policy=conservative"), std::string::npos);
}
