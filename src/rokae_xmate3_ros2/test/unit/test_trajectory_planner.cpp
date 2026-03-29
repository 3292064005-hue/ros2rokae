#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "rokae_xmate3_ros2/gazebo/trajectory_planner.hpp"

using namespace gazebo;

// ============================================================================
// Joint Move Planning Tests
// ============================================================================

TEST(TrajectoryPlannerTest, PlanJointMoveProducesValidSamples) {
  const std::vector<double> start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> end = {0.5, -0.3, 1.0, 0.2, -0.8, 1.5};

  const auto samples = TrajectoryPlanner::planJointMove(start, end, 300.0, 0.01);

  ASSERT_FALSE(samples.empty());
  EXPECT_GT(samples.total_time, 0.0);
  EXPECT_NEAR(samples.sample_dt, 0.01, 1e-9);

  // First point should be the start configuration
  ASSERT_EQ(samples.points.front().size(), 6u);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(samples.points.front()[i], start[i], 1e-6)
        << "Start mismatch at joint " << i;
  }

  // Last point should be the target configuration
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(samples.points.back()[i], end[i], 1e-4)
        << "End mismatch at joint " << i;
  }
}

TEST(TrajectoryPlannerTest, PlanJointMoveVelocitiesAndAccelerationsExist) {
  const std::vector<double> start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> end = {0.3, 0.2, 0.5, -0.1, 0.4, -0.2};

  const auto samples = TrajectoryPlanner::planJointMove(start, end, 200.0, 0.01);

  ASSERT_FALSE(samples.empty());
  EXPECT_EQ(samples.velocities.size(), samples.points.size());
  EXPECT_EQ(samples.accelerations.size(), samples.points.size());

  // End-point velocities should be zero (stop point)
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(samples.velocities.front()[i], 0.0, 1e-3)
        << "Non-zero start velocity at joint " << i;
    EXPECT_NEAR(samples.velocities.back()[i], 0.0, 1e-3)
        << "Non-zero end velocity at joint " << i;
  }
}

TEST(TrajectoryPlannerTest, PlanJointMoveIdenticalStartEndIsDegenerate) {
  const std::vector<double> same = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  const auto samples = TrajectoryPlanner::planJointMove(same, same, 300.0, 0.01);

  // A no-op move should either be empty or a single-point trajectory
  EXPECT_LE(samples.points.size(), 2u);
}

TEST(TrajectoryPlannerTest, PlanJointMoveSlowSpeedProducesLongerTrajectory) {
  const std::vector<double> start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> end = {0.5, 0.3, 0.8, -0.2, 0.6, 1.0};

  const auto fast = TrajectoryPlanner::planJointMove(start, end, 800.0, 0.01);
  const auto slow = TrajectoryPlanner::planJointMove(start, end, 100.0, 0.01);

  ASSERT_FALSE(fast.empty());
  ASSERT_FALSE(slow.empty());
  EXPECT_GT(slow.total_time, fast.total_time)
      << "Slower speed should produce longer trajectory";
}

// ============================================================================
// Cartesian Line Planning Tests
// ============================================================================

TEST(TrajectoryPlannerTest, PlanCartesianLineProducesValidSamples) {
  const std::vector<double> start_pose = {0.5, 0.0, 0.8, 0.0, M_PI, 0.0};
  auto end_pose = start_pose;
  end_pose[0] -= 0.05;  // 50mm in X
  end_pose[2] -= 0.03;  // 30mm in Z

  const auto samples = TrajectoryPlanner::planCartesianLine(start_pose, end_pose, 100.0, 0.01);

  ASSERT_FALSE(samples.empty());
  EXPECT_GT(samples.total_time, 0.0);
  ASSERT_EQ(samples.points.front().size(), 6u);

  // First pose should match start
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(samples.points.front()[i], start_pose[i], 1e-4)
        << "Cartesian start mismatch at " << i;
  }
}

TEST(TrajectoryPlannerTest, PlanCartesianLineIdenticalPointsIsDegenerate) {
  const std::vector<double> pose = {0.5, 0.0, 0.8, 0.0, M_PI, 0.0};
  const auto samples = TrajectoryPlanner::planCartesianLine(pose, pose, 100.0, 0.01);

  EXPECT_LE(samples.points.size(), 2u);
}

// ============================================================================
// Circular Arc Planning Tests
// ============================================================================

TEST(TrajectoryPlannerTest, PlanCircularArcProducesValidSamples) {
  const std::vector<double> start = {0.5, 0.0, 0.8, 0.0, M_PI, 0.0};
  const std::vector<double> aux = {0.5, 0.03, 0.78, 0.0, M_PI, 0.0};
  const std::vector<double> end = {0.5, 0.05, 0.8, 0.0, M_PI, 0.0};

  const auto samples = TrajectoryPlanner::planCircularArc(start, aux, end, 100.0, 0.01);

  ASSERT_FALSE(samples.empty());
  EXPECT_GT(samples.total_time, 0.0);
  EXPECT_GT(samples.points.size(), 3u);

  // All points should be finite
  for (const auto &point : samples.points) {
    for (size_t i = 0; i < point.size(); ++i) {
      EXPECT_TRUE(std::isfinite(point[i]))
          << "Non-finite circular arc point";
    }
  }
}

// ============================================================================
// Spiral Move Planning Tests
// ============================================================================

TEST(TrajectoryPlannerTest, PlanSpiralMoveProducesValidTrajectory) {
  const std::vector<double> start = {0.5, 0.0, 0.8, 0.0, M_PI, 0.0};
  auto end = start;
  end[0] += 0.01;
  end[2] += 0.005;

  const auto samples = TrajectoryPlanner::planSpiralMove(
      start, end,
      0.005,   // radius
      0.0001,  // radius_step
      M_PI,    // angle
      true,    // direction
      100.0, 0.01);

  ASSERT_FALSE(samples.empty());
  EXPECT_GT(samples.total_time, 0.0);
  EXPECT_GT(samples.points.size(), 5u);
}

// ============================================================================
// Configuration Management Tests
// ============================================================================

TEST(TrajectoryPlannerTest, ConfigRoundTrip) {
  const auto original = TrajectoryPlanner::config();

  TrajectoryPlannerConfig modified = original;
  modified.max_joint_step_rad = 0.05;
  modified.max_cartesian_step_m = 0.005;

  TrajectoryPlanner::setConfig(modified);
  const auto readback = TrajectoryPlanner::config();

  EXPECT_NEAR(readback.max_joint_step_rad, 0.05, 1e-9);
  EXPECT_NEAR(readback.max_cartesian_step_m, 0.005, 1e-9);

  // Restore original config
  TrajectoryPlanner::setConfig(original);
}

TEST(TrajectoryPlannerTest, SpeedLimitsAffectTrajectoryDuration) {
  const auto original = TrajectoryPlanner::config();

  auto slow_config = original;
  slow_config.joint_speed_limits_rad_per_sec = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  TrajectoryPlanner::setConfig(slow_config);

  const std::vector<double> start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> end = {0.3, 0.2, 0.5, -0.1, 0.4, -0.2};
  const auto slow_traj = TrajectoryPlanner::planJointMove(start, end, 500.0, 0.01);

  TrajectoryPlanner::setConfig(original);
  const auto fast_traj = TrajectoryPlanner::planJointMove(start, end, 500.0, 0.01);

  ASSERT_FALSE(slow_traj.empty());
  ASSERT_FALSE(fast_traj.empty());
  EXPECT_GE(slow_traj.total_time, fast_traj.total_time);
}

// ============================================================================
// S-curve blend ratio test
// ============================================================================

TEST(TrajectoryPlannerTest, JointMoveSmoothness) {
  const std::vector<double> start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> end = {0.6, -0.4, 1.2, 0.3, -0.9, 2.0};

  const auto samples = TrajectoryPlanner::planJointMove(start, end, 300.0, 0.01);
  ASSERT_GT(samples.points.size(), 10u);

  // Verify trajectory continuity: maximum inter-sample step should be bounded
  for (size_t i = 1; i < samples.points.size(); ++i) {
    double max_step = 0.0;
    for (size_t j = 0; j < 6; ++j) {
      max_step = std::max(max_step, std::fabs(
          samples.points[i][j] - samples.points[i - 1][j]));
    }
    EXPECT_LT(max_step, 0.5)
        << "Excessive inter-sample step at index " << i;
  }
}
