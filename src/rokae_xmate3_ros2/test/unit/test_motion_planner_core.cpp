#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "runtime/joint_retimer.hpp"
#include "runtime/planner_core.hpp"
#include "runtime/planning_utils.hpp"
#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

TEST(MotionPlannerCoreTest, PlansReachableCartesianLookaheadSequence) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> start_joints = {0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
  const auto start_pose = kinematics.forwardKinematicsRPY(start_joints);

  rt::MotionRequest request;
  request.request_id = "planner_smoke";
  request.start_joints = start_joints;
  request.default_speed = 80;
  request.default_zone = 4;
  request.strict_conf = false;
  request.avoid_singularity = true;
  request.trajectory_dt = 0.01;

  rt::MotionCommandSpec line;
  line.kind = rt::MotionKind::move_l;
  line.speed = 75;
  line.zone = 4;
  line.target_cartesian = start_pose;
  line.target_cartesian[0] -= 0.008;
  line.target_cartesian[1] += 0.005;
  line.target_cartesian[2] -= 0.008;

  rt::MotionCommandSpec line2;
  line2.kind = rt::MotionKind::move_l;
  line2.speed = 70;
  line2.zone = 0;
  line2.target_cartesian = line.target_cartesian;
  line2.target_cartesian[0] += 0.010;
  line2.target_cartesian[1] += 0.008;
  line2.target_cartesian[2] += 0.004;

  request.commands = {line, line2};

  rt::MotionPlanner planner;
  const auto plan = planner.plan(request);

  ASSERT_TRUE(plan.valid()) << plan.error_message;
  ASSERT_EQ(plan.segments.size(), 2u);
  for (const auto &segment : plan.segments) {
    EXPECT_FALSE(segment.joint_trajectory.empty());
    ASSERT_EQ(segment.target_joints.size(), 6u);
  }
  EXPECT_TRUE(plan.segments.front().blend_to_next);
  EXPECT_TRUE(plan.segments.front().path_blended);
  EXPECT_EQ(plan.segments.front().path_family, rt::PathFamily::cartesian_line);
  EXPECT_EQ(plan.segments.back().path_family, rt::PathFamily::cartesian_line);
  EXPECT_GT(plan.segments.front().path_exit_trim_m, 0.0);
  EXPECT_GT(plan.segments.back().path_entry_trim_m, 0.0);
  EXPECT_GT(plan.segments.front().blend_length_m, 0.0);
  EXPECT_GT(plan.segments.front().path_length_m, plan.segments.front().blend_length_m);
  ASSERT_FALSE(plan.segments.front().joint_trajectory.empty());
  ASSERT_FALSE(plan.segments.back().joint_trajectory.empty());
  EXPECT_EQ(plan.segments.front().joint_trajectory.back().size(), 6u);
  EXPECT_EQ(plan.segments.back().joint_trajectory.front().size(), 6u);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(plan.segments.front().joint_trajectory.back()[i],
                plan.segments.back().joint_trajectory.front()[i],
                2e-5);
  }
}


TEST(MotionPlannerCoreTest, AppliesJointZoneBlendAndPreservesTimingMetadata) {
  rt::MotionRequest request;
  request.request_id = "joint_zone_blend";
  request.start_joints = {0.0, 0.10, 1.45, 0.0, 1.30, 3.1415926};
  request.default_speed = 300;
  request.default_zone = 10;
  request.trajectory_dt = 0.01;

  rt::MotionCommandSpec first;
  first.kind = rt::MotionKind::move_absj;
  first.speed = 300;
  first.zone = 12;
  first.target_joints = {0.08, 0.18, 1.40, 0.03, 1.26, 3.10};

  rt::MotionCommandSpec second;
  second.kind = rt::MotionKind::move_absj;
  second.speed = 280;
  second.zone = 0;
  second.target_joints = {0.16, 0.24, 1.34, 0.06, 1.20, 3.02};

  request.commands = {first, second};

  rt::MotionPlanner planner;
  const auto plan = planner.plan(request);

  ASSERT_TRUE(plan.valid()) << plan.error_message;
  ASSERT_EQ(plan.segments.size(), 2u);
  EXPECT_TRUE(plan.segments.front().blend_to_next);
  EXPECT_NEAR(plan.segments.front().trajectory_total_time,
              (plan.segments.front().joint_trajectory.size() - 1) * plan.segments.front().trajectory_dt,
              1e-9);
  EXPECT_NEAR(plan.segments.back().trajectory_total_time,
              (plan.segments.back().joint_trajectory.size() - 1) * plan.segments.back().trajectory_dt,
              1e-9);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(plan.segments.front().joint_trajectory.back()[i],
                plan.segments.back().joint_trajectory.front()[i],
                1e-6);
  }
}

TEST(MotionPlannerCoreTest, MixedModeZoneFallsBackToStopPointWithPlanNote) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> start_joints = {0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
  const std::vector<double> joint_target = {0.05, 0.20, 1.48, 0.02, 1.28, 3.08};
  const auto start_pose = kinematics.forwardKinematicsRPY(joint_target);

  rt::MotionRequest request;
  request.request_id = "mixed_mode_fallback";
  request.start_joints = start_joints;
  request.default_speed = 250;
  request.default_zone = 10;
  request.strict_conf = false;
  request.avoid_singularity = true;
  request.trajectory_dt = 0.01;

  rt::MotionCommandSpec absj;
  absj.kind = rt::MotionKind::move_absj;
  absj.speed = 250;
  absj.zone = 12;
  absj.target_joints = joint_target;

  rt::MotionCommandSpec line;
  line.kind = rt::MotionKind::move_l;
  line.speed = 200;
  line.zone = 8;
  line.target_cartesian = start_pose;
  line.target_cartesian[0] -= 0.004;
  line.target_cartesian[1] += 0.004;

  request.commands = {absj, line};

  rt::MotionPlanner planner;
  const auto plan = planner.plan(request);

  ASSERT_TRUE(plan.valid()) << plan.error_message;
  ASSERT_EQ(plan.segments.size(), 2u);
  EXPECT_FALSE(plan.segments.front().blend_to_next);
  EXPECT_EQ(plan.segments.front().path_exit_trim_m, 0.0);
  EXPECT_EQ(plan.segments.back().path_entry_trim_m, 0.0);
  ASSERT_FALSE(plan.notes.empty());
  EXPECT_NE(plan.notes.front().find("mixed-mode junction"), std::string::npos);
}

TEST(MotionPlannerCoreTest, LongCartesianPlansUseAdaptiveSamplingBeyondLegacyCaps) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> start_joints = {0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
  const auto start_pose = kinematics.forwardKinematicsRPY(start_joints);

  rt::MotionRequest request;
  request.request_id = "adaptive_cartesian_sampling";
  request.start_joints = start_joints;
  request.default_speed = 80;
  request.trajectory_dt = 0.005;

  rt::MotionCommandSpec line;
  line.kind = rt::MotionKind::move_l;
  line.speed = 80;
  line.target_cartesian = start_pose;
  line.target_cartesian[0] -= 0.010;
  line.target_cartesian[1] += 0.008;

  request.commands = {line};

  rt::MotionPlanner planner;
  planner.resetDebugCounters();
  const auto plan = planner.plan(request);
  const auto counters = planner.debugCounters();

  ASSERT_TRUE(plan.valid()) << plan.error_message;
  ASSERT_EQ(plan.segments.size(), 1u);
  EXPECT_GT(plan.segments.front().joint_trajectory.size(), 25u);
  EXPECT_LE(plan.segments.front().trajectory_dt, 0.01);
  EXPECT_EQ(plan.segments.front().path_family, rt::PathFamily::cartesian_line);
  EXPECT_FALSE(plan.segments.front().path_blended);
  EXPECT_EQ(plan.segments.front().path_entry_trim_m, 0.0);
  EXPECT_EQ(plan.segments.front().path_exit_trim_m, 0.0);
  EXPECT_NEAR(plan.segments.front().trajectory_total_time,
              (plan.segments.front().joint_trajectory.size() - 1) * plan.segments.front().trajectory_dt,
              1e-9);
  for (std::size_t i = 1; i < plan.segments.front().joint_trajectory.size(); ++i) {
    double max_step = 0.0;
    for (std::size_t joint = 0; joint < 6; ++joint) {
      max_step = std::max(max_step,
                          std::abs(plan.segments.front().joint_trajectory[i - 1][joint] -
                                   plan.segments.front().joint_trajectory[i][joint]));
    }
    EXPECT_LE(max_step, 0.75);
  }
  EXPECT_GT(counters.jacobian_calls, 0u);
  EXPECT_EQ(counters.jacobian_calls, counters.svd_calls);
}

TEST(MotionPlannerCoreTest, CartesianZoneZeroKeepsTrimMetadataAtZero) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> start_joints = {0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
  const auto start_pose = kinematics.forwardKinematicsRPY(start_joints);

  rt::MotionRequest request;
  request.request_id = "cartesian_zone_zero";
  request.start_joints = start_joints;
  request.default_speed = 180.0;
  request.trajectory_dt = 0.01;

  rt::MotionCommandSpec first;
  first.kind = rt::MotionKind::move_l;
  first.speed = 180.0;
  first.zone = 0;
  first.target_cartesian = start_pose;
  first.target_cartesian[0] -= 0.006;
  first.target_cartesian[1] += 0.004;

  rt::MotionCommandSpec second = first;
  second.target_cartesian = first.target_cartesian;
  second.target_cartesian[1] += 0.006;

  request.commands = {first, second};

  rt::MotionPlanner planner;
  const auto plan = planner.plan(request);

  ASSERT_TRUE(plan.valid()) << plan.error_message;
  ASSERT_EQ(plan.segments.size(), 2u);
  EXPECT_FALSE(plan.segments.front().blend_to_next);
  EXPECT_EQ(plan.segments.front().path_exit_trim_m, 0.0);
  EXPECT_EQ(plan.segments.back().path_entry_trim_m, 0.0);
  EXPECT_EQ(plan.segments.front().blend_length_m, 0.0);
}

TEST(MotionPlannerCoreTest, AcceptsNoOpMoveLWithoutFailingFollowingOffsetMoveL) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> start_joints = {0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
  const auto start_pose = kinematics.forwardKinematicsRPY(start_joints);

  rt::MotionRequest request;
  request.request_id = "noop_movel";
  request.start_joints = start_joints;
  request.default_speed = 120.0;
  request.trajectory_dt = 0.01;

  rt::MotionCommandSpec first;
  first.kind = rt::MotionKind::move_l;
  first.speed = 120.0;
  first.zone = 0;
  first.target_cartesian = start_pose;

  rt::MotionCommandSpec second = first;
  second.target_cartesian = start_pose;
  second.target_cartesian[2] += 0.02;

  request.commands = {first, second};

  rt::MotionPlanner planner;
  const auto plan = planner.plan(request);

  ASSERT_TRUE(plan.valid()) << plan.error_message;
  ASSERT_EQ(plan.segments.size(), 2u);
  EXPECT_EQ(plan.segments.front().path_family, rt::PathFamily::cartesian_line);
  EXPECT_EQ(plan.segments.front().joint_trajectory.size(), 1u);
  EXPECT_DOUBLE_EQ(plan.segments.front().trajectory_total_time, 0.0);
  EXPECT_FALSE(plan.segments.front().blend_to_next);
  EXPECT_FALSE(plan.segments.back().joint_trajectory.empty());
}

TEST(MotionPlannerCoreTest, MoveSpUsesCartesianLookaheadPipelineAndDerivativeMetadata) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> start_joints = {0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
  const auto start_pose = kinematics.forwardKinematicsRPY(start_joints);

  rt::MotionRequest request;
  request.request_id = "spiral_pipeline";
  request.start_joints = start_joints;
  request.default_speed = 120.0;
  request.trajectory_dt = 0.01;

  rt::MotionCommandSpec spiral;
  spiral.kind = rt::MotionKind::move_sp;
  spiral.speed = 100.0;
  spiral.zone = 6;
  spiral.target_cartesian = start_pose;
  spiral.target_cartesian[0] += 0.006;
  spiral.target_cartesian[2] += 0.004;
  spiral.radius = 0.001;
  spiral.radius_step = 0.00002;
  spiral.angle = M_PI / 6.0;
  spiral.direction = true;

  request.commands = {spiral};

  rt::MotionPlanner planner;
  const auto plan = planner.plan(request);

  ASSERT_TRUE(plan.valid()) << plan.error_message;
  ASSERT_EQ(plan.segments.size(), 1u);
  const auto &segment = plan.segments.front();
  EXPECT_EQ(segment.path_family, rt::PathFamily::cartesian_spiral);
  EXPECT_GT(segment.path_length_m, 0.0);
  EXPECT_FALSE(segment.path_blended);
  EXPECT_EQ(segment.path_entry_trim_m, 0.0);
  EXPECT_EQ(segment.path_exit_trim_m, 0.0);
  EXPECT_EQ(segment.joint_velocity_trajectory.size(), segment.joint_trajectory.size());
  EXPECT_EQ(segment.joint_acceleration_trajectory.size(), segment.joint_trajectory.size());
  EXPECT_NEAR(segment.trajectory_total_time,
              (segment.joint_trajectory.size() - 1) * segment.trajectory_dt,
              1e-9);
  for (std::size_t i = 1; i < segment.joint_trajectory.size(); ++i) {
    EXPECT_LE(rt::max_joint_step(segment.joint_trajectory[i - 1], segment.joint_trajectory[i]),
              rt::joint_branch_jump_threshold());
  }
}

TEST(MotionPlannerCoreTest, JointMoveDurationIsDominatedBySlowestAxisLimit) {
  const auto original_config = gazebo::TrajectoryPlanner::config();
  auto config = original_config;
  config.joint_speed_limits_rad_per_sec = {3.0, 3.0, 0.5, 3.0, 3.0, 3.0};
  config.joint_acc_limits_rad_per_sec2 = {6.0, 6.0, 0.8, 6.0, 6.0, 6.0};
  gazebo::TrajectoryPlanner::setConfig(config);

  rt::MotionRequest request;
  request.request_id = "slow_axis_joint_duration";
  request.start_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  request.default_speed = 4000.0;
  request.trajectory_dt = 0.01;

  rt::MotionCommandSpec move_absj;
  move_absj.kind = rt::MotionKind::move_absj;
  move_absj.speed = 4000.0;
  move_absj.target_joints = {0.1, 0.1, 0.4, 0.1, 0.1, 0.1};
  request.commands = {move_absj};

  rt::MotionPlanner planner;
  const auto plan = planner.plan(request);

  gazebo::TrajectoryPlanner::setConfig(original_config);

  ASSERT_TRUE(plan.valid()) << plan.error_message;
  ASSERT_EQ(plan.segments.size(), 1u);
  const auto expected_total_time = rt::computeJointRetimerDuration(
      request.start_joints,
      move_absj.target_joints,
      config.joint_speed_limits_rad_per_sec,
      config.joint_acc_limits_rad_per_sec2);
  EXPECT_NEAR(plan.segments.front().trajectory_total_time, expected_total_time, 1e-9);
  EXPECT_EQ(plan.segments.front().joint_velocity_trajectory.size(), plan.segments.front().joint_trajectory.size());
  EXPECT_EQ(plan.segments.front().joint_acceleration_trajectory.size(), plan.segments.front().joint_trajectory.size());
}
