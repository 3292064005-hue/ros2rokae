#include <gtest/gtest.h>

#include <vector>

#include "runtime/planner_core.hpp"
#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

TEST(MotionPlannerCoreTest, PlansReachableMoveLAndMoveCSequence) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> start_joints = {0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
  const auto start_pose = kinematics.forwardKinematicsRPY(start_joints);

  rt::MotionRequest request;
  request.request_id = "planner_smoke";
  request.start_joints = start_joints;
  request.default_speed = 25;
  request.default_zone = 5;
  request.strict_conf = false;
  request.avoid_singularity = true;
  request.trajectory_dt = 0.01;

  rt::MotionCommandSpec line;
  line.kind = rt::MotionKind::move_l;
  line.speed = 20;
  line.zone = 5;
  line.target_cartesian = start_pose;
  line.target_cartesian[0] -= 0.015;
  line.target_cartesian[1] += 0.010;
  line.target_cartesian[2] -= 0.015;

  rt::MotionCommandSpec arc;
  arc.kind = rt::MotionKind::move_c;
  arc.speed = 18;
  arc.zone = 5;
  arc.aux_cartesian = line.target_cartesian;
  arc.aux_cartesian[1] += 0.030;
  arc.target_cartesian = line.target_cartesian;
  arc.target_cartesian[0] += 0.025;
  arc.target_cartesian[1] += 0.030;

  request.commands = {line, arc};

  rt::MotionPlanner planner;
  const auto plan = planner.plan(request);

  ASSERT_TRUE(plan.valid()) << plan.error_message;
  ASSERT_EQ(plan.segments.size(), 2u);
  for (const auto &segment : plan.segments) {
    EXPECT_FALSE(segment.joint_trajectory.empty());
    ASSERT_EQ(segment.target_joints.size(), 6u);
  }
}
