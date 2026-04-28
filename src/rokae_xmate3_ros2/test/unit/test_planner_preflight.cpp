#include <gtest/gtest.h>

#include "runtime/planner_preflight.hpp"

namespace rokae_xmate3_ros2::runtime {

TEST(PlannerPreflight, CapturesRequestPolicyHintsAndRiskMetadata) {
  MotionRequest request;
  request.request_id = "req-1";
  request.start_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  request.strict_conf = true;
  request.avoid_singularity = true;
  request.soft_limit_enabled = true;
  MotionCommandSpec command;
  command.kind = MotionKind::move_j;
  command.target_cartesian = {0.4, 0.1, 0.3, 3.14, 0.0, 0.0};
  command.requested_conf = {90, 0, 0, 0, 0, 0};
  request.commands.push_back(command);

  const auto report = runPlannerPreflight(request);
  EXPECT_TRUE(report.ok);
  EXPECT_EQ(report.request_id, "req-1");
  EXPECT_EQ(report.primary_backend, "kdl");
  EXPECT_EQ(report.auxiliary_backend, "improved_dh");
  EXPECT_EQ(report.fallback_backend, "improved_dh");
  EXPECT_EQ(report.branch_policy, "strict_conf");
  EXPECT_EQ(report.selected_branch, "strict_conf");
  EXPECT_EQ(report.retimer_family, "unified");
  EXPECT_EQ(report.command_count, 1u);
  EXPECT_EQ(report.dominant_motion_kind, "move_j");
  EXPECT_FALSE(report.fallback_permitted);
  EXPECT_GE(report.singularity_risk, 0.0);
  EXPECT_GE(report.continuity_risk, 0.0);
  EXPECT_GE(report.estimated_duration, 0.0);
  EXPECT_FALSE(report.notes.empty());
}

TEST(PlannerPreflight, RejectsInvalidInputsWithStableReasons) {
  MotionRequest request;
  request.request_id = "bad-req";
  request.start_joints = {0.0, 0.0, 0.0};
  MotionCommandSpec command;
  command.kind = MotionKind::move_absj;
  command.target_joints = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  request.commands.push_back(command);

  auto report = runPlannerPreflight(request);
  EXPECT_FALSE(report.ok);
  EXPECT_EQ(report.reject_reason, "invalid_start_joint_count");

  request.start_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  request.commands.front().kind = MotionKind::none;
  report = runPlannerPreflight(request);
  EXPECT_FALSE(report.ok);
  EXPECT_EQ(report.reject_reason, "unsupported_motion_kind");
}

TEST(PlannerPreflight, DetectsSoftLimitViolationAtStart) {
  MotionRequest request;
  request.request_id = "soft-start";
  request.start_joints = {10.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  request.soft_limit_enabled = true;
  MotionCommandSpec command;
  command.kind = MotionKind::move_absj;
  command.target_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  request.commands.push_back(command);

  const auto report = runPlannerPreflight(request);
  EXPECT_FALSE(report.ok);
  EXPECT_EQ(report.reject_reason, "soft_limit_violation");
}

}  // namespace rokae_xmate3_ros2::runtime
