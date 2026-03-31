#include <gtest/gtest.h>

#include "runtime/planner_preflight.hpp"

namespace rokae_xmate3_ros2::runtime {

TEST(PlannerPreflight, CapturesRequestPolicyHints) {
  MotionRequest request;
  request.request_id = "req-1";
  request.start_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  request.strict_conf = true;
  request.avoid_singularity = true;
  request.soft_limit_enabled = true;
  MotionCommandSpec command;
  command.kind = MotionKind::move_j;
  command.target_cartesian = {0.4, 0.1, 0.3, 3.14, 0.0, 0.0};
  request.commands.push_back(command);

  const auto report = runPlannerPreflight(request);
  EXPECT_TRUE(report.ok);
  EXPECT_EQ(report.request_id, "req-1");
  EXPECT_EQ(report.primary_backend, "kdl");
  EXPECT_EQ(report.auxiliary_backend, "improved_dh");
  EXPECT_EQ(report.branch_policy, "strict_conf");
  EXPECT_EQ(report.retimer_family, "unified");
  EXPECT_EQ(report.command_count, 1u);
  EXPECT_EQ(report.dominant_motion_kind, "move_j");
  EXPECT_FALSE(report.fallback_permitted);
}

}  // namespace rokae_xmate3_ros2::runtime
