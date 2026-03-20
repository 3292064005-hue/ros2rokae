#include <gtest/gtest.h>

#include <array>
#include <string>

#include "runtime/runtime_state.hpp"
#include "runtime/request_coordinator.hpp"
#include "runtime/runtime_publish_bridge.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

TEST(MotionRequestCoordinatorTest, SubmitMoveAppendOccupiesRuntimeSlotUntilReset) {
  rt::MotionOptionsState motion_options_state;
  motion_options_state.setDefaultSpeed(35);
  motion_options_state.setDefaultZone(4);
  motion_options_state.setAvoidSingularity(true);

  rt::MotionRuntime runtime;
  rt::MotionRequestCoordinator coordinator(motion_options_state, runtime);

  rokae_xmate3_ros2::action::MoveAppend::Goal goal;
  goal.absj_cmds.resize(1);
  goal.absj_cmds.front().target.joints = {0.1, -0.2, 1.4, 0.0, 1.1, 3.14};
  goal.absj_cmds.front().speed = 0;
  goal.absj_cmds.front().zone = 2;

  const std::array<double, 6> current = {0.0, 0.0, 1.5, 0.0, 1.2, 3.14};
  const auto submission = coordinator.submitMoveAppend(goal, current, 0.01, "req_move_append");
  ASSERT_TRUE(submission.success) << submission.message;
  EXPECT_FALSE(coordinator.canAcceptRequest());

  const auto view = coordinator.currentView();
  EXPECT_TRUE(view.has_request);
  EXPECT_EQ(view.status.request_id, "req_move_append");
  EXPECT_TRUE(view.status.state == rt::ExecutionState::planning ||
              view.status.state == rt::ExecutionState::queued);

  const auto feedback = rt::buildMoveAppendFeedback(view.status, 0, "");
  EXPECT_TRUE(feedback.should_publish);
  EXPECT_EQ(feedback.current_state, rt::to_string(view.status.state));

  coordinator.reset();
  EXPECT_TRUE(coordinator.canAcceptRequest());
}
