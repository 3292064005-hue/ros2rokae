#include <gtest/gtest.h>

#include "runtime/runtime_state_machine.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

TEST(RuntimeStateMachine, PlanningQueuedExecutingCompletedFlow) {
  RuntimeStateMachine machine;
  RuntimeStatus status;
  RuntimePhase phase = RuntimePhase::idle;

  machine.apply(status, phase, RuntimeEvent{RuntimeEventType::planning_requested, "req-1", "planning", 3});
  EXPECT_EQ(status.request_id, "req-1");
  EXPECT_EQ(status.state, ExecutionState::planning);
  EXPECT_EQ(phase, RuntimePhase::planning);

  RuntimeEvent queued;
  queued.type = RuntimeEventType::plan_queued;
  queued.request_id = "req-1";
  queued.message = "queued";
  queued.total_segments = 3;
  machine.apply(status, phase, queued);
  EXPECT_EQ(status.state, ExecutionState::queued);
  EXPECT_EQ(status.total_segments, 3u);

  RuntimeEvent start;
  start.type = RuntimeEventType::execution_started;
  start.request_id = "req-1";
  start.execution_backend = ExecutionBackend::effort;
  start.message = "executing";
  machine.apply(status, phase, start);
  EXPECT_EQ(status.state, ExecutionState::executing);
  EXPECT_EQ(status.execution_backend, ExecutionBackend::effort);
  EXPECT_EQ(phase, RuntimePhase::executing);

  RuntimeEvent complete;
  complete.type = RuntimeEventType::completed;
  complete.request_id = "req-1";
  complete.message = "completed";
  complete.total_segments = 3;
  complete.completed_segments = 3;
  complete.current_segment_index = 2;
  complete.execution_backend = ExecutionBackend::effort;
  complete.terminal_success = true;
  machine.apply(status, phase, complete);
  EXPECT_EQ(status.state, ExecutionState::completed);
  EXPECT_TRUE(status.terminal_success);
  EXPECT_EQ(status.completed_segments, 3u);
  EXPECT_EQ(phase, RuntimePhase::idle);
}


TEST(RuntimeStateMachine, TerminalEventsPreserveExistingBackendWhenEventBackendMissing) {
  RuntimeStateMachine machine;
  RuntimeStatus status;
  RuntimePhase phase = RuntimePhase::idle;

  RuntimeEvent planning;
  planning.type = RuntimeEventType::planning_requested;
  planning.request_id = "req-keep-backend";
  planning.total_segments = 2;
  machine.apply(status, phase, planning);

  RuntimeEvent start;
  start.type = RuntimeEventType::execution_started;
  start.request_id = "req-keep-backend";
  start.execution_backend = ExecutionBackend::jtc;
  machine.apply(status, phase, start);
  EXPECT_EQ(status.execution_backend, ExecutionBackend::jtc);

  RuntimeEvent stop;
  stop.type = RuntimeEventType::stopped;
  stop.request_id = "req-keep-backend";
  stop.message = "stopped";
  stop.execution_backend = ExecutionBackend::none;
  machine.apply(status, phase, stop);

  EXPECT_EQ(status.execution_backend, ExecutionBackend::jtc);
  EXPECT_EQ(status.request_id, "req-keep-backend");
  EXPECT_EQ(status.state, ExecutionState::stopped);
}

TEST(RuntimeStateMachine, FailureAndOwnerChangeAreExplicit) {
  RuntimeStateMachine machine;
  RuntimeStatus status;
  RuntimePhase phase = RuntimePhase::idle;

  RuntimeEvent planning;
  planning.type = RuntimeEventType::planning_requested;
  planning.request_id = "req-2";
  planning.total_segments = 1;
  machine.apply(status, phase, planning);

  RuntimeEvent owner;
  owner.type = RuntimeEventType::owner_changed;
  owner.owner = ControlOwner::trajectory;
  owner.message = "jtc execution";
  machine.apply(status, phase, owner);
  EXPECT_EQ(status.control_owner, ControlOwner::trajectory);
  EXPECT_EQ(status.message, "jtc execution");

  RuntimeEvent fail;
  fail.type = RuntimeEventType::failed;
  fail.request_id = "req-2";
  fail.message = "planning failed";
  fail.execution_backend = ExecutionBackend::jtc;
  fail.terminal_success = false;
  machine.apply(status, phase, fail);
  EXPECT_EQ(status.state, ExecutionState::failed);
  EXPECT_FALSE(status.terminal_success);
  EXPECT_EQ(status.execution_backend, ExecutionBackend::jtc);
  EXPECT_EQ(phase, RuntimePhase::faulted);
}

}  // namespace
}  // namespace rokae_xmate3_ros2::runtime

namespace rokae_xmate3_ros2::runtime {
namespace {

TEST(RuntimeStateMachine, PlanningRejectedAndRetimedEventsExposeLastEvent) {
  RuntimeStateMachine machine;
  RuntimeStatus status;
  RuntimePhase phase = RuntimePhase::idle;

  RuntimeEvent planning;
  planning.type = RuntimeEventType::planning_requested;
  planning.request_id = "req-3";
  planning.total_segments = 1;
  machine.apply(status, phase, planning);
  EXPECT_EQ(status.last_event, "planning_requested");

  RuntimeEvent retimed;
  retimed.type = RuntimeEventType::trajectory_retimed;
  retimed.request_id = "req-3";
  retimed.message = "retimed";
  machine.apply(status, phase, retimed);
  EXPECT_EQ(status.last_event, "trajectory_retimed");
  EXPECT_EQ(status.message, "retimed");

  RuntimeEvent reject;
  reject.type = RuntimeEventType::planning_rejected;
  reject.request_id = "req-3";
  reject.message = "planner rejected request";
  machine.apply(status, phase, reject);
  EXPECT_EQ(status.last_event, "planning_rejected");
  EXPECT_EQ(status.state, ExecutionState::failed);
  EXPECT_EQ(phase, RuntimePhase::faulted);
}

}  // namespace
}  // namespace rokae_xmate3_ros2::runtime
