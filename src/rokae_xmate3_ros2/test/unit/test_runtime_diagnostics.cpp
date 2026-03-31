#include <gtest/gtest.h>

#include "runtime/runtime_state.hpp"

using rokae_xmate3_ros2::runtime::RuntimeDiagnosticsState;

TEST(RuntimeDiagnosticsStateTest, ExtendedFieldsAreTracked) {
  RuntimeDiagnosticsState state;
  state.configure("kdl", {"rt", "nrt", "rt.experimental", "trajectory_executor"}, "rt");
  rokae_xmate3_ros2::runtime::RuntimeStatus status;
  status.request_id = "req-7";
  status.control_owner = rokae_xmate3_ros2::runtime::ControlOwner::trajectory;
  status.runtime_phase = rokae_xmate3_ros2::runtime::RuntimePhase::executing;
  status.execution_backend = rokae_xmate3_ros2::runtime::ExecutionBackend::jtc;
  state.updateRuntimeStatus(status);
  rokae_xmate3_ros2::runtime::RuntimeContractView view;
  view.shutdown_phase = rokae_xmate3_ros2::runtime::ShutdownPhase::running;
  state.updateShutdownContract(view);
  state.setSessionModes(3, 4);
  state.setLoopMetrics(1000.0, 1000.0, 1.5);
  state.setRtSubscriptionPlan("armed_in_loop bytes=144 interval_ms=1 fields=q_m,dq_m,tau_m");
  state.setRtPrearmStatus("ready");
  state.setRtWatchdogSummary("late_cycles=1,max_gap_ms=6.0,avg_gap_ms=3.0", 1, 6.0, 3.0, 1, 2, 1, "late_cycle");
  state.setProfileCapabilitySummary("rt_simulated=active,experimental");
  state.setPlanningCapabilitySummary("kinematics[kdl=active]; retimer[nominal=active]; planner[risk_weighted=active]");
  state.notePlanSummary("plan_explanation[policy=risk_weighted; candidate=nominal]", "nominal");
  state.setRuntimeOptionSummary("default_speed=50.000; simulation_mode=true");
  state.setCatalogSizes(2, 1, 1, 3);

  const auto snap = state.snapshot();
  EXPECT_EQ(snap.backend_mode, "kdl");
  EXPECT_EQ(snap.control_owner, "trajectory");
  EXPECT_EQ(snap.runtime_phase, "executing");
  EXPECT_EQ(snap.shutdown_phase, "running");
  EXPECT_EQ(snap.active_request_id, "req-7");
  EXPECT_EQ(snap.active_execution_backend, "jtc");
  EXPECT_EQ(snap.motion_mode, 3);
  EXPECT_EQ(snap.rt_mode, 4);
  EXPECT_EQ(snap.active_profile, "rt");
  EXPECT_DOUBLE_EQ(snap.loop_hz, 1000.0);
  EXPECT_DOUBLE_EQ(snap.state_stream_hz, 1000.0);
  EXPECT_DOUBLE_EQ(snap.command_latency_ms, 1.5);
  EXPECT_EQ(snap.rt_prearm_status, "ready");
  EXPECT_EQ(snap.rt_late_cycle_count, 1u);
  EXPECT_DOUBLE_EQ(snap.rt_max_gap_ms, 6.0);
  EXPECT_DOUBLE_EQ(snap.rt_avg_gap_ms, 3.0);
  EXPECT_EQ(snap.rt_consecutive_late_cycles, 1u);
  EXPECT_EQ(snap.rt_stale_state_count, 2u);
  EXPECT_EQ(snap.rt_command_starvation_windows, 1u);
  EXPECT_EQ(snap.rt_last_trigger_reason, "late_cycle");
  EXPECT_EQ(snap.last_selected_candidate, "nominal");
  EXPECT_NE(snap.last_plan_summary.find("plan_explanation"), std::string::npos);
  EXPECT_NE(snap.rt_subscription_plan.find("q_m"), std::string::npos);
  EXPECT_NE(snap.profile_capability_summary.find("rt_simulated"), std::string::npos);
  EXPECT_NE(snap.planning_capability_summary.find("kinematics[kdl=active"), std::string::npos);
  EXPECT_NE(snap.event_bus_summary.find("events=1"), std::string::npos);
  EXPECT_NE(snap.runtime_option_summary.find("default_speed"), std::string::npos);
  EXPECT_EQ(snap.tool_catalog_size, 2u);
  EXPECT_EQ(snap.register_catalog_size, 3u);
}


TEST(RuntimeDiagnosticsStateTest, RecentRuntimeEventsTrackTransitions) {
  RuntimeDiagnosticsState state;
  state.configure("kdl", {"rt", "nrt"}, "rt");
  rokae_xmate3_ros2::runtime::RuntimeStatus status;
  status.request_id = "req-events";
  status.runtime_phase = rokae_xmate3_ros2::runtime::RuntimePhase::planning;
  status.last_event = "planning_requested";
  status.message = "planning";
  state.updateRuntimeStatus(status);

  status.runtime_phase = rokae_xmate3_ros2::runtime::RuntimePhase::executing;
  status.execution_backend = rokae_xmate3_ros2::runtime::ExecutionBackend::effort;
  status.last_event = "execution_started";
  status.message = "executing";
  state.updateRuntimeStatus(status);

  status.last_event = "planning_rejected";
  status.message = "plan failed";
  status.state = rokae_xmate3_ros2::runtime::ExecutionState::failed;
  state.updateRuntimeStatus(status);

  const auto snap = state.snapshot();
  EXPECT_EQ(snap.last_runtime_event, "planning_rejected");
  ASSERT_FALSE(snap.recent_runtime_events.empty());
  EXPECT_NE(snap.recent_runtime_events.back().find("planning_rejected:plan failed"), std::string::npos);
  EXPECT_EQ(snap.runtime_event_count, 3u);
  EXPECT_EQ(snap.planning_rejection_count, 1u);
}
