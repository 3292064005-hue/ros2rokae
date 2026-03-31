#include <gtest/gtest.h>

#include "runtime/runtime_state.hpp"

using rokae_xmate3_ros2::runtime::RuntimeDiagnosticsState;

TEST(RuntimeDiagnosticsStateTest, ExtendedFieldsAreTracked) {
  RuntimeDiagnosticsState state;
  state.configure("kdl", {"rt", "nrt"}, "rt");
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
  state.setRtWatchdogSummary("late_cycles=1,max_gap_ms=6.0", 1, 6.0);
  state.setProfileCapabilitySummary("rt_simulated=active,experimental");
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
  EXPECT_NE(snap.rt_subscription_plan.find("q_m"), std::string::npos);
  EXPECT_NE(snap.profile_capability_summary.find("rt_simulated"), std::string::npos);
  EXPECT_NE(snap.runtime_option_summary.find("default_speed"), std::string::npos);
  EXPECT_EQ(snap.tool_catalog_size, 2u);
  EXPECT_EQ(snap.register_catalog_size, 3u);
}
