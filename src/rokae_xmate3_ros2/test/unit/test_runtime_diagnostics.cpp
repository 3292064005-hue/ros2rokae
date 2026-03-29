#include <gtest/gtest.h>

#include "runtime/runtime_state.hpp"

using rokae_xmate3_ros2::runtime::RuntimeDiagnosticsState;

TEST(RuntimeDiagnosticsStateTest, ExtendedFieldsAreTracked) {
  RuntimeDiagnosticsState state;
  state.configure("kdl", {"rt", "nrt"}, "rt");
  rokae_xmate3_ros2::runtime::RuntimeStatus status;
  status.control_owner = rokae_xmate3_ros2::runtime::ControlOwner::trajectory;
  status.runtime_phase = rokae_xmate3_ros2::runtime::RuntimePhase::executing;
  state.updateRuntimeStatus(status);
  rokae_xmate3_ros2::runtime::RuntimeContractView view;
  view.shutdown_phase = rokae_xmate3_ros2::runtime::ShutdownPhase::running;
  state.updateShutdownContract(view);
  state.setSessionModes(3, 4);
  state.setLoopMetrics(1000.0, 1000.0, 1.5);

  const auto snap = state.snapshot();
  EXPECT_EQ(snap.backend_mode, "kdl");
  EXPECT_EQ(snap.control_owner, "trajectory");
  EXPECT_EQ(snap.runtime_phase, "executing");
  EXPECT_EQ(snap.shutdown_phase, "running");
  EXPECT_EQ(snap.motion_mode, 3);
  EXPECT_EQ(snap.rt_mode, 4);
  EXPECT_EQ(snap.active_profile, "rt");
  EXPECT_DOUBLE_EQ(snap.loop_hz, 1000.0);
  EXPECT_DOUBLE_EQ(snap.state_stream_hz, 1000.0);
  EXPECT_DOUBLE_EQ(snap.command_latency_ms, 1.5);
}
