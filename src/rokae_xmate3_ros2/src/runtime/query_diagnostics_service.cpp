#include "runtime/service_facade.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <utility>

#include <Eigen/Dense>

#include "runtime/planner_core.hpp"
#include "runtime/planning_utils.hpp"
#include "runtime/pose_utils.hpp"
#include "runtime/service_facade_utils.hpp"
#include "runtime/unified_retimer.hpp"
#include "rokae_xmate3_ros2/gazebo/model_facade.hpp"

namespace rokae_xmate3_ros2::runtime {

void QueryFacade::handleGetRuntimeDiagnostics(
    const rokae_xmate3_ros2::srv::GetRuntimeDiagnostics::Request &req,
    rokae_xmate3_ros2::srv::GetRuntimeDiagnostics::Response &res) const {
  (void)req;
  const auto snapshot = diagnostics_state_.snapshot();
  const auto runtime_view = motion_runtime_.view();
  res.diagnostics.backend_mode = snapshot.backend_mode;
  res.diagnostics.control_owner = snapshot.control_owner;
  res.diagnostics.runtime_phase = snapshot.runtime_phase;
  res.diagnostics.shutdown_phase = snapshot.shutdown_phase;
  res.diagnostics.active_request_count = snapshot.active_request_count;
  res.diagnostics.active_goal_count = snapshot.active_goal_count;
  res.diagnostics.active_request_id = snapshot.active_request_id;
  res.diagnostics.active_execution_backend = snapshot.active_execution_backend;
  res.diagnostics.last_runtime_event = snapshot.last_runtime_event;
  res.diagnostics.last_plan_summary = snapshot.last_plan_summary;
  res.diagnostics.last_selected_candidate = snapshot.last_selected_candidate;
  res.diagnostics.last_plan_failure = snapshot.last_plan_failure;
  res.diagnostics.last_retimer_note = snapshot.last_retimer_note;
  res.diagnostics.last_servo_dt = snapshot.last_servo_dt;
  res.diagnostics.capability_flags = snapshot.capability_flags;
  res.diagnostics.motion_mode = snapshot.motion_mode;
  res.diagnostics.rt_mode = snapshot.rt_mode;
  res.diagnostics.active_profile = snapshot.active_profile;
  res.diagnostics.loop_hz = snapshot.loop_hz;
  res.diagnostics.state_stream_hz = snapshot.state_stream_hz;
  res.diagnostics.command_latency_ms = snapshot.command_latency_ms;
  res.diagnostics.rt_subscription_plan = snapshot.rt_subscription_plan;
  res.diagnostics.rt_prearm_status = snapshot.rt_prearm_status;
  res.diagnostics.rt_watchdog_summary = snapshot.rt_watchdog_summary;
  res.diagnostics.rt_late_cycle_count = snapshot.rt_late_cycle_count;
  res.diagnostics.rt_max_gap_ms = snapshot.rt_max_gap_ms;
  res.diagnostics.rt_avg_gap_ms = snapshot.rt_avg_gap_ms;
  res.diagnostics.rt_consecutive_late_cycles = snapshot.rt_consecutive_late_cycles;
  res.diagnostics.rt_stale_state_count = snapshot.rt_stale_state_count;
  res.diagnostics.rt_command_starvation_windows = snapshot.rt_command_starvation_windows;
  res.diagnostics.rt_last_trigger_reason = snapshot.rt_last_trigger_reason;
  res.diagnostics.rt_transport_source = snapshot.rt_transport_source;
  res.diagnostics.rt_scheduler_state = snapshot.rt_scheduler_state;
  res.diagnostics.rt_deadline_miss = snapshot.rt_deadline_miss;
  res.diagnostics.rt_rx_latency_us = snapshot.rt_rx_latency_us;
  res.diagnostics.rt_queue_depth = snapshot.rt_queue_depth;
  res.diagnostics.recent_runtime_events = snapshot.recent_runtime_events;
  res.diagnostics.event_bus_summary = snapshot.event_bus_summary;
  res.diagnostics.runtime_event_count = snapshot.runtime_event_count;
  res.diagnostics.planning_rejection_count = snapshot.planning_rejection_count;
  res.diagnostics.watchdog_trigger_count = snapshot.watchdog_trigger_count;
  res.diagnostics.profile_capability_summary = snapshot.profile_capability_summary;
  res.diagnostics.planning_capability_summary = snapshot.planning_capability_summary;
  res.diagnostics.runtime_option_summary = snapshot.runtime_option_summary;
  res.diagnostics.last_api_surface = snapshot.last_api_surface;
  res.diagnostics.last_result_source = snapshot.last_result_source;
  res.diagnostics.rt_dispatch_mode = snapshot.rt_dispatch_mode;
  res.diagnostics.rt_state_source = snapshot.rt_state_source;
  res.diagnostics.model_exactness_summary = snapshot.model_exactness_summary;
  res.diagnostics.model_primary_backend = snapshot.model_primary_backend;
  res.diagnostics.model_fallback_used = snapshot.model_fallback_used;
  res.diagnostics.catalog_provenance_summary = snapshot.catalog_provenance_summary;
  res.diagnostics.tool_catalog_size = snapshot.tool_catalog_size;
  res.diagnostics.wobj_catalog_size = snapshot.wobj_catalog_size;
  res.diagnostics.project_catalog_size = snapshot.project_catalog_size;
  res.diagnostics.register_catalog_size = snapshot.register_catalog_size;
  res.success = true;
  res.message = "query_authority=diagnostics_snapshot;runtime_phase=" + to_string(runtime_view.status.runtime_phase) + ";active_request=" + runtime_view.status.request_id;
}

void QueryFacade::handleGetEndEffectorTorque(const rokae_xmate3_ros2::srv::GetEndEffectorTorque::Request &req,
                                     rokae_xmate3_ros2::srv::GetEndEffectorTorque::Response &res) const {
  (void)req;
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau_array{};
  readAuthorityJointState(pos, vel, tau_array);
  const auto joints = detail::snapshot_joints(pos);
  const auto toolset = tooling_state_.toolset();
  const auto model_facade = detail::make_runtime_model_facade(kinematics_, toolset);
  const auto expected_torque = model_facade.expectedTorque(pos, vel);
  Eigen::Matrix<double, 6, 1> tau = Eigen::Matrix<double, 6, 1>::Zero();
  for (int i = 0; i < 6; ++i) {
    tau(i) = tau_array[i] - expected_torque[i];
  }
  const Eigen::MatrixXd jacobian = kinematics_.computeJacobian(joints);
  const Eigen::Matrix<double, 6, 1> wrench =
      jacobian.transpose().completeOrthogonalDecomposition().solve(tau);
  for (int i = 0; i < 6; ++i) {
    res.end_torque[i] = wrench(i);
  }
  res.success = true;
  res.message.clear();
}

void QueryFacade::handleGetEndWrench(const rokae_xmate3_ros2::srv::GetEndWrench::Request &req,
                                     rokae_xmate3_ros2::srv::GetEndWrench::Response &res) const {
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> measured{};
  readAuthorityJointState(pos, vel, measured);
  rokae_xmate3_ros2::srv::GetEndEffectorTorque::Request legacy_req;
  rokae_xmate3_ros2::srv::GetEndEffectorTorque::Response legacy_res;
  handleGetEndEffectorTorque(legacy_req, legacy_res);
  res.ref_type = req.ref_type;
  for (int i = 0; i < 6; ++i) {
    res.joint_torque_measured[i] = measured[i];
    res.external_joint_torque[i] = 0.0;
  }
  res.cart_force = {legacy_res.end_torque[0], legacy_res.end_torque[1], legacy_res.end_torque[2]};
  res.cart_torque = {legacy_res.end_torque[3], legacy_res.end_torque[4], legacy_res.end_torque[5]};
  res.fidelity_level = "SimApprox";
  res.success = legacy_res.success;
  res.error_code = legacy_res.success ? 0 : 1;
  res.error_msg = legacy_res.message;
}

}  // namespace rokae_xmate3_ros2::runtime
