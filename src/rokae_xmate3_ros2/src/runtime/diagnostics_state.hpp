#ifndef ROKAE_XMATE3_ROS2_RUNTIME_DIAGNOSTICS_STATE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_DIAGNOSTICS_STATE_HPP

#include <mutex>
#include <string>
#include <vector>

#include "runtime/runtime_snapshots.hpp"
#include "rokae_xmate3_ros2/runtime/runtime_contract.hpp"
#include "runtime/runtime_types.hpp"

namespace rokae_xmate3_ros2::runtime {

class RuntimeDiagnosticsState {
 public:
  void configure(const std::string &backend_mode,
                 const std::vector<std::string> &capability_flags,
                 const std::string &active_profile = "unknown");
  void updateRuntimeStatus(const RuntimeStatus &status);
  void updateShutdownContract(const RuntimeContractView &view);
  void notePlanFailure(const std::string &message);
  void notePlanSummary(const std::string &summary, const std::string &selected_candidate = "nominal");
  void noteRetimerNote(const std::string &message);
  void setLastServoDt(double dt);
  void setSessionModes(int motion_mode, int rt_mode);
  void setActiveProfile(const std::string &active_profile);
  void setLoopMetrics(double loop_hz, double state_stream_hz, double command_latency_ms);
  void setRtSubscriptionPlan(const std::string &summary);
  void setRtPrearmStatus(const std::string &status);
  void setRtWatchdogSummary(const std::string &summary,
                            std::uint32_t late_cycle_count,
                            double max_gap_ms,
                            double avg_gap_ms,
                            std::uint32_t consecutive_late_cycles,
                            std::uint32_t stale_state_count,
                            std::uint32_t command_starvation_windows,
                            const std::string &last_trigger_reason);
  void setRtIngressMetrics(const std::string &transport_source,
                           double rx_latency_us,
                           std::uint32_t queue_depth);
  void setRtSchedulerState(const std::string &state);
  void incrementRtDeadlineMiss();
  void setProfileCapabilitySummary(const std::string &summary);
  void setPlanningCapabilitySummary(const std::string &summary);
  void setRuntimeOptionSummary(const std::string &summary);
  void setSemanticSurface(const std::string &api_surface,
                          const std::string &result_source,
                          const std::string &rt_dispatch_mode);
  void setRtStateSource(const std::string &source);
  void setModelExactnessSummary(const std::string &summary);
  void setModelBackendInfo(const std::string &primary_backend, bool fallback_used);
  void setCatalogProvenanceSummary(const std::string &summary);
  void setCatalogSizes(std::uint32_t tool_count,
                       std::uint32_t wobj_count,
                       std::uint32_t project_count,
                       std::uint32_t register_count);
  [[nodiscard]] RuntimeDiagnosticsSnapshot snapshot() const;

 private:
  mutable std::mutex mutex_;
  RuntimeDiagnosticsSnapshot snapshot_;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
