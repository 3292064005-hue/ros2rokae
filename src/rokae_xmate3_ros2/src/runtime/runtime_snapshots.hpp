#ifndef ROKAE_XMATE3_ROS2_RUNTIME_SNAPSHOTS_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_SNAPSHOTS_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"

namespace rokae_xmate3_ros2::runtime {

inline constexpr const char *kRecordedPathSchemaVersion = "v2";
inline constexpr const char *kRecordedPathLegacySchemaVersion = "v1";
inline constexpr const char *kRecordedPathRobotFamily = "xMate6";
inline constexpr const char *kRecordedPathRobotModel = "xMate3";
inline constexpr const char *kRecordedPathCanonicalIdentity = "xCoreSDK:xmate6";
inline constexpr double kRecordedPathMonotonicStepSec = 0.01;
inline constexpr const char *kRecordedPathDefaultTaskPhase = "path_record";

struct ToolsetSnapshot {
  std::string tool_name;
  std::string wobj_name;
  std::vector<double> tool_pose;
  std::vector<double> wobj_pose;
  std::vector<double> base_pose;
  double tool_mass = 0.0;
  std::array<double, 3> tool_com{{0.0, 0.0, 0.0}};
};

struct CollisionDetectionSnapshot {
  bool enabled = false;
  std::array<double, 6> sensitivity{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  std::uint8_t behaviour = 1;
  double fallback = 0.0;
};

struct SoftLimitSnapshot {
  bool enabled = false;
  std::array<std::array<double, 2>, 6> limits = rokae_xmate3_ros2::spec::xmate3::kDefaultSoftLimits;
};

struct ProgramSnapshot {
  bool rl_project_loaded = false;
  bool rl_project_running = false;
  int rl_current_episode = 0;
  bool recording_path = false;
  std::string loaded_rl_project_name;
  std::string loaded_rl_project_path;
};

struct RecordedPathSample {
  double time_from_start_sec = 0.0;
  std::array<double, 6> joint_position{};
  std::array<double, 6> joint_velocity{};
  bool has_end_pose = false;
  std::array<double, 6> end_pose{};
  bool has_contact_force = false;
  double contact_force = 0.0;
  bool contact_established = false;
  std::string image_frame_id;
  std::string task_phase{kRecordedPathDefaultTaskPhase};
  std::string source_id{"sdk_record"};
};

struct ReplayPathAssetMetadata {
  std::string version{kRecordedPathSchemaVersion};
  std::string robot{kRecordedPathRobotFamily};
  std::string robot_model{kRecordedPathRobotModel};
  std::string canonical_identity{kRecordedPathCanonicalIdentity};
  std::string source{"sdk_record"};
  double created_at_sec = 0.0;
  double monotonic_step_sec = kRecordedPathMonotonicStepSec;
};

struct ReplayPathAsset {
  ReplayPathAssetMetadata metadata;
  std::vector<RecordedPathSample> samples;
  ToolsetSnapshot toolset;
  std::string source{"sdk_record"};
};

[[nodiscard]] inline bool isReplayPathSchemaVersionSupported(std::string_view version) noexcept {
  return version.empty() || version == kRecordedPathSchemaVersion || version == kRecordedPathLegacySchemaVersion;
}

inline void normalizeReplayPathAssetForConsumption(ReplayPathAsset &asset) {
  if (asset.metadata.version.empty() || asset.metadata.version == kRecordedPathLegacySchemaVersion) {
    asset.metadata.version = kRecordedPathSchemaVersion;
  }
  if (asset.metadata.robot.empty()) {
    asset.metadata.robot = kRecordedPathRobotFamily;
  }
  if (asset.metadata.robot_model.empty()) {
    asset.metadata.robot_model = kRecordedPathRobotModel;
  }
  if (asset.metadata.canonical_identity.empty()) {
    asset.metadata.canonical_identity = kRecordedPathCanonicalIdentity;
  }
  if (asset.metadata.source.empty()) {
    asset.metadata.source = "sdk_record";
  }
  if (asset.metadata.monotonic_step_sec <= 0.0) {
    asset.metadata.monotonic_step_sec = kRecordedPathMonotonicStepSec;
  }
  if (asset.source.empty()) {
    asset.source = asset.metadata.source;
  }
  for (auto &sample : asset.samples) {
    if (sample.task_phase.empty()) {
      sample.task_phase = kRecordedPathDefaultTaskPhase;
    }
    if (sample.source_id.empty()) {
      sample.source_id = asset.metadata.source;
    }
  }
}

enum class ReplayPathConsumptionTarget {
  replay,
  analysis,
  report,
  all,
};

struct ReplayPathAnalysisSample {
  double time_from_start_sec = 0.0;
  std::array<double, 6> end_pose{};
  double contact_force = 0.0;
  bool contact_established = false;
  std::string image_frame_id;
  std::string task_phase;
  std::string source_id;
};

struct ReplayPathAnalysisInput {
  ReplayPathAssetMetadata metadata;
  std::vector<ReplayPathAnalysisSample> samples;
  std::string source;
  std::string summary;
};

struct ReplayPathReportSummary {
  std::string canonical_identity;
  std::string source;
  std::size_t sample_count = 0;
  std::size_t phase_count = 0;
  std::size_t image_sample_count = 0;
  std::size_t contact_sample_count = 0;
  std::vector<std::string> phases;
  std::string summary;
};

struct ReplayPathConsumptionReport {
  bool ready_for_replay = false;
  bool ready_for_analysis = false;
  bool ready_for_report = false;
  std::size_t sample_count = 0;
  std::size_t samples_with_end_pose = 0;
  std::size_t samples_with_contact_force = 0;
  std::size_t samples_with_image_frame_id = 0;
  std::size_t phase_transition_count = 0;
  std::vector<std::string> phases;
  std::string summary;
  std::string error_message;
};

[[nodiscard]] inline ReplayPathConsumptionReport buildReplayPathConsumptionReport(ReplayPathAsset asset) {
  ReplayPathConsumptionReport report;
  if (!isReplayPathSchemaVersionSupported(asset.metadata.version)) {
    report.error_message = "Path schema version is not supported";
    return report;
  }
  normalizeReplayPathAssetForConsumption(asset);
  report.sample_count = asset.samples.size();
  if (asset.samples.empty()) {
    report.error_message = "Path is empty";
    return report;
  }

  const double minimum_step = asset.metadata.monotonic_step_sec > 0.0 ? asset.metadata.monotonic_step_sec
                                                                       : kRecordedPathMonotonicStepSec;
  double last_time = -minimum_step;
  for (const auto &sample : asset.samples) {
    if (!std::isfinite(sample.time_from_start_sec)) {
      report.error_message = "recorded path sample contains non-finite time";
      return report;
    }
    if (sample.time_from_start_sec <= last_time) {
      report.error_message = "recorded path sample time must be strictly monotonic";
      return report;
    }
    last_time = sample.time_from_start_sec;

    if (sample.task_phase.empty()) {
      report.error_message = "recorded path sample task_phase must not be empty";
      return report;
    }
    if (sample.source_id.empty()) {
      report.error_message = "recorded path sample source_id must not be empty";
      return report;
    }
    if (std::find(report.phases.begin(), report.phases.end(), sample.task_phase) == report.phases.end()) {
      if (!report.phases.empty()) {
        report.phase_transition_count += 1;
      }
      report.phases.push_back(sample.task_phase);
    }
    if (sample.has_end_pose) {
      const auto all_finite = std::all_of(sample.end_pose.begin(), sample.end_pose.end(), [](double value) {
        return std::isfinite(value);
      });
      if (!all_finite) {
        report.error_message = "recorded path end_pose must be finite when present";
        return report;
      }
      report.samples_with_end_pose += 1;
    }
    if (sample.has_contact_force) {
      if (!std::isfinite(sample.contact_force)) {
        report.error_message = "recorded path contact_force must be finite when present";
        return report;
      }
      report.samples_with_contact_force += 1;
    }
    if (!sample.image_frame_id.empty()) {
      report.samples_with_image_frame_id += 1;
    }
  }

  report.ready_for_replay = true;
  report.ready_for_analysis = report.samples_with_end_pose == report.sample_count &&
                              report.samples_with_contact_force == report.sample_count &&
                              report.samples_with_image_frame_id == report.sample_count;
  report.ready_for_report = report.ready_for_analysis && !report.phases.empty();
  report.summary = "samples=" + std::to_string(report.sample_count) +
                   "; end_pose_samples=" + std::to_string(report.samples_with_end_pose) +
                   "; contact_samples=" + std::to_string(report.samples_with_contact_force) +
                   "; image_samples=" + std::to_string(report.samples_with_image_frame_id) +
                   "; phases=" + std::to_string(report.phases.size()) +
                   "; replay_ready=" + std::string(report.ready_for_replay ? "true" : "false") +
                   "; analysis_ready=" + std::string(report.ready_for_analysis ? "true" : "false") +
                   "; report_ready=" + std::string(report.ready_for_report ? "true" : "false") +
                   "; canonical_identity=" + asset.metadata.canonical_identity;
  return report;
}

[[nodiscard]] inline bool buildReplayPathAnalysisInput(const ReplayPathAsset &asset,
                                                       ReplayPathAnalysisInput &analysis,
                                                       std::string *error_message = nullptr) {
  const auto report = buildReplayPathConsumptionReport(asset);
  if (!report.ready_for_analysis) {
    if (error_message != nullptr) {
      *error_message = report.error_message.empty()
                           ? std::string{"recorded path asset is not ready for analysis consumption"}
                           : report.error_message;
    }
    return false;
  }

  ReplayPathAsset normalized = asset;
  normalizeReplayPathAssetForConsumption(normalized);
  analysis = ReplayPathAnalysisInput{};
  analysis.metadata = normalized.metadata;
  analysis.source = normalized.source;
  analysis.summary = report.summary;
  analysis.samples.reserve(normalized.samples.size());
  for (const auto &sample : normalized.samples) {
    ReplayPathAnalysisSample entry;
    entry.time_from_start_sec = sample.time_from_start_sec;
    entry.end_pose = sample.end_pose;
    entry.contact_force = sample.contact_force;
    entry.contact_established = sample.contact_established;
    entry.image_frame_id = sample.image_frame_id;
    entry.task_phase = sample.task_phase;
    entry.source_id = sample.source_id;
    analysis.samples.push_back(std::move(entry));
  }
  if (error_message != nullptr) {
    error_message->clear();
  }
  return true;
}

[[nodiscard]] inline bool buildReplayPathReportSummary(const ReplayPathAsset &asset,
                                                       ReplayPathReportSummary &summary,
                                                       std::string *error_message = nullptr) {
  ReplayPathAnalysisInput analysis;
  if (!buildReplayPathAnalysisInput(asset, analysis, error_message)) {
    return false;
  }
  const auto report = buildReplayPathConsumptionReport(asset);
  summary = ReplayPathReportSummary{};
  summary.canonical_identity = analysis.metadata.canonical_identity;
  summary.source = analysis.source;
  summary.sample_count = analysis.samples.size();
  summary.phase_count = report.phases.size();
  summary.image_sample_count = report.samples_with_image_frame_id;
  summary.contact_sample_count = report.samples_with_contact_force;
  summary.phases = report.phases;
  summary.summary = report.summary;
  if (error_message != nullptr) {
    error_message->clear();
  }
  return true;
}

[[nodiscard]] inline bool validateReplayPathAssetForConsumption(
    const ReplayPathAsset &asset,
    ReplayPathConsumptionTarget target = ReplayPathConsumptionTarget::replay,
    std::string *error_message = nullptr) {
  const auto report = buildReplayPathConsumptionReport(asset);
  bool ok = false;
  switch (target) {
    case ReplayPathConsumptionTarget::replay:
      ok = report.ready_for_replay;
      break;
    case ReplayPathConsumptionTarget::analysis:
      ok = report.ready_for_analysis;
      break;
    case ReplayPathConsumptionTarget::report:
      ok = report.ready_for_report;
      break;
    case ReplayPathConsumptionTarget::all:
      ok = report.ready_for_replay && report.ready_for_analysis && report.ready_for_report;
      break;
  }
  if (error_message != nullptr) {
    *error_message = ok ? std::string{} : (report.error_message.empty()
                                               ? std::string{"recorded path asset failed requested consumption validation"}
                                               : report.error_message);
  }
  return ok;
}

struct RuntimeDiagnosticsSnapshot {
  std::string backend_mode{"unknown"};
  std::string control_owner{"none"};
  std::string runtime_phase{"idle"};
  std::string shutdown_phase{"running"};
  std::uint32_t active_request_count = 0;
  std::uint32_t active_goal_count = 0;
  std::string active_request_id;
  std::string active_execution_backend{"none"};
  std::string last_plan_summary;
  std::string last_selected_candidate{"nominal"};
  std::string last_runtime_event{"reset"};
  std::string last_plan_failure;
  std::string last_retimer_note;
  double last_servo_dt = 0.0;
  std::vector<std::string> capability_flags;
  int motion_mode = 0;
  int rt_mode = -1;
  std::string active_profile{"unknown"};
  double loop_hz = 0.0;
  double state_stream_hz = 0.0;
  double command_latency_ms = 0.0;
  std::string rt_subscription_plan{"inactive"};
  std::string rt_prearm_status{"not_applicable"};
  std::string rt_watchdog_summary{"nominal"};
  std::uint32_t rt_late_cycle_count = 0;
  double rt_max_gap_ms = 0.0;
  double rt_avg_gap_ms = 0.0;
  std::uint32_t rt_consecutive_late_cycles = 0;
  std::uint32_t rt_stale_state_count = 0;
  std::uint32_t rt_command_starvation_windows = 0;
  std::string rt_last_trigger_reason{"nominal"};
  std::string rt_transport_source{"unknown"};
  std::string rt_scheduler_state{"unknown"};
  std::uint32_t rt_deadline_miss = 0;
  double rt_rx_latency_us = 0.0;
  std::uint32_t rt_queue_depth = 0;
  std::vector<std::string> recent_runtime_events;
  std::string event_bus_summary{"events=0"};
  std::uint32_t runtime_event_count = 0;
  std::uint32_t planning_rejection_count = 0;
  std::uint32_t watchdog_trigger_count = 0;
  std::string profile_capability_summary{"unknown"};
  std::string planning_capability_summary{"unknown"};
  std::string runtime_option_summary{"unknown"};
  std::string last_api_surface{"unknown"};
  std::string last_result_source{"runtime"};
  std::string rt_dispatch_mode{"idle"};
  std::string rt_state_source{"unknown"};
  std::string query_authority{"runtime_request_coordinator"};
  std::string fidelity_class{"simulation_grade"};
  std::string model_revision{"xmate6_public_v2026_04"};
  std::string canonical_identity{"xCoreSDK:xmate6"};
  std::string model_exactness_summary{"kinematics=simulation_grade;dynamics=approximate;jacobian=simulation_grade;wrench=approximate"};
  std::string model_primary_backend{"unknown"};
  bool model_fallback_used = false;
  std::string catalog_provenance_summary{"runtime_authoritative"};
  std::uint32_t tool_catalog_size = 0;
  std::uint32_t wobj_catalog_size = 0;
  std::uint32_t project_catalog_size = 0;
  std::uint32_t register_catalog_size = 0;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
