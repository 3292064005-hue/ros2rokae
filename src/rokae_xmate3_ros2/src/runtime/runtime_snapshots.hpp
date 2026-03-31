#ifndef ROKAE_XMATE3_ROS2_RUNTIME_SNAPSHOTS_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_SNAPSHOTS_HPP

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"

namespace rokae_xmate3_ros2::runtime {

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
};

struct ReplayPathAssetMetadata {
  std::string version{"v1"};
  std::string robot{"xMate3"};
  std::string source{"sdk_record"};
  double created_at_sec = 0.0;
};

struct ReplayPathAsset {
  ReplayPathAssetMetadata metadata;
  std::vector<RecordedPathSample> samples;
  ToolsetSnapshot toolset;
  std::string source{"sdk_record"};
};

struct RuntimeDiagnosticsSnapshot {
  std::string backend_mode{"unknown"};
  std::string control_owner{"none"};
  std::string runtime_phase{"idle"};
  std::string shutdown_phase{"running"};
  std::uint32_t active_request_count = 0;
  std::uint32_t active_goal_count = 0;
  std::string active_request_id;
  std::string active_execution_backend{"none"};
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
  std::string profile_capability_summary{"unknown"};
  std::string runtime_option_summary{"unknown"};
  std::uint32_t tool_catalog_size = 0;
  std::uint32_t wobj_catalog_size = 0;
  std::uint32_t project_catalog_size = 0;
  std::uint32_t register_catalog_size = 0;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
