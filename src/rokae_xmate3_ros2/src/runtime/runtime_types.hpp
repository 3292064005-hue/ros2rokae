#ifndef ROKAE_XMATE3_ROS2_RUNTIME_TYPES_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_TYPES_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "rokae_xmate3_ros2/runtime/runtime_contract.hpp"
#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"

namespace rokae_xmate3_ros2::runtime {

enum class MotionKind {
  none,
  move_absj,
  move_j,
  move_l,
  move_c,
  move_cf,
  move_sp,
};

enum class PathFamily {
  none,
  joint,
  cartesian_line,
  cartesian_arc,
  cartesian_continuous_circle,
  cartesian_spiral,
};

enum class ExecutionState {
  idle,
  planning,
  queued,
  executing,
  settling,
  completed,
  completed_relaxed,
  failed,
  stopped,
};

enum class ExecutionBackend {
  none,
  effort,
  jtc,
};

struct MotionCommandSpec {
  MotionKind kind = MotionKind::none;
  std::vector<double> target_joints;
  std::vector<double> target_cartesian;
  std::vector<double> aux_cartesian;
  std::vector<int> requested_conf;
  double speed = 0.0;
  int zone = 0;
  double angle = 0.0;
  double radius = 0.0;
  double radius_step = 0.0;
  bool direction = true;
  bool use_preplanned_trajectory = false;
  std::vector<std::vector<double>> preplanned_trajectory;
  std::vector<std::vector<double>> preplanned_velocity_trajectory;
  std::vector<std::vector<double>> preplanned_acceleration_trajectory;
  double preplanned_dt = 0.01;
};

struct MotionRequest {
  std::string request_id;
  std::vector<double> start_joints;
  std::vector<MotionCommandSpec> commands;
  double default_speed = 50.0;
  double speed_scale = 1.0;
  int default_zone = 0;
  bool strict_conf = false;
  bool avoid_singularity = true;
  bool soft_limit_enabled = false;
  std::array<std::array<double, 2>, 6> soft_limits = rokae_xmate3_ros2::spec::xmate3::kDefaultSoftLimits;
  double trajectory_dt = 0.01;
};

struct PlannedSegment {
  MotionKind kind = MotionKind::none;
  std::vector<double> target_joints;
  std::vector<double> target_cartesian;
  std::vector<double> aux_cartesian;
  double speed = 0.0;
  int zone = 0;
  bool blend_to_next = false;
  std::vector<std::vector<double>> joint_trajectory;
  std::vector<std::vector<double>> joint_velocity_trajectory;
  std::vector<std::vector<double>> joint_acceleration_trajectory;
  double trajectory_dt = 0.01;
  double trajectory_total_time = 0.0;
  PathFamily path_family = PathFamily::none;
  double path_length_m = 0.0;
  double path_entry_trim_m = 0.0;
  double path_exit_trim_m = 0.0;
  bool path_blended = false;
  double blend_length_m = 0.0;
  std::string planner_primary_backend{"kdl"};
  std::string planner_fallback_backend{"improved_dh"};
  std::string planner_selected_branch{"nearest_seed"};
  std::string planner_recommended_stop_point{"segment_start"};
  std::string retimer_note{"nominal"};
  double planner_branch_switch_risk = 0.0;
  double planner_singularity_risk = 0.0;
  double planner_continuity_risk = 0.0;
  double planner_soft_limit_risk = 0.0;
};

struct MotionPlan {
  std::string request_id;
  std::vector<PlannedSegment> segments;
  std::vector<std::string> notes;
  std::vector<std::string> degradation_chain;
  std::string error_message;
  std::string primary_backend{"kdl"};
  std::string fallback_backend{"improved_dh"};
  std::string selected_branch{"nearest_seed"};
  std::string recommended_stop_point{"segment_start"};
  std::string retimer_family{"unified"};
  std::string selection_policy{"risk_weighted"};
  std::string selected_candidate{"nominal"};
  std::vector<std::string> candidate_summaries;
  std::string explanation_summary;
  std::string dominant_motion_kind{"none"};
  double estimated_duration = 0.0;
  double branch_switch_risk = 0.0;
  double singularity_risk = 0.0;
  double continuity_risk = 0.0;
  double soft_limit_risk = 0.0;

  [[nodiscard]] bool valid() const noexcept {
    return error_message.empty() && !segments.empty();
  }
};

struct RobotSnapshot {
  std::array<double, 6> joint_position{};
  std::array<double, 6> joint_velocity{};
  std::array<double, 6> joint_torque{};
  bool power_on = false;
  bool drag_mode = false;
  bool collision_detection_enabled = false;
  bool soft_limit_enabled = false;
};

struct ControlCommand {
  std::array<double, 6> effort{};
  bool has_effort = false;
};

struct TrajectoryExecutionPoint {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
  double time_from_start = 0.0;
};

struct TrajectoryExecutionGoal {
  std::string request_id;
  std::vector<TrajectoryExecutionPoint> points;
  std::vector<double> segment_end_times;
  std::size_t total_segments = 0;
  std::size_t segment_index_offset = 0;
  double original_time_offset = 0.0;
  double original_time_scale = 1.0;
};

struct TrajectoryExecutionState {
  std::string request_id;
  bool active = false;
  bool accepted = false;
  bool completed = false;
  bool succeeded = false;
  bool canceled = false;
  bool failed = false;
  double desired_time_from_start = 0.0;
  std::vector<double> actual_position;
  std::vector<double> actual_velocity;
  std::vector<double> actual_acceleration;
  std::string message;
};

struct RuntimeStatus {
  std::string request_id;
  ExecutionState state = ExecutionState::idle;
  std::string message;
  std::size_t total_segments = 0;
  std::size_t completed_segments = 0;
  std::size_t current_segment_index = 0;
  bool terminal_success = false;
  std::uint64_t revision = 0;
  ExecutionBackend execution_backend = ExecutionBackend::none;
  ControlOwner control_owner = ControlOwner::none;
  RuntimePhase runtime_phase = RuntimePhase::idle;
  std::string last_event{"reset"};

  [[nodiscard]] bool terminal() const noexcept {
    return state == ExecutionState::completed || state == ExecutionState::completed_relaxed ||
           state == ExecutionState::failed || state == ExecutionState::stopped;
  }
};

struct RuntimeView {
  RuntimeStatus status;
  bool has_request = false;
  bool active_motion = false;
  bool can_accept_request = true;
  bool terminal = false;
  bool queue_initialized = false;
  bool queue_has_pending_commands = false;
  bool snapshot_fresh = true;
  bool nrt_ready = false;
  bool connected = false;
  bool power_on = false;
  bool drag_mode = false;
  int motion_mode = 0;
  std::uint64_t snapshot_age_ms = 0;
  std::string snapshot_stale_reason;

  [[nodiscard]] bool busy() const noexcept {
    return has_request && !terminal;
  }
};

class BackendInterface {
 public:
  virtual ~BackendInterface() = default;

  [[nodiscard]] virtual RobotSnapshot readSnapshot() const = 0;
  virtual void setControlOwner(ControlOwner owner) { (void)owner; }
  [[nodiscard]] virtual ControlOwner controlOwner() const { return ControlOwner::none; }
  virtual void applyControl(const ControlCommand &command) = 0;
  virtual void clearControl() = 0;
  virtual void setBrakeLock(const RobotSnapshot &snapshot, bool locked) {
    (void)snapshot;
    (void)locked;
  }
  [[nodiscard]] virtual bool brakesLocked() const { return false; }
  [[nodiscard]] virtual bool supportsTrajectoryExecution() const { return false; }
  virtual bool startTrajectoryExecution(const TrajectoryExecutionGoal &goal, std::string &message) {
    (void)goal;
    message = "trajectory execution backend unavailable";
    return false;
  }
  virtual void cancelTrajectoryExecution(const std::string &reason) { (void)reason; }
  [[nodiscard]] virtual TrajectoryExecutionState readTrajectoryExecutionState() const { return {}; }
};

inline const char *to_string(MotionKind kind) noexcept {
  switch (kind) {
    case MotionKind::move_absj:
      return "move_absj";
    case MotionKind::move_j:
      return "move_j";
    case MotionKind::move_l:
      return "move_l";
    case MotionKind::move_c:
      return "move_c";
    case MotionKind::move_cf:
      return "move_cf";
    case MotionKind::move_sp:
      return "move_sp";
    default:
      return "none";
  }
}

inline const char *to_string(PathFamily family) noexcept {
  switch (family) {
    case PathFamily::joint:
      return "joint";
    case PathFamily::cartesian_line:
      return "cartesian_line";
    case PathFamily::cartesian_arc:
      return "cartesian_arc";
    case PathFamily::cartesian_continuous_circle:
      return "cartesian_continuous_circle";
    case PathFamily::cartesian_spiral:
      return "cartesian_spiral";
    default:
      return "none";
  }
}

inline const char *to_string(ExecutionState state) noexcept {
  switch (state) {
    case ExecutionState::planning:
      return "planning";
    case ExecutionState::queued:
      return "queued";
    case ExecutionState::executing:
      return "executing";
    case ExecutionState::settling:
      return "settling";
    case ExecutionState::completed:
      return "completed";
    case ExecutionState::completed_relaxed:
      return "completed_relaxed";
    case ExecutionState::failed:
      return "failed";
    case ExecutionState::stopped:
      return "stopped";
    default:
      return "idle";
  }
}

inline const char *to_string(ExecutionBackend backend) noexcept {
  switch (backend) {
    case ExecutionBackend::effort:
      return "effort";
    case ExecutionBackend::jtc:
      return "jtc";
    default:
      return "none";
  }
}

inline bool is_active_state(ExecutionState state) noexcept {
  return state == ExecutionState::planning || state == ExecutionState::queued ||
         state == ExecutionState::executing || state == ExecutionState::settling;
}

inline bool is_terminal_state(ExecutionState state) noexcept {
  return state == ExecutionState::completed || state == ExecutionState::completed_relaxed ||
         state == ExecutionState::failed || state == ExecutionState::stopped ||
         state == ExecutionState::idle;
}

}  // namespace rokae_xmate3_ros2::runtime

#endif
