#ifndef ROKAE_XMATE3_ROS2_RUNTIME_TYPES_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_TYPES_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

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
  double preplanned_dt = 0.01;
};

struct MotionRequest {
  std::string request_id;
  std::vector<double> start_joints;
  std::vector<MotionCommandSpec> commands;
  double default_speed = 50.0;
  int default_zone = 0;
  bool strict_conf = false;
  bool avoid_singularity = true;
  bool soft_limit_enabled = false;
  std::array<std::array<double, 2>, 6> soft_limits{{
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
      {{-3.14, 3.14}},
  }};
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
  double trajectory_dt = 0.01;
  double trajectory_total_time = 0.0;
  PathFamily path_family = PathFamily::none;
  double path_length_m = 0.0;
  double path_entry_trim_m = 0.0;
  double path_exit_trim_m = 0.0;
  bool path_blended = false;
  double blend_length_m = 0.0;
};

struct MotionPlan {
  std::string request_id;
  std::vector<PlannedSegment> segments;
  std::vector<std::string> notes;
  std::string error_message;

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

  [[nodiscard]] bool busy() const noexcept {
    return has_request && !terminal;
  }
};

class BackendInterface {
 public:
  virtual ~BackendInterface() = default;

  [[nodiscard]] virtual RobotSnapshot readSnapshot() const = 0;
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
