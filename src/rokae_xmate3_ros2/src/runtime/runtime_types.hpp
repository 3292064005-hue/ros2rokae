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

enum class ExecutionState {
  idle,
  planning,
  queued,
  executing,
  settling,
  completed,
  failed,
  stopped,
};

struct MotionCommandSpec {
  MotionKind kind = MotionKind::none;
  std::vector<double> target_joints;
  std::vector<double> target_cartesian;
  std::vector<double> aux_cartesian;
  std::vector<int> requested_conf;
  int speed = 0;
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
  int default_speed = 50;
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
  int speed = 0;
  int zone = 0;
  std::vector<std::vector<double>> joint_trajectory;
  double trajectory_dt = 0.01;
};

struct MotionPlan {
  std::string request_id;
  std::vector<PlannedSegment> segments;
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

struct RuntimeStatus {
  std::string request_id;
  ExecutionState state = ExecutionState::idle;
  std::string message;
  std::size_t total_segments = 0;
  std::size_t completed_segments = 0;
  std::size_t current_segment_index = 0;
  bool terminal_success = false;
  std::uint64_t revision = 0;

  [[nodiscard]] bool terminal() const noexcept {
    return state == ExecutionState::completed || state == ExecutionState::failed ||
           state == ExecutionState::stopped;
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
    case ExecutionState::failed:
      return "failed";
    case ExecutionState::stopped:
      return "stopped";
    default:
      return "idle";
  }
}

inline bool is_active_state(ExecutionState state) noexcept {
  return state == ExecutionState::planning || state == ExecutionState::queued ||
         state == ExecutionState::executing || state == ExecutionState::settling;
}

inline bool is_terminal_state(ExecutionState state) noexcept {
  return state == ExecutionState::completed || state == ExecutionState::failed ||
         state == ExecutionState::stopped || state == ExecutionState::idle;
}

}  // namespace rokae_xmate3_ros2::runtime

#endif
