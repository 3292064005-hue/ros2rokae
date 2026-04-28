#ifndef ROKAE_ERROR_CATEGORY_HPP
#define ROKAE_ERROR_CATEGORY_HPP

#include <string>
#include <system_error>

namespace rokae {

/// SDK-specific error codes for programmatic error discrimination.
/// These replace generic std::errc values with semantically meaningful SDK errors.
enum class SdkError : int {
  ok = 0,

  // Connection errors (100-199)
  not_connected = 100,
  already_connected = 101,
  connection_refused = 102,
  service_timeout = 103,
  service_unavailable = 104,

  // Motion errors (200-299)
  motion_rejected = 200,
  motion_queue_full = 201,
  motion_not_ready = 202,
  motion_in_progress = 203,
  trajectory_aborted = 204,
  trajectory_empty = 205,
  speed_out_of_range = 206,
  zone_out_of_range = 207,

  // Kinematics errors (300-399)
  ik_failed = 300,
  ik_no_valid_solution = 301,
  ik_singular_position = 302,
  fk_invalid_input = 303,
  joint_limit_exceeded = 304,
  soft_limit_exceeded = 305,
  cartesian_limit_exceeded = 306,

  // Safety errors (400-499)
  collision_detected = 400,
  emergency_stop = 401,
  power_off = 402,
  brake_engaged = 403,

  // IO errors (500-599)
  io_board_invalid = 500,
  io_port_invalid = 501,
  register_not_found = 502,
  register_type_mismatch = 503,

  // Model errors (600-699)
  model_unavailable = 600,
  model_invalid_input = 601,
  dynamics_computation_failed = 602,

  // RL/Program errors (700-799)
  project_not_found = 700,
  project_already_running = 701,
  project_not_running = 702,

  // Runtime errors (800-899)
  runtime_not_idle = 800,
  runtime_faulted = 801,
  shutdown_in_progress = 802,
  backend_unavailable = 803,
  owner_conflict = 804,

  // Calibration errors (900-999)
  calibration_insufficient_points = 900,
  calibration_failed = 901,

  // Path recording errors (1000-1099)
  path_not_found = 1000,
  path_recording_in_progress = 1001,
  path_recording_not_active = 1002,
  path_replay_failed = 1003,

  // Generic
  invalid_argument = 9000,
  internal_error = 9001,
  not_implemented = 9002,
};

/// Error category for SDK-specific error codes.
class SdkErrorCategory final : public std::error_category {
 public:
  [[nodiscard]] const char *name() const noexcept override {
    return "rokae_sdk";
  }

  [[nodiscard]] std::string message(int ev) const override {
    switch (static_cast<SdkError>(ev)) {
      case SdkError::ok: return "success";

      // Connection
      case SdkError::not_connected: return "robot not connected";
      case SdkError::already_connected: return "robot already connected";
      case SdkError::connection_refused: return "connection refused";
      case SdkError::service_timeout: return "service call timed out";
      case SdkError::service_unavailable: return "service unavailable";

      // Motion
      case SdkError::motion_rejected: return "motion command rejected";
      case SdkError::motion_queue_full: return "motion queue full";
      case SdkError::motion_not_ready: return "motion system not ready";
      case SdkError::motion_in_progress: return "motion already in progress";
      case SdkError::trajectory_aborted: return "trajectory execution aborted";
      case SdkError::trajectory_empty: return "trajectory is empty";
      case SdkError::speed_out_of_range: return "speed value out of valid range";
      case SdkError::zone_out_of_range: return "zone value out of valid range";

      // Kinematics
      case SdkError::ik_failed: return "inverse kinematics failed";
      case SdkError::ik_no_valid_solution: return "no valid IK solution found";
      case SdkError::ik_singular_position: return "target is at or near a singular position";
      case SdkError::fk_invalid_input: return "invalid input for forward kinematics";
      case SdkError::joint_limit_exceeded: return "joint position exceeds limits";
      case SdkError::soft_limit_exceeded: return "soft limit exceeded";
      case SdkError::cartesian_limit_exceeded: return "cartesian workspace limit exceeded";

      // Safety
      case SdkError::collision_detected: return "collision detected";
      case SdkError::emergency_stop: return "emergency stop active";
      case SdkError::power_off: return "robot power is off";
      case SdkError::brake_engaged: return "brake is engaged";

      // IO
      case SdkError::io_board_invalid: return "invalid IO board index";
      case SdkError::io_port_invalid: return "invalid IO port index";
      case SdkError::register_not_found: return "register key not found";
      case SdkError::register_type_mismatch: return "register type mismatch";

      // Model
      case SdkError::model_unavailable: return "model backend unavailable";
      case SdkError::model_invalid_input: return "invalid input for model computation";
      case SdkError::dynamics_computation_failed: return "dynamics computation failed";

      // RL
      case SdkError::project_not_found: return "RL project not found";
      case SdkError::project_already_running: return "RL project already running";
      case SdkError::project_not_running: return "RL project not running";

      // Runtime
      case SdkError::runtime_not_idle: return "runtime is not idle";
      case SdkError::runtime_faulted: return "runtime is in faulted state";
      case SdkError::shutdown_in_progress: return "shutdown is in progress";
      case SdkError::backend_unavailable: return "execution backend unavailable";
      case SdkError::owner_conflict: return "control owner conflict";

      // Calibration
      case SdkError::calibration_insufficient_points: return "insufficient points for calibration";
      case SdkError::calibration_failed: return "frame calibration failed";

      // Path
      case SdkError::path_not_found: return "recorded path not found";
      case SdkError::path_recording_in_progress: return "path recording already in progress";
      case SdkError::path_recording_not_active: return "no active path recording";
      case SdkError::path_replay_failed: return "path replay failed";

      // Generic
      case SdkError::invalid_argument: return "invalid argument";
      case SdkError::internal_error: return "internal error";
      case SdkError::not_implemented: return "feature not implemented";
    }
    return "unknown SDK error (" + std::to_string(ev) + ")";
  }

  [[nodiscard]] std::error_condition default_error_condition(int ev) const noexcept override {
    switch (static_cast<SdkError>(ev)) {
      case SdkError::ok:
        return {};
      case SdkError::not_connected:
      case SdkError::connection_refused:
        return std::errc::not_connected;
      case SdkError::service_timeout:
        return std::errc::timed_out;
      case SdkError::invalid_argument:
      case SdkError::fk_invalid_input:
      case SdkError::model_invalid_input:
        return std::errc::invalid_argument;
      case SdkError::joint_limit_exceeded:
      case SdkError::soft_limit_exceeded:
      case SdkError::cartesian_limit_exceeded:
      case SdkError::speed_out_of_range:
      case SdkError::zone_out_of_range:
        return std::errc::result_out_of_range;
      case SdkError::not_implemented:
        return std::errc::function_not_supported;
      default:
        return std::error_condition(ev, *this);
    }
  }
};

/// Singleton accessor for the SDK error category.
inline const SdkErrorCategory &sdk_error_category() noexcept {
  static const SdkErrorCategory instance;
  return instance;
}

/// Create a std::error_code from an SdkError.
inline std::error_code make_error_code(SdkError e) noexcept {
  return {static_cast<int>(e), sdk_error_category()};
}

/// Create a std::error_condition from an SdkError.
inline std::error_condition make_error_condition(SdkError e) noexcept {
  return {static_cast<int>(e), sdk_error_category()};
}

}  // namespace rokae

// Register rokae::SdkError as an error code enum so std::error_code
// can be implicitly constructed from it.
namespace std {
template <>
struct is_error_code_enum<rokae::SdkError> : true_type {};
}  // namespace std

#endif  // ROKAE_ERROR_CATEGORY_HPP
