#ifndef ROKAE_SDK_SHIM_CORE_HPP
#define ROKAE_SDK_SHIM_CORE_HPP

#include <algorithm>
#include <any>
#include <array>
#include <atomic>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rokae/base.h"
#include "rokae/exception.h"
#include "rokae/data_types.h"
#include "rokae_xmate3_ros2/gazebo/model_facade.hpp"
#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "rokae_xmate3_ros2/robot.hpp"
#include "rokae_xmate3_ros2/utils.hpp"
#include "compat/rt_motion_primitives.hpp"
#include "runtime/rt_command_bridge.hpp"
#include "rokae_xmate3_ros2/runtime/rt_semantic_topics.hpp"

namespace rokae {

template <MotionControlMode ControlMode>
class MotionControl;

template <WorkType Wt, unsigned short DoF>
class RtMotionControl;

template <unsigned short DoF>
class RtMotionControlCobot;

template <unsigned short DoF>
class Model_T;

template <unsigned short DoF>
class xMateModel;

template <unsigned short DoF>
using XMateModel = xMateModel<DoF>;

template <WorkType Wt, unsigned short DoF>
class Robot_T;

template <unsigned short DoF>
class Cobot;

class xMateRobot;

enum class SegmentFrame : unsigned {
  joint1 = 1, joint2 = 2, joint3 = 3, joint4 = 4, joint5 = 5,
  joint6 = 6, joint7 = 7, flange = 8, endEffector = 9, stiffness = 10
};

namespace detail {

inline std::array<double, 16> identity_matrix_16() {
  return {1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0};
}

template <typename T>
struct is_vector : std::false_type {};

template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};

template <typename T>
inline constexpr bool is_vector_v = is_vector<T>::value;

inline std::string format_double(double value) {
  std::ostringstream oss;
  oss << std::setprecision(17) << value;
  return oss.str();
}

template <typename T>
inline std::string serialize_scalar(const T &value) {
  if constexpr (std::is_same_v<T, bool>) {
    return value ? "1" : "0";
  } else if constexpr (std::is_integral_v<T>) {
    return std::to_string(value);
  } else {
    return format_double(static_cast<double>(value));
  }
}

template <typename T>
inline bool deserialize_scalar(const std::string &text, T &value) {
  if constexpr (std::is_same_v<T, bool>) {
    if (text == "1" || text == "true" || text == "TRUE") {
      value = true;
      return true;
    }
    if (text == "0" || text == "false" || text == "FALSE") {
      value = false;
      return true;
    }
    return false;
  } else if constexpr (std::is_integral_v<T>) {
    try {
      value = static_cast<T>(std::stoll(text));
      return true;
    } catch (...) {
      return false;
    }
  } else {
    try {
      value = static_cast<T>(std::stod(text));
      return true;
    } catch (...) {
      return false;
    }
  }
}

inline std::string register_size_key(const std::string &name) {
  return name + ".__size";
}

inline std::string register_elem_key(const std::string &name, unsigned index) {
  return name + "[" + std::to_string(index) + "]";
}

inline std::string serialize_bool(bool value) {
  return value ? "1" : "0";
}

template <typename Container>
inline std::string serialize_numeric_container(const Container &values) {
  std::ostringstream oss;
  bool first = true;
  for (const auto &value : values) {
    if (!first) {
      oss << ',';
    }
    first = false;
    oss << format_double(static_cast<double>(value));
  }
  return oss.str();
}

inline std::string build_rt_payload(std::uint64_t seq, const std::array<double, 6> &values, bool finished = false) {
  std::ostringstream oss;
  oss << "seq=" << seq << ";finished=" << serialize_bool(finished)
      << ";values=" << serialize_numeric_container(values);
  return oss.str();
}

inline std::string build_rt_payload(std::uint64_t seq, const std::vector<double> &values, bool finished = false) {
  std::ostringstream oss;
  oss << "seq=" << seq << ";finished=" << serialize_bool(finished)
      << ";values=" << serialize_numeric_container(values);
  return oss.str();
}


template <std::size_t N>
inline std::array<double, N> vector_to_array(const std::vector<double> &values) {
  std::array<double, N> out{};
  const auto count = std::min<std::size_t>(N, values.size());
  std::copy_n(values.begin(), count, out.begin());
  return out;
}

inline bool publish_custom_data(const std::shared_ptr<RobotSession> &session,
                                const std::string &topic,
                                const std::string &payload,
                                error_code &ec) {
  if (!session || !session->robot) {
    ec = make_error_code(SdkError::not_connected);
    return false;
  }
  session->robot->sendCustomData(topic, payload, ec);
  remember_error(session, ec);
  return !ec;
}

inline void publish_rt_metadata(const std::shared_ptr<RobotSession> &session,
                                const std::string &dispatch_mode,
                                error_code &ec) {
  if (!session || !session->robot) {
    ec = make_error_code(SdkError::not_connected);
    return;
  }
  rokae_xmate3_ros2::runtime::rt_command_bridge::publishMetadata(*session->robot, dispatch_mode, ec);
  remember_error(session, ec);
}

inline bool publish_rt_joint_command(const std::shared_ptr<RobotSession> &session,
                                     const std::array<double, 6> &values,
                                     bool finished,
                                     error_code &ec) {
  if (!session || !session->robot) {
    ec = make_error_code(SdkError::not_connected);
    return false;
  }
  const auto ok = rokae_xmate3_ros2::runtime::rt_command_bridge::publishCommand(
      *session->robot,
      session->rt_command_sequence,
      rokae_xmate3_ros2::runtime::rt_command_bridge::CommandKind::JointPosition,
      values,
      finished,
      ec,
      "independent_rt");
  remember_error(session, ec);
  return ok;
}

inline bool publish_rt_cartesian_command(const std::shared_ptr<RobotSession> &session,
                                         const std::array<double, 6> &values,
                                         bool finished,
                                         error_code &ec) {
  if (!session || !session->robot) {
    ec = make_error_code(SdkError::not_connected);
    return false;
  }
  const auto ok = rokae_xmate3_ros2::runtime::rt_command_bridge::publishCommand(
      *session->robot,
      session->rt_command_sequence,
      rokae_xmate3_ros2::runtime::rt_command_bridge::CommandKind::CartesianPosition,
      values,
      finished,
      ec,
      "independent_rt");
  remember_error(session, ec);
  return ok;
}

inline bool publish_rt_torque_command(const std::shared_ptr<RobotSession> &session,
                                      const std::array<double, 6> &values,
                                      bool finished,
                                      error_code &ec) {
  if (!session || !session->robot) {
    ec = make_error_code(SdkError::not_connected);
    return false;
  }
  const auto ok = rokae_xmate3_ros2::runtime::rt_command_bridge::publishCommand(
      *session->robot,
      session->rt_command_sequence,
      rokae_xmate3_ros2::runtime::rt_command_bridge::CommandKind::Torque,
      values,
      finished,
      ec,
      "independent_rt");
  remember_error(session, ec);
  return ok;
}

inline std::array<double, 6> posture_to_array6(const CartesianPosition &pose) {
  return {pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz};
}

inline bool wait_until_joint_target(const std::shared_ptr<RobotSession> &session,
                                    const std::array<double, 6> &target,
                                    std::chrono::milliseconds timeout,
                                    double tolerance_rad,
                                    error_code &ec) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    std::array<double, 6> position{}, velocity{}, torque{};
    if (!session->robot->getRtJointData(position, velocity, torque, ec)) {
      return false;
    }
    bool reached = true;
    for (std::size_t i = 0; i < 6; ++i) {
      if (std::fabs(position[i] - target[i]) > tolerance_rad) {
        reached = false;
        break;
      }
    }
    if (reached) {
      ec.clear();
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ec = make_error_code(SdkError::service_timeout);
  return false;
}

inline bool wait_until_cartesian_target(const std::shared_ptr<RobotSession> &session,
                                        const std::array<double, 6> &target,
                                        std::chrono::milliseconds timeout,
                                        double translation_tolerance,
                                        double angular_tolerance,
                                        error_code &ec) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto pose = session->robot->cartPosture(CoordinateType::flangeInBase, ec);
    if (ec) {
      return false;
    }
    const auto current = posture_to_array6(pose);
    if (compat_rt::cartesianPoseWithinTolerance(current, target, translation_tolerance, angular_tolerance)) {
      ec.clear();
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ec = make_error_code(SdkError::service_timeout);
  return false;
}

inline Eigen::Matrix4d array16_to_matrix4d(const std::array<double, 16> &values) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      matrix(row, col) = values[static_cast<std::size_t>(row * 4 + col)];
    }
  }
  return matrix;
}

inline bool validate_rt_joint_start(const std::shared_ptr<RobotSession> &session,
                                    const std::array<double, 6> &start,
                                    double tolerance_rad,
                                    error_code &ec) {
  if (!session || !session->robot) {
    ec = make_error_code(SdkError::not_connected);
    return false;
  }
  std::array<double, 6> position{}, velocity{}, torque{};
  if (!session->robot->getRtJointData(position, velocity, torque, ec)) {
    return false;
  }
  for (std::size_t i = 0; i < start.size(); ++i) {
    if (std::fabs(position[i] - start[i]) > tolerance_rad) {
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
  }
  ec.clear();
  return true;
}

inline bool validate_rt_cartesian_start(const std::shared_ptr<RobotSession> &session,
                                        const std::array<double, 6> &start,
                                        double translation_tolerance,
                                        double angular_tolerance,
                                        error_code &ec) {
  if (!session || !session->robot) {
    ec = make_error_code(SdkError::not_connected);
    return false;
  }
  const auto pose = session->robot->cartPosture(CoordinateType::flangeInBase, ec);
  if (ec) {
    return false;
  }
  const auto current = posture_to_array6(pose);
  if (!compat_rt::cartesianPoseWithinTolerance(current, start, translation_tolerance, angular_tolerance)) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  ec.clear();
  return true;
}

inline bool cartesian_target_within_limit(const std::array<double, 6> &target,
                                          const std::array<double, 3> &lengths,
                                          const std::array<double, 16> &frame) {
  if (std::all_of(lengths.begin(), lengths.end(), [](double value) { return value <= 1e-9; })) {
    return true;
  }
  const Eigen::Matrix4d limit_frame = array16_to_matrix4d(frame);
  const Eigen::Vector4d point(target[0], target[1], target[2], 1.0);
  const Eigen::Vector4d local = limit_frame.inverse() * point;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double half_extent = std::max(0.0, lengths[axis]) * 0.5;
    if (std::fabs(local(static_cast<int>(axis))) > half_extent + 1e-9) {
      return false;
    }
  }
  return true;
}

inline std::string build_cartesian_limit_payload(const std::array<double, 3> &lengths,
                                                 const std::array<double, 16> &frame) {
  std::ostringstream oss;
  oss << "lengths=" << serialize_numeric_container(lengths)
      << ";frame=" << serialize_numeric_container(frame);
  return oss.str();
}

inline std::string build_load_payload(const Load &load) {
  std::ostringstream oss;
  oss << "mass=" << format_double(load.mass)
      << ";cog=" << serialize_numeric_container(load.cog)
      << ";inertia=" << serialize_numeric_container(load.inertia);
  return oss.str();
}

inline WorkToolInfo make_default_tool_info() {
  WorkToolInfo tool;
  tool.name = "tool0";
  tool.alias = "tool0";
  tool.robotHeld = true;
  tool.pos = Frame({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  tool.load = Load{};
  return tool;
}

struct RobotSession {
  explicit RobotSession(std::shared_ptr<::rokae::ros2::xMateRobot> robot_ptr)
      : robot(std::move(robot_ptr)) {
    tools.push_back(make_default_tool_info());
    previous_joint_velocity.fill(0.0);
    previous_joint_torque.fill(0.0);
    model_f_t_ee = {1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0};
    model_ee_t_k = model_f_t_ee;
    force_control_frame = model_f_t_ee;
  }

  std::shared_ptr<::rokae::ros2::xMateRobot> robot;
  mutable std::mutex mutex;
  std::error_code last_error_code;
  EventInfo last_move_event;
  EventInfo last_safety_event;
  EventCallback move_event_watcher;
  EventCallback safety_event_watcher;
  std::vector<std::string> state_fields;
  std::unordered_map<std::string, std::any> state_cache;
  std::chrono::steady_clock::duration state_interval{std::chrono::milliseconds(1)};
  std::array<double, 6> previous_joint_velocity{};
  std::array<double, 6> previous_joint_torque{};
  std::chrono::steady_clock::time_point previous_state_time{};
  bool has_previous_state = false;
  std::array<double, 6> previous_pose_abc{};
  std::array<double, 6> previous_pose_velocity{};
  bool has_previous_pose = false;
  std::string rt_state_plan_summary{"inactive"};
  bool rt_state_plan_rejected = false;
  int max_cache_size = 10;
  uint64_t next_cmd_id = 1;
  RLProjectInfo current_project;
  std::vector<RLProjectInfo> projects;
  std::vector<WorkToolInfo> tools;
  std::vector<WorkToolInfo> wobjs;
  Toolset toolset_cache{};
  Load model_load_cache{};
  std::array<double, 16> model_f_t_ee{};
  std::array<double, 16> model_ee_t_k{};
  std::array<double, 16> force_control_frame{};
  FrameType force_control_type = FrameType::world;
  xPanelOpt::Vout xpanel_vout = xPanelOpt::off;
  unsigned rt_network_tolerance = 0;
  bool use_rci_client = false;
  std::shared_ptr<RtMotionControlCobot<6>> rt_controller;
  std::string pending_move_id;
  int pending_move_count = 0;
  std::atomic<std::uint64_t> rt_command_sequence{1};
};

/**
 * @brief Create the default SDK-wrapper ROS client options.
 * @details Compatibility wrappers now default to strict runtime authority so
 *          catalog reads fail visibly unless the caller explicitly opts back
 *          into legacy cache fallback.
 * @return Normalized strict-authority client options.
 */
inline ::rokae::ros2::RosClientOptions make_compatibility_client_options() {
  ::rokae::ros2::RosClientOptions options;
  options.catalog_policy = ::rokae::ros2::strictRuntimeCatalogPolicy();
  return options;
}

inline std::shared_ptr<RobotSession> make_session() {
  auto options = make_compatibility_client_options();
  return std::make_shared<RobotSession>(std::make_shared<::rokae::ros2::xMateRobot>(options));
}

inline std::shared_ptr<RobotSession> make_session(const std::string &remote_ip, const std::string &local_ip = "") {
  auto options = make_compatibility_client_options();
  options.remote_ip = remote_ip;
  options.local_ip = local_ip;
  return std::make_shared<RobotSession>(std::make_shared<::rokae::ros2::xMateRobot>(options));
}

/**
 * @brief Normalize externally supplied SDK-wrapper ROS client options.
 * @param options Caller-provided options.
 * @return Options with a default strict runtime-authority catalog policy when
 *         no explicit override was supplied.
 * @details Legacy catalog fallback remains available only through an explicit
 *          policy override or the dedicated environment variable gate.
 */
inline ::rokae::ros2::RosClientOptions normalize_compatibility_client_options(::rokae::ros2::RosClientOptions options) {
  if (!options.catalog_policy.has_value()) {
    options.catalog_policy = ::rokae::ros2::strictRuntimeCatalogPolicy();
  }
  return options;
}

inline std::shared_ptr<RobotSession> make_session(const ::rokae::ros2::RosClientOptions &options) {
  auto normalized = normalize_compatibility_client_options(options);
  return std::make_shared<RobotSession>(std::make_shared<::rokae::ros2::xMateRobot>(normalized));
}

inline void remember_error(const std::shared_ptr<RobotSession> &session, const error_code &ec) {
  if (!session) {
    return;
  }
  std::lock_guard<std::mutex> lock(session->mutex);
  session->last_error_code = ec;
}

inline void publish_move_event(const std::shared_ptr<RobotSession> &session,
                               const std::string &cmd_id,
                               bool reached_target,
                               int waypoint_index,
                               const error_code &ec,
                               const std::string &remark) {
  EventCallback callback;
  {
    std::lock_guard<std::mutex> lock(session->mutex);
    session->last_move_event[EventInfoKey::MoveExecution::ID] = cmd_id;
    session->last_move_event[EventInfoKey::MoveExecution::ReachTarget] = reached_target;
    session->last_move_event[EventInfoKey::MoveExecution::WaypointIndex] = waypoint_index;
    session->last_move_event[EventInfoKey::MoveExecution::Error] = ec;
    session->last_move_event[EventInfoKey::MoveExecution::Remark] = remark;
    callback = session->move_event_watcher;
  }
  if (callback) {
    callback(session->last_move_event);
  }
}

inline void publish_safety_event(const std::shared_ptr<RobotSession> &session, bool collided) {
  EventCallback callback;
  {
    std::lock_guard<std::mutex> lock(session->mutex);
    session->last_safety_event[EventInfoKey::Safety::Collided] = collided;
    callback = session->safety_event_watcher;
  }
  if (callback) {
    callback(session->last_safety_event);
  }
}

inline int rt_speed_ratio_to_nrt_speed_mm_per_s(double speed_ratio) {
  const double clamped_ratio = std::clamp(speed_ratio, 0.01, 1.0);
  return std::clamp(static_cast<int>(std::lround(clamped_ratio * 4000.0)), 5, 4000);
}

inline bool wait_until_idle(const std::shared_ptr<RobotSession> &session,
                            error_code &ec,
                            std::chrono::milliseconds timeout = std::chrono::seconds(30)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto state = session->robot->operationState(ec);
    if (ec) {
      return false;
    }
    if (state == OperationState::idle || state == OperationState::jog || state == OperationState::drag) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ec = std::make_error_code(std::errc::timed_out);
  return false;
}

inline void clear_pending_move(const std::shared_ptr<RobotSession> &session) {
  if (!session) {
    return;
  }
  std::lock_guard<std::mutex> lock(session->mutex);
  session->pending_move_id.clear();
  session->pending_move_count = 0;
}

inline std::pair<std::string, int> pending_move_info(const std::shared_ptr<RobotSession> &session) {
  if (!session) {
    return {};
  }
  std::lock_guard<std::mutex> lock(session->mutex);
  return {session->pending_move_id, session->pending_move_count};
}

inline bool approx_equal_joint_command(const JointPosition &lhs, const JointPosition &rhs, double eps = 1e-3) {
  if (lhs.joints.size() != rhs.joints.size()) {
    return false;
  }
  for (size_t i = 0; i < lhs.joints.size(); ++i) {
    if (std::abs(lhs.joints[i] - rhs.joints[i]) > eps) {
      return false;
    }
  }
  return true;
}

inline bool approx_equal_cartesian_command(const CartesianPosition &lhs,
                                           const CartesianPosition &rhs,
                                           double pos_eps = 1e-4,
                                           double rot_eps = 1e-3) {
  return std::abs(lhs.x - rhs.x) <= pos_eps &&
         std::abs(lhs.y - rhs.y) <= pos_eps &&
         std::abs(lhs.z - rhs.z) <= pos_eps &&
         std::abs(lhs.rx - rhs.rx) <= rot_eps &&
         std::abs(lhs.ry - rhs.ry) <= rot_eps &&
         std::abs(lhs.rz - rhs.rz) <= rot_eps;
}

inline std::array<double, 6> to_array6(const CartesianPosition &pose) {
  return {pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz};
}

inline CartesianPosition from_array6(const std::array<double, 6> &values) {
  return CartesianPosition(values);
}

inline unsigned refresh_state_cache(const std::shared_ptr<RobotSession> &session, error_code &ec) {
  ec.clear();
  if (!session) {
    ec = std::make_error_code(std::errc::not_connected);
    return 0;
  }

  std::vector<std::string> fields;
  std::chrono::steady_clock::duration interval{std::chrono::milliseconds(1)};
  bool plan_rejected = false;
  {
    std::lock_guard<std::mutex> lock(session->mutex);
    fields = session->state_fields;
    interval = session->state_interval;
    plan_rejected = session->rt_state_plan_rejected;
    session->state_cache.clear();
  }
  if (plan_rejected) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    remember_error(session, ec);
    return 0;
  }
  if (fields.empty()) {
    return 0;
  }

  std::array<double, 6> joints{};
  std::array<double, 6> joint_vel{};
  std::array<double, 6> joint_tau{};
  if (!session->robot->getRtJointData(joints, joint_vel, joint_tau, ec)) {
    if (!ec) {
      ec = std::make_error_code(std::errc::io_error);
    }
    remember_error(session, ec);
    return 0;
  }

  const auto requires_pose_fields = std::any_of(fields.begin(), fields.end(), [](const std::string &field) {
    return field == RtSupportedFields::tcpPoseAbc_m ||
           field == RtSupportedFields::tcpPose_m ||
           field == RtSupportedFields::tcpPose_c ||
           field == RtSupportedFields::tcpVel_c ||
           field == RtSupportedFields::tcpAcc_c;
  });
  const auto requires_end_torque_fields = std::any_of(fields.begin(), fields.end(), [](const std::string &field) {
    return field == RtSupportedFields::tauExt_inBase || field == RtSupportedFields::tauExt_inStiff;
  });

  CartesianPosition pose{};
  std::array<double, 16> pose_matrix{};
  std::array<double, 6> pose_abc{};
  if (requires_pose_fields) {
    pose = session->robot->cartPosture(CoordinateType::flangeInBase, ec);
    if (ec) {
      remember_error(session, ec);
      return 0;
    }
    pose_matrix = Utils::postureToMatrix(pose);
    pose_abc = {pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz};
  }

  std::array<double, 6> end_torque{};
  if (requires_end_torque_fields) {
    end_torque = session->robot->getEndEffectorTorque(ec);
    if (ec) {
      remember_error(session, ec);
      return 0;
    }
  }

  const auto now = std::chrono::steady_clock::now();
  const double interval_dt = std::max(1e-6, std::chrono::duration<double>(interval).count());
  std::array<double, 6> joint_acc{};
  std::array<double, 6> tau_vel{};
  std::array<double, 6> tcp_vel{};
  std::array<double, 6> tcp_acc{};
  {
    std::lock_guard<std::mutex> lock(session->mutex);
    const double derived_dt = session->has_previous_state
                                ? std::max(1e-6, std::chrono::duration<double>(now - session->previous_state_time).count())
                                : interval_dt;
    if (session->has_previous_state) {
      for (size_t i = 0; i < 6; ++i) {
        joint_acc[i] = (joint_vel[i] - session->previous_joint_velocity[i]) / derived_dt;
        tau_vel[i] = (joint_tau[i] - session->previous_joint_torque[i]) / derived_dt;
      }
    }
    if (requires_pose_fields && session->has_previous_pose) {
      for (size_t i = 0; i < 6; ++i) {
        tcp_vel[i] = (pose_abc[i] - session->previous_pose_abc[i]) / derived_dt;
        tcp_acc[i] = (tcp_vel[i] - session->previous_pose_velocity[i]) / derived_dt;
      }
    }

    session->previous_joint_velocity = joint_vel;
    session->previous_joint_torque = joint_tau;
    if (requires_pose_fields) {
      session->previous_pose_abc = pose_abc;
      session->previous_pose_velocity = tcp_vel;
      session->has_previous_pose = true;
    }
    session->previous_state_time = now;
    session->has_previous_state = true;

    const std::array<double, 6> zero6{};
    for (const auto &field : fields) {
      if (field == RtSupportedFields::jointPos_m || field == RtSupportedFields::jointPos_c || field == RtSupportedFields::theta_m) {
        session->state_cache[field] = joints;
      } else if (field == RtSupportedFields::jointVel_m || field == RtSupportedFields::jointVel_c || field == RtSupportedFields::thetaVel_m) {
        session->state_cache[field] = joint_vel;
      } else if (field == RtSupportedFields::jointAcc_c) {
        session->state_cache[field] = joint_acc;
      } else if (field == RtSupportedFields::tau_m || field == RtSupportedFields::tau_c || field == RtSupportedFields::tauFiltered_m ||
                 field == RtSupportedFields::motorTau || field == RtSupportedFields::motorTauFiltered) {
        session->state_cache[field] = joint_tau;
      } else if (field == RtSupportedFields::tcpPoseAbc_m) {
        session->state_cache[field] = pose_abc;
      } else if (field == RtSupportedFields::tcpPose_m || field == RtSupportedFields::tcpPose_c) {
        session->state_cache[field] = pose_matrix;
      } else if (field == RtSupportedFields::tcpVel_c) {
        session->state_cache[field] = tcp_vel;
      } else if (field == RtSupportedFields::tcpAcc_c) {
        session->state_cache[field] = tcp_acc;
      } else if (field == RtSupportedFields::tauVel_c) {
        session->state_cache[field] = tau_vel;
      } else if (field == RtSupportedFields::tauExt_inBase || field == RtSupportedFields::tauExt_inStiff) {
        session->state_cache[field] = end_torque;
      } else {
        session->state_cache[field] = zero6;
      }
    }
    session->last_error_code = ec;
    return static_cast<unsigned>(session->state_cache.size());
  }
}

template <typename Command>
inline void append_command(::rokae::ros2::xMateRobot &robot, const Command &cmd, error_code &ec);

template <>
inline void append_command(::rokae::ros2::xMateRobot &robot, const MoveAbsJCommand &cmd, error_code &ec) {
  robot.moveAbsJ(cmd, ec);
}

template <>
inline void append_command(::rokae::ros2::xMateRobot &robot, const MoveJCommand &cmd, error_code &ec) {
  robot.moveJ(cmd, ec);
}

template <>
inline void append_command(::rokae::ros2::xMateRobot &robot, const MoveLCommand &cmd, error_code &ec) {
  robot.moveL(cmd, ec);
}

template <>
inline void append_command(::rokae::ros2::xMateRobot &robot, const MoveCCommand &cmd, error_code &ec) {
  robot.moveC(cmd, ec);
}

template <>
inline void append_command(::rokae::ros2::xMateRobot &robot, const MoveCFCommand &cmd, error_code &ec) {
  robot.moveCF(cmd, ec);
}

template <>
inline void append_command(::rokae::ros2::xMateRobot &robot, const MoveSPCommand &cmd, error_code &ec) {
  robot.moveSP(cmd, ec);
}

template <typename T>
inline void write_register_value(::rokae::ros2::xMateRobot &robot,
                                 const std::string &name,
                                 unsigned index,
                                 const T &value,
                                 error_code &ec) {
  robot.writeRegister(register_elem_key(name, index), serialize_scalar(value), ec);
}

template <typename T>
inline bool read_register_value(::rokae::ros2::xMateRobot &robot,
                                const std::string &name,
                                unsigned index,
                                T &value,
                                error_code &ec) {
  const auto raw = robot.readRegister(register_elem_key(name, index), ec);
  if (ec) {
    return false;
  }
  if (!deserialize_scalar(raw, value)) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  return true;
}

}  // namespace detail

class XCORE_API BaseMotionControl : public Base<BaseMotionControl> {
public:
  BaseMotionControl() = default;
  explicit BaseMotionControl(std::shared_ptr<detail::RobotSession> session) : session_(std::move(session)) {}
  virtual ~BaseMotionControl() = default;

protected:
  std::shared_ptr<detail::RobotSession> session_;
};

template <>
class XCORE_API MotionControl<MotionControlMode::RtCommand> : public BaseMotionControl {
public:
  MotionControl() = default;
  explicit MotionControl(std::shared_ptr<detail::RobotSession> session)
      : BaseMotionControl(std::move(session)) {}
  virtual ~MotionControl() {
    stopLoop();
  }

  void disconnectNetwork() noexcept {
    stopLoop();
    stopMove();
    if (session_) {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->state_fields.clear();
      session_->state_cache.clear();
      session_->has_previous_state = false;
    }
  }

  void reconnectNetwork(error_code &ec) noexcept {
    if (!session_) {
      ec = std::make_error_code(std::errc::not_connected);
      return;
    }
    ec.clear();
    detail::remember_error(session_, ec);
  }

  // Gazebo shim note: this preserves the SDK RT API surface while dispatching to an
  // explicit runtime-owned RT command channel instead of reusing the NRT move queue.
  template <class Command>
  void setControlLoop(const std::function<Command(void)> &callback, int priority = 0, bool useStateDataInLoop = false) noexcept {
    (void)priority;
    has_last_joint_command_ = false;
    has_last_cartesian_command_ = false;
    last_dispatch_time_ = std::chrono::steady_clock::time_point{};
    control_loop_ = [callback]() { return std::any(callback()); };
    loop_dispatch_ = [this](const std::any &value) {
      if (!session_) {
        return;
      }
      const auto now = std::chrono::steady_clock::now();
      if (last_dispatch_time_.time_since_epoch().count() != 0 &&
          now - last_dispatch_time_ < dispatch_interval_) {
        return;
      }
      error_code ec;
      if (value.type() == typeid(JointPosition)) {
        const auto &command = std::any_cast<const JointPosition &>(value);
        const std::array<double, 6> target = detail::vector_to_array<6>(command.joints);
        if (!command.isFinished()) {
          if (has_last_joint_command_ && detail::approx_equal_joint_command(command, last_joint_command_)) {
            return;
          }
          detail::publish_rt_joint_command(session_, target, false, ec);
          if (!ec) {
            last_dispatch_time_ = now;
            last_joint_command_ = command;
            has_last_joint_command_ = true;
          }
        } else {
          detail::publish_rt_joint_command(session_, target, true, ec);
        }
      } else if (value.type() == typeid(CartesianPosition)) {
        const auto &command = std::any_cast<const CartesianPosition &>(value);
        const auto target = detail::posture_to_array6(command);
        if (!command.isFinished()) {
          if (has_last_cartesian_command_ &&
              detail::approx_equal_cartesian_command(command, last_cartesian_command_)) {
            return;
          }
          detail::publish_rt_cartesian_command(session_, target, false, ec);
          if (!ec) {
            last_dispatch_time_ = now;
            last_cartesian_command_ = command;
            has_last_cartesian_command_ = true;
          }
        } else {
          detail::publish_rt_cartesian_command(session_, target, true, ec);
        }
      } else if (value.type() == typeid(Torque)) {
        const auto &command = std::any_cast<const Torque &>(value);
        const std::array<double, 6> target = detail::vector_to_array<6>(command.tau);
        detail::publish_rt_torque_command(session_, target, command.isFinished() != 0, ec);
        if (!ec) {
          last_dispatch_time_ = now;
        }
      }
      detail::remember_error(session_, ec);
    };
    use_state_data_in_loop_ = useStateDataInLoop;
  }

  void startLoop(bool blocking = true) {
    if (!control_loop_) {
      return;
    }
    auto body = [this]() {
      while (loop_running_) {
        if (use_state_data_in_loop_ && session_) {
          error_code refresh_ec;
          detail::refresh_state_cache(session_, refresh_ec);
        }
        const std::any command = control_loop_();
        if (loop_dispatch_) {
          loop_dispatch_(command);
        }
        bool finished = false;
        if (command.type() == typeid(JointPosition)) {
          finished = std::any_cast<const JointPosition &>(command).isFinished() != 0;
        } else if (command.type() == typeid(CartesianPosition)) {
          finished = std::any_cast<const CartesianPosition &>(command).isFinished() != 0;
        } else if (command.type() == typeid(Torque)) {
          finished = std::any_cast<const Torque &>(command).isFinished() != 0;
        }
        if (finished) {
          stopMove();
          loop_running_ = false;
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    };
    if (blocking) {
      stopLoop();
      loop_running_ = true;
      body();
    } else {
      stopLoop();
      loop_running_ = true;
      loop_thread_ = std::thread(body);
    }
  }

  void stopLoop() {
    loop_running_ = false;
    if (loop_thread_.joinable()) {
      loop_thread_.join();
    }
  }

  void stopMove() {
    if (!session_) {
      return;
    }
    error_code ec;
    rokae_xmate3_ros2::runtime::rt_command_bridge::publishStop(*session_->robot, ec);
    detail::remember_error(session_, ec);
  }

  void automaticErrorRecovery(error_code &ec) noexcept {
    if (!session_) {
      ec = std::make_error_code(std::errc::not_connected);
      return;
    }
    session_->robot->clearServoAlarm(ec);
    if (!ec) {
      session_->robot->moveReset(ec);
    }
    if (!ec && session_->robot->powerState(ec) != PowerState::on) {
      session_->robot->setPowerState(true, ec);
    }
    detail::remember_error(session_, ec);
  }

protected:
  std::function<std::any(void)> control_loop_;
  std::function<void(const std::any &)> loop_dispatch_;
  std::thread loop_thread_;
  std::atomic<bool> loop_running_{false};
  bool use_state_data_in_loop_ = false;
  std::chrono::steady_clock::time_point last_dispatch_time_{};
  std::chrono::milliseconds dispatch_interval_{1};
  bool has_last_joint_command_ = false;
  JointPosition last_joint_command_;
  bool has_last_cartesian_command_ = false;
  CartesianPosition last_cartesian_command_;
};

template <WorkType Wt, unsigned short DoF>
class XCORE_API RtMotionControl : public MotionControl<MotionControlMode::RtCommand> {
public:
  RtMotionControl() = default;
  explicit RtMotionControl(std::shared_ptr<detail::RobotSession> session)
      : MotionControl<MotionControlMode::RtCommand>(std::move(session)) {}

  /**
   * @brief Arm the runtime RT channel for a specific control mode.
   * @param rtMode Requested controller-side RT mode.
   * @throws no-throw; errors are reported through the session error state.
   * @details The call validates the requested mode transition through the runtime facade and
   *          publishes the semantic dispatch metadata consumed by the Gazebo RT bridge.
   */
  void startMove(RtControllerMode rtMode) {
    if (!this->session_) {
      return;
    }
    error_code ec;
    this->session_->robot->setRtControlMode(rtMode, ec);
    if (!ec) {
      detail::publish_rt_metadata(this->session_, "independent_rt", ec);
    }
    detail::remember_error(this->session_, ec);
  }

  bool setFilterLimit(bool limit_rate, double cutoff_frequency) noexcept {
    filter_limit_enabled_ = limit_rate;
    filter_cutoff_frequency_ = cutoff_frequency;
    return true;
  }

  /**
   * @brief Configure the Cartesian safety box consumed by the RT bridge.
   * @param lengths Box dimensions along XYZ expressed in metres.
   * @param frame Homogeneous transform of the box centre in base coordinates.
   * @param ec Receives invalid_argument when any extent is negative and not finite.
   * @details The configuration is persisted locally and published to the runtime semantic store.
   */
  void setCartesianLimit(const std::array<double, 3> &lengths, const std::array<double, 16> &frame, error_code &ec) noexcept {
    if (!std::all_of(lengths.begin(), lengths.end(), [](double value) { return std::isfinite(value) && value >= 0.0; })) {
      ec = std::make_error_code(std::errc::invalid_argument);
      detail::remember_error(this->session_, ec);
      return;
    }
    cartesian_limit_lengths_ = lengths;
    cartesian_limit_frame_ = frame;
    ec.clear();
    detail::publish_custom_data(this->session_, rokae_xmate3_ros2::runtime::rt_topics::kConfigCartesianLimit,
                                detail::build_cartesian_limit_payload(lengths, frame), ec);
  }

  /**
   * @brief Configure the TCP transform used by RT Cartesian commands.
   * @param frame Homogeneous transform from flange to TCP.
   * @param ec Error state propagated to the caller.
   */
  void setEndEffectorFrame(const std::array<double, 16> &frame, error_code &ec) noexcept {
    if (!std::all_of(frame.begin(), frame.end(), [](double value) { return std::isfinite(value); })) {
      ec = std::make_error_code(std::errc::invalid_argument);
      detail::remember_error(this->session_, ec);
      return;
    }
    end_effector_frame_ = frame;
    {
      std::lock_guard<std::mutex> lock(this->session_->mutex);
      this->session_->model_f_t_ee = frame;
      this->session_->toolset_cache.end = Frame(frame);
    }
    ec.clear();
    detail::publish_custom_data(this->session_, rokae_xmate3_ros2::runtime::rt_topics::kConfigEndEffectorFrame,
                                detail::serialize_numeric_container(frame), ec);
  }

  /**
   * @brief Configure payload data consumed by RT model approximations.
   * @param load Payload description expressed in SI units.
   * @param ec Error state propagated to the caller.
   */
  void setLoad(const Load &load, error_code &ec) noexcept {
    if (!std::isfinite(load.mass) || load.mass < 0.0 ||
        !std::all_of(load.cog.begin(), load.cog.end(), [](double value) { return std::isfinite(value); }) ||
        !std::all_of(load.inertia.begin(), load.inertia.end(), [](double value) { return std::isfinite(value) && value >= 0.0; })) {
      ec = std::make_error_code(std::errc::invalid_argument);
      detail::remember_error(this->session_, ec);
      return;
    }
    load_ = load;
    {
      std::lock_guard<std::mutex> lock(this->session_->mutex);
      this->session_->model_load_cache = load;
      this->session_->toolset_cache.load = load;
    }
    ec.clear();
    detail::publish_custom_data(this->session_, rokae_xmate3_ros2::runtime::rt_topics::kConfigLoad,
                                detail::build_load_payload(load), ec);
  }

  /**
   * @brief Execute an RT joint move using the independent RT semantic channel.
   * @param speed Normalised speed factor in (0, 1].
   * @param start Expected robot joint state at command start.
   * @param target Target joint position.
   * @throws RealtimeMotionException Raised when the runtime rejects the command or the robot never reaches the target.
   * @details The shim now validates that @p start matches the live joint state before publishing the RT command.
   */
  void MoveJ(double speed, const std::array<double, DoF> &start, const std::array<double, DoF> &target) {
    if (!this->session_) {
      return;
    }
    if (!compat_rt::isSpeedFactorValid(speed)) {
      throw RealtimeParameterException("RtMotionControl::MoveJ invalid speed",
                                       std::make_error_code(std::errc::invalid_argument));
    }
    const auto timeout = std::chrono::milliseconds(static_cast<int>(std::clamp(30.0 / std::max(speed, 0.05), 5.0, 60.0) * 1000.0));
    error_code ec;
    if constexpr (DoF == 6) {
      const auto start6 = detail::vector_to_array<6>(std::vector<double>(start.begin(), start.end()));
      if (!detail::validate_rt_joint_start(this->session_, start6, 1e-2, ec)) {
        detail::remember_error(this->session_, ec);
        throw_if_error<RealtimeParameterException>(ec, "RtMotionControl::MoveJ start validation failed");
        return;
      }
      detail::publish_rt_joint_command(this->session_, detail::vector_to_array<6>(std::vector<double>(target.begin(), target.end())), false, ec);
      if (!ec) {
        detail::wait_until_joint_target(this->session_, detail::vector_to_array<6>(std::vector<double>(target.begin(), target.end())), timeout, 2e-2, ec);
      }
    }
    detail::remember_error(this->session_, ec);
    throw_if_error<RealtimeMotionException>(ec, "RtMotionControl::MoveJ");
  }

  /**
   * @brief Execute an RT Cartesian line move.
   * @param speed Normalised speed factor in (0, 1].
   * @param start Expected flange/TCP pose at command start.
   * @param target Target pose.
   * @throws RealtimeMotionException Raised when start validation, safety-box validation or execution fails.
   */
  void MoveL(double speed, CartesianPosition &start, CartesianPosition &target) {
    if (!this->session_) {
      return;
    }
    if (!compat_rt::isSpeedFactorValid(speed)) {
      throw RealtimeParameterException("RtMotionControl::MoveL invalid speed",
                                       std::make_error_code(std::errc::invalid_argument));
    }
    error_code ec;
    const auto start_pose = detail::posture_to_array6(start);
    const auto target_pose = detail::posture_to_array6(target);
    if (!detail::validate_rt_cartesian_start(this->session_, start_pose, 3e-2, 6e-2, ec)) {
      detail::remember_error(this->session_, ec);
      throw_if_error<RealtimeParameterException>(ec, "RtMotionControl::MoveL start validation failed");
      return;
    }
    if (!detail::cartesian_target_within_limit(target_pose, cartesian_limit_lengths_, cartesian_limit_frame_)) {
      ec = std::make_error_code(std::errc::result_out_of_range);
      detail::remember_error(this->session_, ec);
      throw_if_error<RealtimeParameterException>(ec, "RtMotionControl::MoveL cartesian limit violated");
      return;
    }
    detail::publish_rt_cartesian_command(this->session_, target_pose, false, ec);
    if (!ec) {
      const auto timeout = std::chrono::milliseconds(static_cast<int>(std::clamp(30.0 / std::max(speed, 0.05), 5.0, 60.0) * 1000.0));
      detail::wait_until_cartesian_target(this->session_, target_pose, timeout, 3e-2, 6e-2, ec);
    }
    detail::remember_error(this->session_, ec);
    throw_if_error<RealtimeMotionException>(ec, "RtMotionControl::MoveL");
  }

  /**
   * @brief Execute an RT circular move using a verified geometric circle fit.
   * @param speed Normalised speed factor in (0, 1].
   * @param start Expected start pose.
   * @param aux Auxiliary point defining the arc.
   * @param target Arc end pose.
   * @throws RealtimeMotionException Raised on invalid arc geometry, start mismatch or execution failure.
   */
  void MoveC(double speed, CartesianPosition &start, CartesianPosition &aux, CartesianPosition &target) {
    if (!this->session_) {
      return;
    }
    if (!compat_rt::isSpeedFactorValid(speed)) {
      throw RealtimeParameterException("RtMotionControl::MoveC invalid speed",
                                       std::make_error_code(std::errc::invalid_argument));
    }
    error_code ec;
    const auto start_pose = detail::posture_to_array6(start);
    const auto aux_pose = detail::posture_to_array6(aux);
    const auto target_pose = detail::posture_to_array6(target);
    if (!detail::validate_rt_cartesian_start(this->session_, start_pose, 4e-2, 8e-2, ec)) {
      detail::remember_error(this->session_, ec);
      throw_if_error<RealtimeParameterException>(ec, "RtMotionControl::MoveC start validation failed");
      return;
    }
    if (!detail::cartesian_target_within_limit(target_pose, cartesian_limit_lengths_, cartesian_limit_frame_) ||
        !detail::cartesian_target_within_limit(aux_pose, cartesian_limit_lengths_, cartesian_limit_frame_)) {
      ec = std::make_error_code(std::errc::result_out_of_range);
      detail::remember_error(this->session_, ec);
      throw_if_error<RealtimeParameterException>(ec, "RtMotionControl::MoveC cartesian limit violated");
      return;
    }
    const auto geometry = compat_rt::fitCircularArc(start_pose, aux_pose, target_pose, ec);
    if (!geometry) {
      detail::remember_error(this->session_, ec);
      const Eigen::Vector3d p1(start_pose[0], start_pose[1], start_pose[2]);
      const Eigen::Vector3d p2(aux_pose[0], aux_pose[1], aux_pose[2]);
      const Eigen::Vector3d p3(target_pose[0], target_pose[1], target_pose[2]);
      if ((p2 - p1).cross(p3 - p1).squaredNorm() < 1e-10) {
        throw_if_error<RealtimeParameterException>(ec, "RtMotionControl::MoveC degenerate arc");
      }
      throw_if_error<RealtimeParameterException>(ec, "RtMotionControl::MoveC invalid radius");
      return;
    }
    const int steps = compat_rt::estimateArcInterpolationSteps(speed, geometry->radius, geometry->sweep_angle);
    const Eigen::Quaterniond q_start = rokae_xmate3_ros2::runtime::pose_utils::rpyToQuaternion(start_pose[3], start_pose[4], start_pose[5]);
    const Eigen::Quaterniond q_target = rokae_xmate3_ros2::runtime::pose_utils::rpyToQuaternion(target_pose[3], target_pose[4], target_pose[5]);
    const Eigen::Vector3d start_point(start_pose[0], start_pose[1], start_pose[2]);
    for (int i = 1; i <= steps && !ec; ++i) {
      const double alpha = static_cast<double>(i) / static_cast<double>(steps);
      const Eigen::AngleAxisd rot(alpha * geometry->sweep_angle, geometry->axis);
      const Eigen::Vector3d point = geometry->center + rot * (start_point - geometry->center);
      const Eigen::Quaterniond q = q_start.slerp(alpha, q_target);
      const auto pose = compat_rt::quaternionToPose(q, point);
      if (!detail::cartesian_target_within_limit(pose, cartesian_limit_lengths_, cartesian_limit_frame_)) {
        ec = std::make_error_code(std::errc::result_out_of_range);
        break;
      }
      detail::publish_rt_cartesian_command(this->session_, pose, i == steps, ec);
      if (!ec) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    if (!ec) {
      const auto timeout = std::chrono::milliseconds(static_cast<int>(std::clamp(40.0 / std::max(speed, 0.05), 8.0, 80.0) * 1000.0));
      detail::wait_until_cartesian_target(this->session_, target_pose, timeout, 4e-2, 8e-2, ec);
    }
    detail::remember_error(this->session_, ec);
    throw_if_error<RealtimeMotionException>(ec, "RtMotionControl::MoveC");
  }


protected:
  bool filter_limit_enabled_ = false;
  double filter_cutoff_frequency_ = 0.0;
  std::array<double, 3> cartesian_limit_lengths_{};
  std::array<double, 16> cartesian_limit_frame_{};
  std::array<double, 16> end_effector_frame_{};
  Load load_{};
};

template <unsigned short DoF>
class XCORE_API RtMotionControlCobot : public RtMotionControl<WorkType::collaborative, DoF> {
public:
  RtMotionControlCobot() = default;
  explicit RtMotionControlCobot(std::shared_ptr<detail::RobotSession> session)
      : RtMotionControl<WorkType::collaborative, DoF>(std::move(session)) {}

  void setJointImpedance(const std::array<double, DoF> &factor, error_code &ec) noexcept {
    joint_impedance_ = factor;
    ec.clear();
    if constexpr (DoF == 6) {
      detail::publish_custom_data(this->session_, rokae_xmate3_ros2::runtime::rt_topics::kConfigJointImpedance,
                                  detail::serialize_numeric_container(factor), ec);
    }
  }

  void setCartesianImpedance(const std::array<double, 6> &factor, error_code &ec) noexcept {
    cartesian_impedance_ = factor;
    ec.clear();
    detail::publish_custom_data(this->session_, rokae_xmate3_ros2::runtime::rt_topics::kConfigCartesianImpedance,
                                detail::serialize_numeric_container(factor), ec);
  }

  void setFilterFrequency(double jointFrequency, double cartesianFrequency, double torqueFrequency, error_code &ec) noexcept {
    filter_frequencies_ = {jointFrequency, cartesianFrequency, torqueFrequency};
    ec.clear();
    detail::publish_custom_data(this->session_, rokae_xmate3_ros2::runtime::rt_topics::kConfigFilterFrequency,
                                detail::serialize_numeric_container(filter_frequencies_), ec);
  }

  void setCollisionBehaviour(const std::array<double, DoF> &torqueThresholds, error_code &ec) noexcept {
    collision_thresholds_ = torqueThresholds;
    if (!this->session_) {
      ec = std::make_error_code(std::errc::not_connected);
      return;
    }
    std::array<double, 6> sensitivity{};
    constexpr std::array<double, 6> default_max{{75.0, 75.0, 45.0, 30.0, 30.0, 20.0}};
    for (size_t i = 0; i < DoF && i < 6; ++i) {
      const double threshold = std::max(1e-6, std::abs(torqueThresholds[i]));
      const double normalized = default_max[i] / threshold;
      sensitivity[i] = std::clamp(normalized, 0.01, 2.0);
    }
    this->session_->robot->enableCollisionDetection(sensitivity, StopLevel::stop1, 0.0, ec);
    detail::remember_error(this->session_, ec);
  }

  void setCartesianImpedanceDesiredTorque(const std::array<double, 6> &torque, error_code &ec) noexcept {
    desired_cartesian_torque_ = torque;
    ec.clear();
    detail::publish_custom_data(this->session_, rokae_xmate3_ros2::runtime::rt_topics::kConfigCartesianDesiredWrench,
                                detail::serialize_numeric_container(torque), ec);
  }

  void setTorqueFilterCutOffFrequency(double frequency, error_code &ec) noexcept {
    torque_cutoff_frequency_ = frequency;
    ec.clear();
    detail::publish_custom_data(this->session_, rokae_xmate3_ros2::runtime::rt_topics::kConfigTorqueCutoffFrequency,
                                detail::format_double(frequency), ec);
  }

  void setFcCoor(const std::array<double, 16> &frame, FrameType type, error_code &ec) noexcept {
    if (type != FrameType::world && type != FrameType::tool && type != FrameType::path) {
      ec = std::make_error_code(std::errc::invalid_argument);
      detail::remember_error(this->session_, ec);
      return;
    }
    if (!std::all_of(frame.begin(), frame.end(), [](double value) { return std::isfinite(value); })) {
      ec = std::make_error_code(std::errc::invalid_argument);
      detail::remember_error(this->session_, ec);
      return;
    }
    fc_frame_ = frame;
    fc_type_ = type;
    {
      std::lock_guard<std::mutex> lock(this->session_->mutex);
      this->session_->force_control_frame = frame;
      this->session_->force_control_type = type;
    }
    ec.clear();
    std::ostringstream payload;
    payload << "type=" << static_cast<int>(type) << ";values=" << detail::serialize_numeric_container(frame);
    detail::publish_custom_data(this->session_, rokae_xmate3_ros2::runtime::rt_topics::kConfigForceControlFrame,
                                payload.str(), ec);
  }

private:
  std::array<double, DoF> joint_impedance_{};
  std::array<double, DoF> collision_thresholds_{};
  std::array<double, 6> cartesian_impedance_{};
  std::array<double, 3> filter_frequencies_{};
  std::array<double, 6> desired_cartesian_torque_{};
  double torque_cutoff_frequency_ = 0.0;
  std::array<double, 16> fc_frame_{};
  FrameType fc_type_ = FrameType::world;
};

class XCORE_API BaseModel : public Base<BaseModel> {
public:
  BaseModel() = default;
  explicit BaseModel(std::shared_ptr<detail::RobotSession> session) : session_(std::move(session)) {}
  virtual ~BaseModel() = default;

  std::array<double, 6> baseFrame(error_code &ec) const noexcept {
    return session_->robot->baseFrame(ec);
  }

  Toolset toolset(error_code &ec) const noexcept {
    auto toolset_value = session_->robot->toolset(ec);
    if (!ec && session_) {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->toolset_cache = toolset_value;
      session_->model_load_cache = toolset_value.load;
      session_->model_f_t_ee = toolset_value.end.pos;
      if (session_->model_ee_t_k == std::array<double, 16>{}) {
        session_->model_ee_t_k = detail::identity_matrix_16();
      }
    }
    detail::remember_error(session_, ec);
    return toolset_value;
  }

  void setToolset(const Toolset &toolset_value, error_code &ec) noexcept {
    session_->robot->setToolset(toolset_value, ec);
    if (!ec && session_) {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->toolset_cache = toolset_value;
      session_->model_load_cache = toolset_value.load;
      session_->model_f_t_ee = toolset_value.end.pos;
      session_->model_ee_t_k = detail::identity_matrix_16();
    }
    detail::remember_error(session_, ec);
  }

  Toolset setToolset(const std::string &toolName, const std::string &wobjName, error_code &ec) noexcept {
    session_->robot->setToolset(toolName, wobjName, ec);
    if (ec) {
      detail::remember_error(session_, ec);
      return {};
    }
    auto toolset_value = session_->robot->toolset(ec);
    if (!ec && session_) {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->toolset_cache = toolset_value;
      session_->model_load_cache = toolset_value.load;
      session_->model_f_t_ee = toolset_value.end.pos;
      session_->model_ee_t_k = detail::identity_matrix_16();
    }
    detail::remember_error(session_, ec);
    return toolset_value;
  }

protected:
  std::shared_ptr<detail::RobotSession> session_;
};

template <unsigned short DoF>
class XCORE_API Model_T : public BaseModel {
public:
  Model_T() = default;
  explicit Model_T(std::shared_ptr<detail::RobotSession> session) : BaseModel(std::move(session)) {}

  std::array<double, DoF> calcIk(CartesianPosition posture, error_code &ec) noexcept {
    const auto joints = this->session_->robot->calcIk(posture, ec);
    std::array<double, DoF> result{};
    for (size_t i = 0; i < DoF && i < joints.joints.size(); ++i) {
      result[i] = joints.joints[i];
    }
    return result;
  }

  CartesianPosition calcFk(const std::array<double, DoF> &joints, error_code &ec) noexcept {
    JointPosition input(std::vector<double>(joints.begin(), joints.end()));
    return this->session_->robot->calcFk(input, ec);
  }
};

template <unsigned short DoF>
class XCORE_API xMateModel : public Model_T<DoF> {
public:
  xMateModel() = default;
  explicit xMateModel(std::shared_ptr<detail::RobotSession> session)
      : Model_T<DoF>(std::move(session)) {
    syncContextFromSession();
  }

  void setLoad(double mass, const std::array<double, 3> &cog, const std::array<double, 3> &inertia) {
    if (!std::isfinite(mass) || mass < 0.0 ||
        !std::all_of(cog.begin(), cog.end(), [](double value) { return std::isfinite(value); }) ||
        !std::all_of(inertia.begin(), inertia.end(), [](double value) { return std::isfinite(value) && value >= 0.0; })) {
      return;
    }
    model_load_.mass = mass;
    model_load_.cog = cog;
    model_load_.inertia = inertia;
    if (this->session_) {
      std::lock_guard<std::mutex> lock(this->session_->mutex);
      this->session_->model_load_cache = model_load_;
      this->session_->toolset_cache.load = model_load_;
    }
  }

  void setTcpCoor(const std::array<double, 16> &f_t_ee, const std::array<double, 16> &ee_t_k) {
    if (!std::all_of(f_t_ee.begin(), f_t_ee.end(), [](double value) { return std::isfinite(value); }) ||
        !std::all_of(ee_t_k.begin(), ee_t_k.end(), [](double value) { return std::isfinite(value); })) {
      return;
    }
    f_t_ee_ = f_t_ee;
    ee_t_k_ = ee_t_k;
    if (this->session_) {
      std::lock_guard<std::mutex> lock(this->session_->mutex);
      this->session_->model_f_t_ee = f_t_ee_;
      this->session_->model_ee_t_k = ee_t_k_;
      this->session_->toolset_cache.end = Frame(f_t_ee_);
    }
  }

  std::array<double, 16> getCartPose(const std::array<double, DoF> &jntPos, SegmentFrame nr = SegmentFrame::flange) {
    syncContextFromSession();
    std::vector<double> joints(jntPos.begin(), jntPos.end());
    Eigen::Matrix4d matrix = kinematics_.forwardKinematics(joints);
    if (nr == SegmentFrame::endEffector) {
      matrix = matrix * arrayToMatrix(f_t_ee_) * arrayToMatrix(ee_t_k_);
    }
    return matrixToArray(matrix);
  }

  std::array<double, 6> getCartVel(const std::array<double, DoF> &jntPos,
                                   const std::array<double, DoF> &jntVel,
                                   SegmentFrame nr = SegmentFrame::flange) {
    syncContextFromSession();
    (void)nr;
    return modelFacade().cartVelocity(jntPos, jntVel);
  }

  std::array<double, 6> getCartAcc(const std::array<double, DoF> &jntPos,
                                   const std::array<double, DoF> &jntVel,
                                   const std::array<double, DoF> &jntAcc,
                                   SegmentFrame nr = SegmentFrame::flange) {
    syncContextFromSession();
    (void)nr;
    return modelFacade().cartAcceleration(jntPos, jntVel, jntAcc);
  }

  int getJointPos(const std::array<double, 16> &cartPos,
                  double elbow,
                  const std::array<double, DoF> &jntInit,
                  std::array<double, DoF> &jntPos) {
    syncContextFromSession();
    (void)elbow;
    Eigen::Matrix4d target_matrix = arrayToMatrix(cartPos);
    const Eigen::Matrix4d tcp_matrix = arrayToMatrix(f_t_ee_) * arrayToMatrix(ee_t_k_);
    if (!tcp_matrix.isApprox(Eigen::Matrix4d::Identity(), 1e-12)) {
      target_matrix = target_matrix * tcp_matrix.inverse();
    }
    std::vector<double> target = {
      target_matrix(0, 3), target_matrix(1, 3), target_matrix(2, 3),
      std::atan2(target_matrix(2, 1), target_matrix(2, 2)),
      std::atan2(-target_matrix(2, 0),
                 std::sqrt(target_matrix(0, 0) * target_matrix(0, 0) +
                           target_matrix(1, 0) * target_matrix(1, 0))),
      std::atan2(target_matrix(1, 0), target_matrix(0, 0))};
    std::vector<double> init(jntInit.begin(), jntInit.end());
    auto solution = kinematics_.inverseKinematicsSeededFast(target, init);
    if (solution.size() != DoF) {
      solution = kinematics_.inverseKinematics(target, init);
    }
    if (solution.size() != DoF) {
      return -1;
    }
    for (size_t i = 0; i < DoF; ++i) {
      jntPos[i] = solution[i];
    }
    return 0;
  }

  std::array<double, DoF> getJointVel(const std::array<double, 6> &cartVel, const std::array<double, DoF> &jntPos) {
    syncContextFromSession();
    std::vector<double> joints(jntPos.begin(), jntPos.end());
    const auto jacobian = kinematics_.computeJacobian(joints);
    const auto pseudo_inv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::Matrix<double, 6, 1> cart;
    for (size_t i = 0; i < 6; ++i) {
      cart(i) = cartVel[i];
    }
    const auto joint = pseudo_inv * cart;
    std::array<double, DoF> result{};
    for (size_t i = 0; i < DoF; ++i) {
      result[i] = joint(i);
    }
    return result;
  }

  std::array<double, DoF> getJointAcc(const std::array<double, 6> &cartAcc,
                                      const std::array<double, DoF> &jntPos,
                                      const std::array<double, DoF> &jntVel) {
    syncContextFromSession();
    (void)jntVel;
    return modelFacade().jointAcceleration(cartAcc, jntPos);
  }

  std::array<double, DoF * 6> jacobian(const std::array<double, DoF> &jntPos, SegmentFrame nr = SegmentFrame::flange) {
    syncContextFromSession();
    (void)nr;
    const auto jac = modelFacade().jacobian(jntPos);
    std::array<double, DoF * 6> result{};
    for (size_t row = 0; row < 6; ++row) {
      for (size_t col = 0; col < DoF; ++col) {
        result[row * DoF + col] = jac(row, col);
      }
    }
    return result;
  }

  std::array<double, DoF * 6> jacobian(const std::array<double, DoF> &jntPos,
                                       const std::array<double, 16> &f_t_ee,
                                       const std::array<double, 16> &ee_t_k,
                                       SegmentFrame nr = SegmentFrame::flange) {
    f_t_ee_ = f_t_ee;
    ee_t_k_ = ee_t_k;
    return jacobian(jntPos, nr);
  }

  std::array<double, DoF> getTorque(const std::array<double, DoF> &jntPos,
                                    const std::array<double, DoF> &jntVel,
                                    const std::array<double, DoF> &jntAcc,
                                    TorqueType torque_type) {
    syncContextFromSession();
    std::array<double, DoF> full{};
    std::array<double, DoF> inertia{};
    std::array<double, DoF> coriolis{};
    std::array<double, DoF> gravity{};
    getTorqueNoFriction(jntPos, jntVel, jntAcc, full, inertia, coriolis, gravity);
    switch (torque_type) {
      case TorqueType::inertia:
        return inertia;
      case TorqueType::coriolis:
        return coriolis;
      case TorqueType::gravity:
        return gravity;
      case TorqueType::friction:
        return friction_;
      case TorqueType::full:
      default:
        return full;
    }
  }

  void getTorqueWithFriction(const std::array<double, DoF> &jntPos,
                             const std::array<double, DoF> &jntVel,
                             const std::array<double, DoF> &jntAcc,
                             std::array<double, DoF> &trq_full,
                             std::array<double, DoF> &trq_inertia,
                             std::array<double, DoF> &trq_coriolis,
                             std::array<double, DoF> &trq_friction,
                             std::array<double, DoF> &trq_gravity) {
    syncContextFromSession();
    const auto breakdown = approximateDynamics(jntPos, jntVel, jntAcc);
    for (size_t i = 0; i < DoF; ++i) {
      trq_full[i] = breakdown.full[i] + friction_[i];
      trq_inertia[i] = breakdown.inertia[i];
      trq_coriolis[i] = breakdown.coriolis[i];
      trq_friction[i] = friction_[i];
      trq_gravity[i] = breakdown.gravity[i];
    }
  }

  void getTorqueNoFriction(const std::array<double, DoF> &jntPos,
                           const std::array<double, DoF> &jntVel,
                           const std::array<double, DoF> &jntAcc,
                           std::array<double, DoF> &trq_full,
                           std::array<double, DoF> &trq_inertia,
                           std::array<double, DoF> &trq_coriolis,
                           std::array<double, DoF> &trq_gravity) {
    syncContextFromSession();
    const auto breakdown = approximateDynamics(jntPos, jntVel, jntAcc);
    for (size_t i = 0; i < DoF; ++i) {
      trq_full[i] = breakdown.full[i];
      trq_inertia[i] = breakdown.inertia[i];
      trq_coriolis[i] = breakdown.coriolis[i];
      trq_gravity[i] = breakdown.gravity[i];
    }
  }

private:
  void syncContextFromSession() {
    if (!this->session_) {
      return;
    }
    std::lock_guard<std::mutex> lock(this->session_->mutex);
    model_load_ = this->session_->model_load_cache;
    f_t_ee_ = this->session_->model_f_t_ee;
    ee_t_k_ = this->session_->model_ee_t_k;
    if (f_t_ee_ == std::array<double, 16>{}) {
      f_t_ee_ = detail::identity_matrix_16();
    }
    if (ee_t_k_ == std::array<double, 16>{}) {
      ee_t_k_ = detail::identity_matrix_16();
    }
  }

  [[nodiscard]] rokae_xmate3_ros2::gazebo_model::LoadContext loadContext() const {
    return rokae_xmate3_ros2::gazebo_model::LoadContext{model_load_.mass, model_load_.cog};
  }

  [[nodiscard]] rokae_xmate3_ros2::gazebo_model::ModelFacade modelFacade() {
    std::array<double, 6> tool_pose{};
    const Eigen::Matrix4d tool_matrix = arrayToMatrix(f_t_ee_) * arrayToMatrix(ee_t_k_);
    const auto tool_pose_array =
        rokae_xmate3_ros2::gazebo_model::matrixToPose(tool_matrix);
    for (std::size_t i = 0; i < 6; ++i) {
      tool_pose[i] = tool_pose_array[i];
    }
    return rokae_xmate3_ros2::gazebo_model::makeModelFacade(
        kinematics_, tool_pose, loadContext());
  }

  [[nodiscard]] rokae_xmate3_ros2::gazebo_model::DynamicsBreakdown approximateDynamics(
      const std::array<double, DoF> &jntPos,
      const std::array<double, DoF> &jntVel,
      const std::array<double, DoF> &jntAcc) {
    const std::array<double, 6> external_force{};
    return modelFacade().dynamics(jntPos, jntVel, jntAcc, external_force);
  }

  static Eigen::Matrix4d arrayToMatrix(const std::array<double, 16> &values) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        matrix(row, col) = values[row * 4 + col];
      }
    }
    return matrix;
  }

  static std::array<double, 16> matrixToArray(const Eigen::Matrix4d &matrix) {
    std::array<double, 16> result{};
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        result[row * 4 + col] = matrix(row, col);
      }
    }
    return result;
  }

  static std::array<double, 6> to_six(const std::array<double, DoF> &values) {
    std::array<double, 6> out{};
    for (size_t i = 0; i < DoF && i < 6; ++i) {
      out[i] = values[i];
    }
    return out;
  }

  gazebo::xMate3Kinematics kinematics_{};
  Load model_load_{};
  std::array<double, 16> f_t_ee_{};
  std::array<double, 16> ee_t_k_{};
  std::array<double, DoF> friction_{};
};

class XCORE_API BaseRobot : public Base<BaseRobot> {
public:
  virtual ~BaseRobot() = default;

  void disconnectFromRobot(error_code &ec) noexcept {
    session_->robot->disconnectFromRobot(ec);
    detail::remember_error(session_, ec);
  }

  PowerState powerState(error_code &ec) const noexcept {
    const auto state = session_->robot->powerState(ec);
    detail::remember_error(session_, ec);
    return state;
  }

  void setPowerState(bool on, error_code &ec) noexcept {
    session_->robot->setPowerState(on, ec);
    detail::remember_error(session_, ec);
  }

  OperateMode operateMode(error_code &ec) const noexcept {
    const auto mode = session_->robot->operateMode(ec);
    detail::remember_error(session_, ec);
    return mode;
  }

  void setOperateMode(OperateMode mode, error_code &ec) noexcept {
    session_->robot->setOperateMode(mode, ec);
    detail::remember_error(session_, ec);
  }

  Info robotInfo(error_code &ec) const noexcept {
    const auto info = session_->robot->robotInfo(ec);
    detail::remember_error(session_, ec);
    return info;
  }

  OperationState operationState(error_code &ec) const noexcept {
    const auto state = session_->robot->operationState(ec);
    detail::remember_error(session_, ec);
    return state;
  }

  std::array<double, 6> posture(CoordinateType ct, error_code &ec) noexcept {
    const auto pose = session_->robot->posture(ct, ec);
    detail::remember_error(session_, ec);
    return pose;
  }

  CartesianPosition cartPosture(CoordinateType ct, error_code &ec) noexcept {
    const auto pose = session_->robot->cartPosture(ct, ec);
    detail::remember_error(session_, ec);
    return pose;
  }

  std::array<double, 6> flangePos(error_code &ec) noexcept {
    return posture(CoordinateType::flangeInBase, ec);
  }

  std::array<double, 6> baseFrame(error_code &ec) const noexcept {
    const auto frame = session_->robot->baseFrame(ec);
    detail::remember_error(session_, ec);
    return frame;
  }

  Toolset toolset(error_code &ec) const noexcept {
    auto toolset_value = session_->robot->toolset(ec);
    if (!ec) {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->toolset_cache = toolset_value;
      session_->model_load_cache = toolset_value.load;
      session_->model_f_t_ee = toolset_value.end.pos;
      if (session_->model_ee_t_k == std::array<double, 16>{}) {
        session_->model_ee_t_k = detail::identity_matrix_16();
      }
    }
    detail::remember_error(session_, ec);
    return toolset_value;
  }

  void setToolset(const Toolset &toolset_value, error_code &ec) noexcept {
    session_->robot->setToolset(toolset_value, ec);
    if (!ec) {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->toolset_cache = toolset_value;
      session_->model_load_cache = toolset_value.load;
      session_->model_f_t_ee = toolset_value.end.pos;
      session_->model_ee_t_k = detail::identity_matrix_16();
    }
    detail::remember_error(session_, ec);
  }

  Toolset setToolset(const std::string &toolName, const std::string &wobjName, error_code &ec) noexcept {
    session_->robot->setToolset(toolName, wobjName, ec);
    if (ec) {
      detail::remember_error(session_, ec);
      return {};
    }
    auto toolset_value = session_->robot->toolset(ec);
    if (!ec) {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->toolset_cache = toolset_value;
      session_->model_load_cache = toolset_value.load;
      session_->model_f_t_ee = toolset_value.end.pos;
      session_->model_ee_t_k = detail::identity_matrix_16();
    }
    detail::remember_error(session_, ec);
    return toolset_value;
  }

  bool getDI(unsigned int board, unsigned int port, error_code &ec) noexcept {
    const auto state = session_->robot->getDI(board, port, ec);
    detail::remember_error(session_, ec);
    return state;
  }

  void setDI(unsigned board, unsigned port, bool state, error_code &ec) noexcept {
    session_->robot->setDI(board, port, state, ec);
    detail::remember_error(session_, ec);
  }

  bool getDO(unsigned int board, unsigned int port, error_code &ec) noexcept {
    const auto state = session_->robot->getDO(board, port, ec);
    detail::remember_error(session_, ec);
    return state;
  }

  void setDO(unsigned int board, unsigned int port, bool state, error_code &ec) noexcept {
    session_->robot->setDO(board, port, state, ec);
    detail::remember_error(session_, ec);
  }

  double getAI(unsigned board, unsigned port, error_code &ec) noexcept {
    const auto value = session_->robot->getAI(board, port, ec);
    detail::remember_error(session_, ec);
    return value;
  }

  void setAO(unsigned board, unsigned port, double value, error_code &ec) noexcept {
    session_->robot->setAO(board, port, value, ec);
    detail::remember_error(session_, ec);
  }

  void setSimulationMode(bool state, error_code &ec) noexcept {
    session_->robot->setSimulationMode(state, ec);
    detail::remember_error(session_, ec);
  }

  template <typename T>
  void readRegister(const std::string &name, unsigned index, T &value, error_code &ec) noexcept {
    if constexpr (detail::is_vector_v<T>) {
      value.clear();
      const auto size_raw = session_->robot->readRegister(detail::register_size_key(name), ec);
      if (ec) {
        detail::remember_error(session_, ec);
        return;
      }
      size_t size = 0;
      if (!detail::deserialize_scalar(size_raw, size)) {
        ec = std::make_error_code(std::errc::invalid_argument);
        detail::remember_error(session_, ec);
        return;
      }
      value.resize(size);
      for (size_t i = 0; i < size; ++i) {
        if (!detail::read_register_value(*session_->robot, name, static_cast<unsigned>(i), value[i], ec)) {
          detail::remember_error(session_, ec);
          return;
        }
      }
      ec.clear();
    } else {
      detail::read_register_value(*session_->robot, name, index, value, ec);
    }
    detail::remember_error(session_, ec);
  }

  template <typename T>
  void writeRegister(const std::string &name, unsigned index, T value, error_code &ec) noexcept {
    if constexpr (detail::is_vector_v<T>) {
      session_->robot->writeRegister(detail::register_size_key(name), std::to_string(value.size()), ec);
      if (ec) {
        detail::remember_error(session_, ec);
        return;
      }
      for (size_t i = 0; i < value.size(); ++i) {
        detail::write_register_value(*session_->robot, name, static_cast<unsigned>(i), value[i], ec);
        if (ec) {
          detail::remember_error(session_, ec);
          return;
        }
      }
      ec.clear();
    } else {
      detail::write_register_value(*session_->robot, name, index, value, ec);
    }
    detail::remember_error(session_, ec);
  }

  void clearServoAlarm(error_code &ec) noexcept {
    session_->robot->clearServoAlarm(ec);
    detail::remember_error(session_, ec);
  }

  static std::string sdkVersion() noexcept {
    return ::rokae::ros2::xMateRobot::sdkVersion();
  }

  std::vector<LogInfo> queryControllerLog(unsigned count, const std::set<LogInfo::Level> &level, error_code &ec) noexcept {
    if (count == 0 || count > 10) {
      ec = std::make_error_code(std::errc::invalid_argument);
      detail::remember_error(session_, ec);
      return {};
    }
    auto logs = session_->robot->queryControllerLog(count, ec);
    if (!ec && !level.empty()) {
      logs.erase(std::remove_if(logs.begin(), logs.end(), [&](const LogInfo &log) {
        return level.count(static_cast<LogInfo::Level>(log.level)) == 0;
      }), logs.end());
    }
    detail::remember_error(session_, ec);
    return logs;
  }

  void setMotionControlMode(MotionControlMode mode, error_code &ec) noexcept {
    session_->robot->setMotionControlMode(mode, ec);
    detail::remember_error(session_, ec);
  }

  void moveReset(error_code &ec) noexcept {
    session_->robot->moveReset(ec);
    detail::clear_pending_move(session_);
    detail::remember_error(session_, ec);
  }

  void moveStart(error_code &ec) noexcept {
    session_->robot->moveStart(ec);
    if (!ec) {
      error_code state_ec;
      const auto state = session_->robot->operationState(state_ec);
      if (!state_ec && (state == OperationState::idle || state == OperationState::unknown)) {
        const auto [cmd_id, count] = detail::pending_move_info(session_);
        if (!cmd_id.empty() && count > 0) {
          detail::publish_move_event(session_, cmd_id, true, count - 1, {}, "completed");
        }
        detail::clear_pending_move(session_);
      }
    }
    detail::remember_error(session_, ec);
  }

  void stop(error_code &ec) noexcept {
    session_->robot->stop(ec);
    detail::remember_error(session_, ec);
  }

  template <class Command>
  void moveAppend(const std::vector<Command> &cmds, std::string &cmdID, error_code &ec) noexcept {
    if (cmds.empty() || cmds.size() > 100) {
      ec = std::make_error_code(std::errc::invalid_argument);
      detail::remember_error(session_, ec);
      return;
    }
    cmdID = std::to_string(session_->next_cmd_id++);
    {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->pending_move_id = cmdID;
      session_->pending_move_count = static_cast<int>(cmds.size());
    }
    int index = 0;
    for (const auto &cmd : cmds) {
      detail::append_command(*session_->robot, cmd, ec);
      if (ec) {
        detail::publish_move_event(session_, cmdID, false, index, ec, "append failed");
        detail::clear_pending_move(session_);
        detail::remember_error(session_, ec);
        return;
      }
      ++index;
    }
    ec.clear();
    detail::publish_move_event(session_, cmdID, false, 0, ec, "queued");
    detail::remember_error(session_, ec);
  }

  template <class Command>
  void moveAppend(std::initializer_list<Command> cmds, std::string &cmdID, error_code &ec) noexcept {
    moveAppend(std::vector<Command>(cmds), cmdID, ec);
  }

  void setDefaultSpeed(int speed, error_code &ec) noexcept {
    session_->robot->setDefaultSpeed(speed, ec);
    detail::remember_error(session_, ec);
  }

  void setDefaultZone(int zone, error_code &ec) noexcept {
    session_->robot->setDefaultZone(zone, ec);
    detail::remember_error(session_, ec);
  }

  void setDefaultConfOpt(bool forced, error_code &ec) noexcept {
    session_->robot->setDefaultConfOpt(forced, ec);
    detail::remember_error(session_, ec);
  }

  void setMaxCacheSize(int number, error_code &ec) noexcept {
    if (number <= 0) {
      ec = std::make_error_code(std::errc::invalid_argument);
    } else {
      session_->max_cache_size = number;
      ec.clear();
    }
    detail::remember_error(session_, ec);
  }

  void adjustSpeedOnline(double scale, error_code &ec) noexcept {
    session_->robot->adjustSpeedOnline(scale, ec);
    detail::remember_error(session_, ec);
  }

  virtual void startJog(JogOpt::Space space, double rate, double step, unsigned index, bool direction, error_code &ec) noexcept {
    session_->robot->startJog(space, rate, step, index, direction, ec);
    detail::remember_error(session_, ec);
  }

  template <class Command>
  void executeCommand(const std::vector<Command> &cmds, error_code &ec) noexcept {
    if (cmds.empty() || cmds.size() > 1000) {
      ec = std::make_error_code(std::errc::invalid_argument);
      detail::remember_error(session_, ec);
      return;
    }
    std::string cmd_id;
    session_->robot->moveReset(ec);
    if (ec) {
      detail::remember_error(session_, ec);
      return;
    }
    moveAppend(cmds, cmd_id, ec);
    if (ec) {
      detail::remember_error(session_, ec);
      return;
    }
    session_->robot->moveStart(ec);
    if (!ec) {
      detail::wait_until_idle(session_, ec, std::chrono::seconds(60));
    }
    detail::publish_move_event(session_, cmd_id, !static_cast<bool>(ec), static_cast<int>(cmds.size()) - 1, ec,
                               ec ? "execution failed" : "completed");
    detail::remember_error(session_, ec);
  }

  template <class Command>
  void executeCommand(std::initializer_list<Command> cmds, error_code &ec) noexcept {
    executeCommand(std::vector<Command>(cmds), ec);
  }

  std::error_code lastErrorCode() noexcept {
    std::lock_guard<std::mutex> lock(session_->mutex);
    return session_->last_error_code;
  }

  void setEventWatcher(Event eventType, const EventCallback &callback, error_code &ec) noexcept {
    {
      std::lock_guard<std::mutex> lock(session_->mutex);
      if (eventType == Event::moveExecution) {
        session_->move_event_watcher = callback;
      } else {
        session_->safety_event_watcher = callback;
      }
    }
    ec.clear();
    detail::remember_error(session_, ec);
  }

  EventInfo queryEventInfo(Event eventType, error_code &ec) noexcept {
    EventInfo info;
    {
      std::lock_guard<std::mutex> lock(session_->mutex);
      info = eventType == Event::moveExecution ? session_->last_move_event : session_->last_safety_event;
    }
    ec.clear();
    detail::remember_error(session_, ec);
    return info;
  }

  void startReceiveRobotState(std::chrono::steady_clock::duration interval,
                              const std::vector<std::string> &fields) noexcept {
    session_->robot->startReceiveRobotState(interval, fields);
    error_code ec = session_->robot->lastErrorCode();
    {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->state_interval = interval;
      session_->state_cache.clear();
      session_->has_previous_state = false;
      session_->has_previous_pose = false;
      if (ec) {
        session_->state_fields.clear();
        session_->rt_state_plan_rejected = true;
        session_->rt_state_plan_summary = "strict_rejected";
      } else {
        session_->state_fields = fields;
        session_->rt_state_plan_rejected = false;
        session_->rt_state_plan_summary = "strict_ok";
      }
    }
    detail::remember_error(session_, ec);
  }

  void stopReceiveRobotState() noexcept {
    session_->robot->stopReceiveRobotState();
    {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->state_fields.clear();
      session_->state_cache.clear();
      session_->has_previous_state = false;
      session_->has_previous_pose = false;
      session_->rt_state_plan_rejected = false;
      session_->rt_state_plan_summary = "inactive";
    }
    detail::remember_error(session_, {});
  }

  unsigned updateRobotState(std::chrono::steady_clock::duration timeout) {
    const auto count = session_->robot->updateRobotState(timeout);
    error_code ec = session_->robot->lastErrorCode();
    if (ec || count == 0U) {
      detail::remember_error(session_, ec);
      return 0U;
    }

    std::vector<std::string> fields;
    {
      std::lock_guard<std::mutex> lock(session_->mutex);
      fields = session_->state_fields;
      session_->state_cache.clear();
    }

    std::unordered_map<std::string, std::any> new_cache;
    for (const auto &field : fields) {
      std::array<double, 16> matrix16{};
      std::array<double, 6> array6{};
      double scalar{};
      bool boolean{};
      if (field == RtSupportedFields::tcpPose_m || field == RtSupportedFields::tcpPose_c) {
        if (session_->robot->getStateDataMatrix16(field, matrix16) != 0) {
          ec = std::make_error_code(std::errc::invalid_argument);
          detail::remember_error(session_, ec);
          return 0U;
        }
        new_cache[field] = matrix16;
      } else if (field == RtCompatFields::samplePeriod_s) {
        if (session_->robot->getStateDataScalarDouble(field, scalar) != 0) {
          ec = std::make_error_code(std::errc::invalid_argument);
          detail::remember_error(session_, ec);
          return 0U;
        }
        new_cache[field] = scalar;
      } else if (field == RtCompatFields::sampleFresh) {
        if (session_->robot->getStateDataBool(field, boolean) != 0) {
          ec = std::make_error_code(std::errc::invalid_argument);
          detail::remember_error(session_, ec);
          return 0U;
        }
        new_cache[field] = boolean;
      } else {
        if (session_->robot->getStateDataArray6(field, array6) != 0) {
          ec = std::make_error_code(std::errc::invalid_argument);
          detail::remember_error(session_, ec);
          return 0U;
        }
        new_cache[field] = array6;
      }
    }

    {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->state_cache = std::move(new_cache);
      session_->has_previous_state = true;
    }
    ec.clear();
    detail::remember_error(session_, ec);
    return count;
  }

  template <typename R>
  int getStateData(const std::string &fieldName, R &data) {
    std::lock_guard<std::mutex> lock(session_->mutex);
    auto it = session_->state_cache.find(fieldName);
    if (it == session_->state_cache.end()) {
      return -1;
    }
    try {
      data = std::any_cast<R>(it->second);
      return 0;
    } catch (const std::bad_any_cast &) {
      return -1;
    }
  }

  std::vector<RLProjectInfo> projectsInfo(error_code &ec) noexcept {
    const auto projects = session_->robot->projectInfo(ec);
    if (!ec) {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->projects = projects;
    }
    detail::remember_error(session_, ec);
    return projects;
  }

  void loadProject(const std::string &name, const std::vector<std::string> &tasks, error_code &ec) noexcept {
    std::string loaded_name;
    session_->robot->loadRLProject(name, loaded_name, ec);
    if (!ec) {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->current_project.name = loaded_name.empty() ? name : loaded_name;
      session_->current_project.taskList = tasks;
      session_->current_project.is_running = false;
      auto it = std::find_if(session_->projects.begin(), session_->projects.end(), [&](const RLProjectInfo &project) {
        return project.name == session_->current_project.name;
      });
      if (it == session_->projects.end()) {
        session_->projects.push_back(session_->current_project);
      } else {
        *it = session_->current_project;
      }
    }
    detail::remember_error(session_, ec);
  }

  void ppToMain(error_code &ec) noexcept {
    session_->robot->ppToMain(ec);
    detail::remember_error(session_, ec);
  }

  void runProject(error_code &ec) noexcept {
    int current_episode = 0;
    std::string project_name;
    {
      std::lock_guard<std::mutex> lock(session_->mutex);
      project_name = session_->current_project.name;
    }
    session_->robot->startRLProject(project_name, current_episode, ec);
    if (!ec) {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->current_project.is_running = true;
    }
    detail::remember_error(session_, ec);
  }

  void pauseProject(error_code &ec) noexcept {
    int finished_episode = 0;
    std::string project_name;
    {
      std::lock_guard<std::mutex> lock(session_->mutex);
      project_name = session_->current_project.name;
    }
    session_->robot->stopRLProject(project_name, finished_episode, ec);
    if (!ec) {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->current_project.is_running = false;
    }
    detail::remember_error(session_, ec);
  }

  void setProjectRunningOpt(double rate, bool loop, error_code &ec) noexcept {
    session_->robot->setProjectRunningOpt(rate, loop, ec);
    if (!ec) {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->current_project.run_rate = rate;
      session_->current_project.loop_mode = loop;
    }
    detail::remember_error(session_, ec);
  }

  std::vector<WorkToolInfo> toolsInfo(error_code &ec) noexcept {
    const auto tools = session_->robot->toolsInfo(ec);
    if (!ec) {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->tools = tools;
    }
    detail::remember_error(session_, ec);
    return tools;
  }

  std::vector<WorkToolInfo> wobjsInfo(error_code &ec) noexcept {
    const auto wobjs = session_->robot->wobjsInfo(ec);
    if (!ec) {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->wobjs = wobjs;
    }
    detail::remember_error(session_, ec);
    return wobjs;
  }

protected:
  BaseRobot() : session_(detail::make_session()) {}
  explicit BaseRobot(std::shared_ptr<detail::RobotSession> session) : session_(std::move(session)) {}

  std::shared_ptr<detail::RobotSession> session_;
};

template <WorkType Wt, unsigned short DoF>
class XCORE_API Robot_T : virtual public BaseRobot {
public:
  Robot_T() = default;
  explicit Robot_T(const std::string &remoteIP) : BaseRobot(detail::make_session(remoteIP)) {}
  explicit Robot_T(const ::rokae::ros2::RosClientOptions &options) : BaseRobot(detail::make_session(options)) {}

  void connectToRobot(error_code &ec) noexcept {
    this->session_->robot->connectToRobot(ec);
    detail::remember_error(this->session_, ec);
  }

  void connectToRobot(const std::string &remoteIP, const std::string &localIP = "") {
    this->session_ = detail::make_session(remoteIP, localIP);
    error_code ec;
    this->session_->robot->connectToRobot(ec);
    detail::remember_error(this->session_, ec);
  }

  std::array<double, DoF> jointPos(error_code &ec) noexcept {
    const auto value = this->session_->robot->jointPos(ec);
    detail::remember_error(this->session_, ec);
    std::array<double, DoF> out{};
    std::copy_n(value.begin(), DoF, out.begin());
    return out;
  }

  std::array<double, DoF> jointVel(error_code &ec) noexcept {
    const auto value = this->session_->robot->jointVel(ec);
    detail::remember_error(this->session_, ec);
    std::array<double, DoF> out{};
    std::copy_n(value.begin(), DoF, out.begin());
    return out;
  }

  std::array<double, DoF> jointTorques(error_code &ec) noexcept {
    const auto value = this->session_->robot->jointTorques(ec);
    detail::remember_error(this->session_, ec);
    std::array<double, DoF> out{};
    std::copy_n(value.begin(), DoF, out.begin());
    return out;
  }

  [[deprecated("Use jointTorques() instead")]]
  std::array<double, DoF> jointTorque(error_code &ec) noexcept {
    return jointTorques(ec);
  }

  /**
   * @brief Compatibility stub for frame calibration.
   * @param type Requested frame category.
   * @param points Joint-space sample set.
   * @param is_held Reserved compatibility flag.
   * @param ec Receives function_not_supported because frame calibration is intentionally not implemented.
   * @param base_aux Reserved base calibration hint.
   * @return Empty calibration result with success=false.
   * @details The simulation package no longer provides calibrateFrame(). The API surface is retained only to
   *          avoid breaking historical call sites at compile time; every invocation returns
   *          std::errc::function_not_supported.
   */
  FrameCalibrationResult calibrateFrame(FrameType type,
                                        const std::vector<std::array<double, DoF>> &points,
                                        bool is_held,
                                        error_code &ec,
                                        const std::array<double, 3> &base_aux = {}) noexcept {
    (void)type;
    (void)points;
    (void)is_held;
    (void)base_aux;
    FrameCalibrationResult result{};
    ec = std::make_error_code(std::errc::function_not_supported);
    if (this->session_) {
      detail::remember_error(this->session_, ec);
    }
    return result;
  }
  bool getSoftLimit(std::array<double[2], DoF> &limits, error_code &ec) noexcept {
    std::array<std::array<double, 2>, 6> tmp{};
    const auto enabled = this->session_->robot->getSoftLimit(tmp, ec);
    if (!ec) {
      for (size_t i = 0; i < DoF; ++i) {
        limits[i][0] = tmp[i][0];
        limits[i][1] = tmp[i][1];
      }
    }
    detail::remember_error(this->session_, ec);
    return enabled;
  }

  void setSoftLimit(bool enable, error_code &ec, const std::array<double[2], DoF> &limits = {{DBL_MAX, DBL_MAX}}) noexcept {
    std::array<std::array<double, 2>, 6> tmp{};
    for (size_t i = 0; i < DoF; ++i) {
      tmp[i][0] = limits[i][0];
      tmp[i][1] = limits[i][1];
    }
    this->session_->robot->setSoftLimit(enable, ec, tmp);
    detail::remember_error(this->session_, ec);
  }
};

class XCORE_API BaseCobot : virtual public BaseRobot {
public:
  using BaseRobot::BaseRobot;

  void enableDrag(DragParameter::Space space, DragParameter::Type type, error_code &ec) noexcept {
    this->session_->robot->enableDrag(space, type, ec);
    detail::remember_error(this->session_, ec);
  }

  void disableDrag(error_code &ec) noexcept {
    this->session_->robot->disableDrag(ec);
    detail::remember_error(this->session_, ec);
  }

  void startRecordPath(int duration, error_code &ec) noexcept {
    this->session_->robot->startRecordPath(std::chrono::seconds(duration), ec);
    detail::remember_error(this->session_, ec);
  }

  void stopRecordPath(error_code &ec) noexcept {
    this->session_->robot->stopRecordPath(ec);
    detail::remember_error(this->session_, ec);
  }

  void cancelRecordPath(error_code &ec) noexcept {
    this->session_->robot->cancelRecordPath(ec);
    detail::remember_error(this->session_, ec);
  }

  void saveRecordPath(const std::string &name, error_code &ec, const std::string &saveAs = "") noexcept {
    this->session_->robot->saveRecordPath(name, ec, saveAs);
    detail::remember_error(this->session_, ec);
  }

  void replayPath(const std::string &name, double rate, error_code &ec) noexcept {
    this->session_->robot->replayPath(name, rate, ec);
    detail::remember_error(this->session_, ec);
  }

  void removePath(const std::string &name, error_code &ec, bool removeAll = false) noexcept {
    this->session_->robot->removePath(name, ec, removeAll);
    detail::remember_error(this->session_, ec);
  }

  std::vector<std::string> queryPathLists(error_code &ec) noexcept {
    const auto paths = this->session_->robot->queryPathLists(ec);
    detail::remember_error(this->session_, ec);
    return paths;
  }

  void setxPanelVout(xPanelOpt::Vout opt, error_code &ec) noexcept {
    {
      std::lock_guard<std::mutex> lock(this->session_->mutex);
      this->session_->xpanel_vout = opt;
    }
    this->session_->robot->setxPanelVout(opt, ec);
    detail::remember_error(this->session_, ec);
  }

  void setRtNetworkTolerance(unsigned percent, error_code &ec) noexcept {
    if (percent > 100u) {
      ec = std::make_error_code(std::errc::invalid_argument);
      detail::remember_error(this->session_, ec);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(this->session_->mutex);
      this->session_->rt_network_tolerance = percent;
    }
    ec.clear();
    detail::publish_custom_data(this->session_, rokae_xmate3_ros2::runtime::rt_topics::kConfigRtNetworkTolerance,
                                std::to_string(percent), ec);
    detail::remember_error(this->session_, ec);
  }

  void useRciClient(bool use, error_code &ec) noexcept {
    {
      std::lock_guard<std::mutex> lock(this->session_->mutex);
      this->session_->use_rci_client = use;
    }
    ec.clear();
    detail::publish_custom_data(this->session_, rokae_xmate3_ros2::runtime::rt_topics::kConfigUseRciClient,
                                use ? "1" : "0", ec);
    detail::remember_error(this->session_, ec);
  }

  void disableCollisionDetection(error_code &ec) noexcept {
    this->session_->robot->disableCollisionDetection(ec);
    detail::remember_error(this->session_, ec);
  }
};

template <unsigned short DoF>
class XCORE_API Cobot : public Robot_T<WorkType::collaborative, DoF>, public BaseCobot {
public:
  Cobot() = default;
  explicit Cobot(const std::string &remoteIP, const std::string &localIP = "")
      : BaseRobot(detail::make_session(remoteIP, localIP)) {}
  explicit Cobot(const ::rokae::ros2::RosClientOptions &options)
      : BaseRobot(detail::make_session(options)) {}

  virtual ~Cobot() = default;

  std::weak_ptr<RtMotionControlCobot<DoF>> getRtMotionController() {
    static_assert(DoF == 6, "Only xMate3 6DoF collaborative robot is supported by this shim.");
    if (!this->session_->rt_controller) {
      this->session_->rt_controller = std::make_shared<RtMotionControlCobot<DoF>>(this->session_);
    }
    return this->session_->rt_controller;
  }

  void enableCollisionDetection(const std::array<double, DoF> &sensitivity,
                                StopLevel behaviour,
                                double fallback_compliance,
                                error_code &ec) noexcept {
    std::array<double, 6> tmp{};
    std::copy_n(sensitivity.begin(), DoF, tmp.begin());
    this->session_->robot->enableCollisionDetection(tmp, behaviour, fallback_compliance, ec);
    detail::remember_error(this->session_, ec);
    if (!ec) {
      detail::publish_safety_event(this->session_, false);
    }
  }

  std::array<double, 6> getEndEffectorTorque(error_code &ec) noexcept {
    const auto wrench = this->session_->robot->getEndEffectorTorque(ec);
    detail::remember_error(this->session_, ec);
    return wrench;
  }

  [[deprecated("Use getEndEffectorTorque() instead")]]
  std::array<double, 6> getEndTorque(error_code &ec) noexcept {
    return getEndEffectorTorque(ec);
  }

  void getEndTorque(FrameType ref_type,
                    std::array<double, DoF> &joint_torque_measured,
                    std::array<double, DoF> &external_torque_measured,
                    std::array<double, 3> &cart_torque,
                    std::array<double, 3> &cart_force,
                    error_code &ec) noexcept {
    std::array<double, 6> measured{};
    std::array<double, 6> external{};
    this->session_->robot->getEndTorque(ref_type, measured, external, cart_torque, cart_force, ec);
    if (!ec) {
      std::copy_n(measured.begin(), DoF, joint_torque_measured.begin());
      std::copy_n(external.begin(), DoF, external_torque_measured.begin());
    }
    detail::remember_error(this->session_, ec);
  }

  xMateModel<DoF> model() {
    return xMateModel<DoF>(this->session_);
  }
};

class XCORE_API xMateRobot : public Cobot<6> {
public:
  xMateRobot() = default;
  explicit xMateRobot(const std::string &remoteIP, const std::string &localIP = "")
      : BaseRobot(detail::make_session(remoteIP, localIP)) {}
  explicit xMateRobot(const ::rokae::ros2::RosClientOptions &options)
      : BaseRobot(detail::make_session(options)) {}

  void setAvoidSingularity(bool enable, error_code &ec) noexcept {
    this->session_->robot->setAvoidSingularity(enable, ec);
    detail::remember_error(this->session_, ec);
  }

  bool getAvoidSingularity(error_code &ec) noexcept {
    const auto enabled = this->session_->robot->getAvoidSingularity(ec);
    detail::remember_error(this->session_, ec);
    return enabled;
  }

  void startJog(JogOpt::Space space, double rate, double step, unsigned index, bool direction, error_code &ec) noexcept override {
    BaseRobot::startJog(space, rate, step, index, direction, ec);
  }
};


}  // namespace rokae

#endif  // ROKAE_SDK_SHIM_CORE_HPP
