#ifndef ROKAE_SDK_SHIM_HPP
#define ROKAE_SDK_SHIM_HPP

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
#include "rokae/data_types.h"
#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "rokae_xmate3_ros2/gazebo/trajectory_planner.hpp"
#include "rokae_xmate3_ros2/robot.hpp"
#include "rokae_xmate3_ros2/utils.hpp"

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
  int max_cache_size = 10;
  uint64_t next_cmd_id = 1;
  RLProjectInfo current_project;
  std::vector<RLProjectInfo> projects;
  std::vector<WorkToolInfo> tools;
  std::vector<WorkToolInfo> wobjs;
  Toolset toolset_cache{};
  xPanelOpt::Vout xpanel_vout = xPanelOpt::off;
  unsigned rt_network_tolerance = 0;
  bool use_rci_client = false;
  std::shared_ptr<RtMotionControlCobot<6>> rt_controller;
  std::string pending_move_id;
  int pending_move_count = 0;
};

inline std::shared_ptr<RobotSession> make_session() {
  return std::make_shared<RobotSession>(std::make_shared<::rokae::ros2::xMateRobot>());
}

inline std::shared_ptr<RobotSession> make_session(const std::string &remote_ip, const std::string &local_ip = "") {
  return std::make_shared<RobotSession>(std::make_shared<::rokae::ros2::xMateRobot>(remote_ip, local_ip));
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
  {
    std::lock_guard<std::mutex> lock(session->mutex);
    fields = session->state_fields;
    interval = session->state_interval;
    session->state_cache.clear();
  }
  if (fields.empty()) {
    return 0;
  }

  std::array<double, 6> joints{};
  std::array<double, 6> joint_vel{};
  std::array<double, 6> joint_tau{};
  const bool got_rt = session->robot->getRtJointData(joints, joint_vel, joint_tau, ec);
  if (!got_rt) {
    ec.clear();
    joints = session->robot->jointPos(ec);
    if (ec) {
      remember_error(session, ec);
      return 0;
    }
    joint_vel = session->robot->jointVel(ec);
    if (ec) {
      remember_error(session, ec);
      return 0;
    }
    joint_tau = session->robot->jointTorque(ec);
    if (ec) {
      remember_error(session, ec);
      return 0;
    }
  }

  const auto pose = session->robot->cartPosture(CoordinateType::flangeInBase, ec);
  if (ec) {
    remember_error(session, ec);
    return 0;
  }
  const auto pose_matrix = Utils::postureToMatrix(pose);
  const auto end_torque = session->robot->getEndTorque(ec);
  if (ec) {
    remember_error(session, ec);
    return 0;
  }

  const auto now = std::chrono::steady_clock::now();
  const double dt = std::max(1e-6, std::chrono::duration<double>(interval).count());
  std::array<double, 6> joint_acc{};
  std::array<double, 6> tau_vel{};
  {
    std::lock_guard<std::mutex> lock(session->mutex);
    const double derived_dt = session->has_previous_state
                                ? std::max(1e-6, std::chrono::duration<double>(now - session->previous_state_time).count())
                                : dt;
    for (size_t i = 0; i < 6; ++i) {
      if (session->has_previous_state) {
        joint_acc[i] = (joint_vel[i] - session->previous_joint_velocity[i]) / derived_dt;
        tau_vel[i] = (joint_tau[i] - session->previous_joint_torque[i]) / derived_dt;
      }
      session->previous_joint_velocity[i] = joint_vel[i];
      session->previous_joint_torque[i] = joint_tau[i];
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
        session->state_cache[field] = std::array<double, 6>{pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz};
      } else if (field == RtSupportedFields::tcpPose_m || field == RtSupportedFields::tcpPose_c) {
        session->state_cache[field] = pose_matrix;
      } else if (field == RtSupportedFields::tcpVel_c || field == RtSupportedFields::tcpAcc_c) {
        session->state_cache[field] = zero6;
      } else if (field == RtSupportedFields::elbow_m || field == RtSupportedFields::elbow_c ||
                 field == RtSupportedFields::elbowVel_c || field == RtSupportedFields::elbowAcc_c) {
        session->state_cache[field] = 0.0;
      } else if (field == RtSupportedFields::tauVel_c) {
        session->state_cache[field] = tau_vel;
      } else if (field == RtSupportedFields::tauExt_inBase || field == RtSupportedFields::tauExt_inStiff) {
        session->state_cache[field] = end_torque;
      }
    }
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

  // Gazebo shim note: this keeps the SDK RT API shape, but runs as a simulated RT facade.
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
      const auto state = session_->robot->operationState(ec);
      if (ec) {
        detail::remember_error(session_, ec);
        return;
      }
      if (state != OperationState::idle && state != OperationState::unknown) {
        return;
      }
      if (value.type() == typeid(JointPosition)) {
        const auto &command = std::any_cast<const JointPosition &>(value);
        if (!command.isFinished()) {
          if (has_last_joint_command_ && detail::approx_equal_joint_command(command, last_joint_command_)) {
            return;
          }
          MoveAbsJCommand move(command);
          session_->robot->moveReset(ec);
          if (!ec) {
            session_->robot->moveAbsJ(move, ec);
          }
          if (!ec) {
            session_->robot->moveStart(ec);
          }
          if (!ec) {
            last_dispatch_time_ = now;
            last_joint_command_ = command;
            has_last_joint_command_ = true;
          }
        }
      } else if (value.type() == typeid(CartesianPosition)) {
        const auto &command = std::any_cast<const CartesianPosition &>(value);
        if (!command.isFinished()) {
          if (has_last_cartesian_command_ &&
              detail::approx_equal_cartesian_command(command, last_cartesian_command_)) {
            return;
          }
          auto move = MoveLCommand(command);
          session_->robot->moveReset(ec);
          if (!ec) {
            session_->robot->moveL(move, ec);
          }
          if (!ec) {
            session_->robot->moveStart(ec);
          }
          if (!ec) {
            last_dispatch_time_ = now;
            last_cartesian_command_ = command;
            has_last_cartesian_command_ = true;
          }
        }
      } else if (value.type() == typeid(Torque)) {
        // Gazebo shim: torque loop is simulation-only and not a true hardware torque servo.
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
    session_->robot->stop(ec);
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

  void startMove(RtControllerMode rtMode) {
    if (!this->session_) {
      return;
    }
    error_code ec;
    this->session_->robot->setRtControlMode(rtMode, ec);
    detail::remember_error(this->session_, ec);
  }

  bool setFilterLimit(bool limit_rate, double cutoff_frequency) noexcept {
    filter_limit_enabled_ = limit_rate;
    filter_cutoff_frequency_ = cutoff_frequency;
    return true;
  }

  void setCartesianLimit(const std::array<double, 3> &lengths, const std::array<double, 16> &frame, error_code &ec) noexcept {
    cartesian_limit_lengths_ = lengths;
    cartesian_limit_frame_ = frame;
    ec.clear();
  }

  void setEndEffectorFrame(const std::array<double, 16> &frame, error_code &ec) noexcept {
    end_effector_frame_ = frame;
    ec.clear();
  }

  void setLoad(const Load &load, error_code &ec) noexcept {
    load_ = load;
    ec.clear();
  }

  void MoveJ(double speed, const std::array<double, DoF> &start, const std::array<double, DoF> &target) {
    if (!this->session_) {
      return;
    }
    error_code ec;
    (void)start;
    MoveAbsJCommand cmd;
    cmd.target = JointPosition(std::vector<double>(target.begin(), target.end()));
    cmd.speed = detail::rt_speed_ratio_to_nrt_speed_mm_per_s(speed);
    cmd.zone = 0;
    this->session_->robot->moveReset(ec);
    if (!ec) {
      this->session_->robot->moveAbsJ(cmd, ec);
    }
    if (!ec) {
      this->session_->robot->moveStart(ec);
    }
    if (!ec) {
      detail::wait_until_idle(this->session_, ec, std::chrono::seconds(30));
    }
    detail::remember_error(this->session_, ec);
  }

  void MoveL(double speed, CartesianPosition &start, CartesianPosition &target) {
    if (!this->session_) {
      return;
    }
    error_code ec;
    (void)start;
    MoveLCommand cmd(target);
    cmd.speed = detail::rt_speed_ratio_to_nrt_speed_mm_per_s(speed);
    this->session_->robot->moveReset(ec);
    if (!ec) {
      this->session_->robot->moveL(cmd, ec);
    }
    if (!ec) {
      this->session_->robot->moveStart(ec);
    }
    if (!ec) {
      detail::wait_until_idle(this->session_, ec, std::chrono::seconds(30));
    }
    detail::remember_error(this->session_, ec);
  }

  void MoveC(double speed, CartesianPosition &start, CartesianPosition &aux, CartesianPosition &target) {
    if (!this->session_) {
      return;
    }
    error_code ec;
    (void)start;
    MoveCCommand cmd(target, aux);
    cmd.speed = detail::rt_speed_ratio_to_nrt_speed_mm_per_s(speed);
    this->session_->robot->moveReset(ec);
    if (!ec) {
      this->session_->robot->moveC(cmd, ec);
    }
    if (!ec) {
      this->session_->robot->moveStart(ec);
    }
    if (!ec) {
      detail::wait_until_idle(this->session_, ec, std::chrono::seconds(30));
    }
    detail::remember_error(this->session_, ec);
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
  }

  void setCartesianImpedance(const std::array<double, 6> &factor, error_code &ec) noexcept {
    cartesian_impedance_ = factor;
    ec.clear();
  }

  void setFilterFrequency(double jointFrequency, double cartesianFrequency, double torqueFrequency, error_code &ec) noexcept {
    filter_frequencies_ = {jointFrequency, cartesianFrequency, torqueFrequency};
    ec.clear();
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
  }

  void setTorqueFilterCutOffFrequency(double frequency, error_code &ec) noexcept {
    torque_cutoff_frequency_ = frequency;
    ec.clear();
  }

  void setFcCoor(const std::array<double, 16> &frame, FrameType type, error_code &ec) noexcept {
    fc_frame_ = frame;
    fc_type_ = type;
    ec.clear();
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
    return session_->robot->toolset(ec);
  }

  void setToolset(const Toolset &toolset_value, error_code &ec) noexcept {
    session_->robot->setToolset(toolset_value, ec);
  }

  Toolset setToolset(const std::string &toolName, const std::string &wobjName, error_code &ec) noexcept {
    session_->robot->setToolset(toolName, wobjName, ec);
    if (ec) {
      return {};
    }
    return session_->robot->toolset(ec);
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
      : Model_T<DoF>(std::move(session)) {}

  void setLoad(double mass, const std::array<double, 3> &cog, const std::array<double, 3> &inertia) {
    model_load_.mass = mass;
    model_load_.cog = cog;
    model_load_.inertia = inertia;
  }

  void setTcpCoor(const std::array<double, 16> &f_t_ee, const std::array<double, 16> &ee_t_k) {
    f_t_ee_ = f_t_ee;
    ee_t_k_ = ee_t_k;
  }

  std::array<double, 16> getCartPose(const std::array<double, DoF> &jntPos, SegmentFrame nr = SegmentFrame::flange) {
    (void)nr;
    std::vector<double> joints(jntPos.begin(), jntPos.end());
    const auto matrix = kinematics_.forwardKinematics(joints);
    std::array<double, 16> result{};
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        result[row * 4 + col] = matrix(row, col);
      }
    }
    return result;
  }

  std::array<double, 6> getCartVel(const std::array<double, DoF> &jntPos,
                                   const std::array<double, DoF> &jntVel,
                                   SegmentFrame nr = SegmentFrame::flange) {
    (void)nr;
    std::vector<double> joints(jntPos.begin(), jntPos.end());
    const auto jacobian = kinematics_.computeJacobian(joints);
    Eigen::Matrix<double, 6, 1> velocity = Eigen::Matrix<double, 6, 1>::Zero();
    for (size_t i = 0; i < DoF; ++i) {
      velocity(i) = jntVel[i];
    }
    const auto cart = jacobian * velocity;
    std::array<double, 6> result{};
    for (size_t i = 0; i < 6; ++i) {
      result[i] = cart(i);
    }
    return result;
  }

  std::array<double, 6> getCartAcc(const std::array<double, DoF> &jntPos,
                                   const std::array<double, DoF> &jntVel,
                                   const std::array<double, DoF> &jntAcc,
                                   SegmentFrame nr = SegmentFrame::flange) {
    (void)nr;
    const auto vel = getCartVel(jntPos, jntVel);
    std::array<double, 6> result{};
    for (size_t i = 0; i < 6; ++i) {
      result[i] = vel[i] + (i < DoF ? jntAcc[i] * 0.001 : 0.0);
    }
    return result;
  }

  int getJointPos(const std::array<double, 16> &cartPos,
                  double elbow,
                  const std::array<double, DoF> &jntInit,
                  std::array<double, DoF> &jntPos) {
    (void)elbow;
    std::vector<double> target = {
      cartPos[3], cartPos[7], cartPos[11],
      std::atan2(cartPos[9], cartPos[10]),
      std::atan2(-cartPos[8], std::sqrt(cartPos[0] * cartPos[0] + cartPos[4] * cartPos[4])),
      std::atan2(cartPos[4], cartPos[0])};
    std::vector<double> init(jntInit.begin(), jntInit.end());
    const auto solution = kinematics_.inverseKinematics(target, init);
    if (solution.size() != DoF) {
      return -1;
    }
    for (size_t i = 0; i < DoF; ++i) {
      jntPos[i] = solution[i];
    }
    return 0;
  }

  std::array<double, DoF> getJointVel(const std::array<double, 6> &cartVel, const std::array<double, DoF> &jntPos) {
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
    (void)jntVel;
    return getJointVel(cartAcc, jntPos);
  }

  std::array<double, DoF * 6> jacobian(const std::array<double, DoF> &jntPos, SegmentFrame nr = SegmentFrame::flange) {
    (void)nr;
    std::vector<double> joints(jntPos.begin(), jntPos.end());
    const auto jac = kinematics_.computeJacobian(joints);
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
    (void)f_t_ee;
    (void)ee_t_k;
    return jacobian(jntPos, nr);
  }

  std::array<double, DoF> getTorque(const std::array<double, DoF> &jntPos,
                                    const std::array<double, DoF> &jntVel,
                                    const std::array<double, DoF> &jntAcc,
                                    TorqueType torque_type) {
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
    std::array<double, 6> external_force{};
    std::array<double, 6> full6{};
    std::array<double, 6> gravity6{};
    std::array<double, 6> coriolis6{};
    error_code ec;
    if (this->session_) {
      this->session_->robot->calcJointTorque(to_six(jntPos), to_six(jntVel), to_six(jntAcc), external_force, full6, gravity6, coriolis6, ec);
    }
    for (size_t i = 0; i < DoF; ++i) {
      trq_full[i] = full6[i] + friction_[i];
      trq_inertia[i] = jntAcc[i] * 0.02;
      trq_coriolis[i] = coriolis6[i];
      trq_friction[i] = friction_[i];
      trq_gravity[i] = gravity6[i];
    }
  }

  void getTorqueNoFriction(const std::array<double, DoF> &jntPos,
                           const std::array<double, DoF> &jntVel,
                           const std::array<double, DoF> &jntAcc,
                           std::array<double, DoF> &trq_full,
                           std::array<double, DoF> &trq_inertia,
                           std::array<double, DoF> &trq_coriolis,
                           std::array<double, DoF> &trq_gravity) {
    std::array<double, 6> external_force{};
    std::array<double, 6> full6{};
    std::array<double, 6> gravity6{};
    std::array<double, 6> coriolis6{};
    error_code ec;
    if (this->session_) {
      this->session_->robot->calcJointTorque(to_six(jntPos), to_six(jntVel), to_six(jntAcc), external_force, full6, gravity6, coriolis6, ec);
    }
    for (size_t i = 0; i < DoF; ++i) {
      trq_full[i] = full6[i];
      trq_inertia[i] = jntAcc[i] * 0.02;
      trq_coriolis[i] = coriolis6[i];
      trq_gravity[i] = gravity6[i];
    }
  }

private:
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
    }
    detail::remember_error(session_, ec);
    return toolset_value;
  }

  void setToolset(const Toolset &toolset_value, error_code &ec) noexcept {
    session_->robot->setToolset(toolset_value, ec);
    if (!ec) {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->toolset_cache = toolset_value;
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

  void stopReceiveRobotState() noexcept {
    std::lock_guard<std::mutex> lock(session_->mutex);
    session_->state_fields.clear();
    session_->state_cache.clear();
    session_->has_previous_state = false;
  }

  unsigned updateRobotState(std::chrono::steady_clock::duration timeout) {
    (void)timeout;
    error_code ec;
    const auto count = detail::refresh_state_cache(session_, ec);
    detail::remember_error(session_, ec);
    return ec ? 0U : count;
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
    std::vector<RLProjectInfo> projects;
    {
      std::lock_guard<std::mutex> lock(session_->mutex);
      projects = session_->projects;
    }
    ec.clear();
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
    ec.clear();
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
    {
      std::lock_guard<std::mutex> lock(session_->mutex);
      session_->current_project.run_rate = rate;
      session_->current_project.loop_mode = loop;
    }
    ec.clear();
    detail::remember_error(session_, ec);
  }

  std::vector<WorkToolInfo> toolsInfo(error_code &ec) noexcept {
    std::vector<WorkToolInfo> tools;
    {
      std::lock_guard<std::mutex> lock(session_->mutex);
      if (session_->tools.empty()) {
        session_->tools.push_back(detail::make_default_tool_info());
      }
      tools = session_->tools;
    }
    ec.clear();
    detail::remember_error(session_, ec);
    return tools;
  }

  std::vector<WorkToolInfo> wobjsInfo(error_code &ec) noexcept {
    std::vector<WorkToolInfo> wobjs;
    {
      std::lock_guard<std::mutex> lock(session_->mutex);
      wobjs = session_->wobjs;
    }
    ec.clear();
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

  std::array<double, DoF> jointTorque(error_code &ec) noexcept {
    const auto value = this->session_->robot->jointTorque(ec);
    detail::remember_error(this->session_, ec);
    std::array<double, DoF> out{};
    std::copy_n(value.begin(), DoF, out.begin());
    return out;
  }

  FrameCalibrationResult calibrateFrame(FrameType type,
                                        const std::vector<std::array<double, DoF>> &points,
                                        bool is_held,
                                        error_code &ec,
                                        const std::array<double, 3> &base_aux = {}) noexcept {
    (void)is_held;
    FrameCalibrationResult result{};
    if (!this->session_) {
      ec = std::make_error_code(std::errc::not_connected);
      return result;
    }
    if constexpr (DoF != 6) {
      ec = std::make_error_code(std::errc::not_supported);
      detail::remember_error(this->session_, ec);
      return result;
    }
    if (points.size() < 3) {
      ec = std::make_error_code(std::errc::invalid_argument);
      detail::remember_error(this->session_, ec);
      return result;
    }

    gazebo::xMate3Kinematics kinematics;
    std::vector<Eigen::Matrix3d> rotations;
    std::vector<Eigen::Vector3d> flange_positions;
    rotations.reserve(points.size());
    flange_positions.reserve(points.size());
    for (const auto &sample : points) {
      std::vector<double> joints(sample.begin(), sample.end());
      const Eigen::Matrix4d tf = kinematics.forwardKinematics(joints);
      rotations.push_back(tf.block<3, 3>(0, 0));
      flange_positions.push_back(tf.block<3, 1>(0, 3));
    }

    auto stats_from_errors = [](const std::vector<double> &errors) {
      std::array<double, 3> stats{};
      if (errors.empty()) {
        return stats;
      }
      const auto [min_it, max_it] = std::minmax_element(errors.begin(), errors.end());
      const double sum = std::accumulate(errors.begin(), errors.end(), 0.0);
      stats[0] = *min_it;
      stats[1] = sum / static_cast<double>(errors.size());
      stats[2] = *max_it;
      return stats;
    };

    auto frame_from_origin_axes = [](const Eigen::Vector3d &origin,
                                     Eigen::Vector3d x_axis,
                                     Eigen::Vector3d y_hint) {
      if (x_axis.norm() < 1e-9) {
        x_axis = Eigen::Vector3d::UnitX();
      }
      x_axis.normalize();
      y_hint -= y_hint.dot(x_axis) * x_axis;
      if (y_hint.norm() < 1e-9) {
        y_hint = Eigen::Vector3d::UnitY() - Eigen::Vector3d::UnitY().dot(x_axis) * x_axis;
      }
      if (y_hint.norm() < 1e-9) {
        y_hint = Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitZ().dot(x_axis) * x_axis;
      }
      y_hint.normalize();
      Eigen::Vector3d z_axis = x_axis.cross(y_hint);
      if (z_axis.norm() < 1e-9) {
        z_axis = Eigen::Vector3d::UnitZ();
      }
      z_axis.normalize();
      Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
      Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
      tf.block<3, 1>(0, 0) = x_axis;
      tf.block<3, 1>(0, 1) = y_axis;
      tf.block<3, 1>(0, 2) = z_axis;
      tf.block<3, 1>(0, 3) = origin;
      std::array<double, 16> pose{};
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          pose[r * 4 + c] = tf(r, c);
        }
      }
      return Frame(pose);
    };

    if (type == FrameType::tool) {
      Eigen::MatrixXd A(static_cast<int>((points.size() * (points.size() - 1)) / 2) * 3, 3);
      Eigen::VectorXd b(A.rows());
      int row = 0;
      for (size_t i = 0; i < rotations.size(); ++i) {
        for (size_t j = i + 1; j < rotations.size(); ++j) {
          A.block<3, 3>(row, 0) = rotations[i] - rotations[j];
          b.segment<3>(row) = flange_positions[j] - flange_positions[i];
          row += 3;
        }
      }
      const Eigen::Vector3d tcp_in_flange = A.completeOrthogonalDecomposition().solve(b);
      std::vector<Eigen::Vector3d> tcp_positions;
      std::vector<double> errors;
      tcp_positions.reserve(points.size());
      errors.reserve(points.size());
      for (size_t i = 0; i < rotations.size(); ++i) {
        tcp_positions.push_back(flange_positions[i] + rotations[i] * tcp_in_flange);
      }
      Eigen::Vector3d tcp_mean = Eigen::Vector3d::Zero();
      for (const auto &p : tcp_positions) {
        tcp_mean += p;
      }
      tcp_mean /= static_cast<double>(tcp_positions.size());
      for (const auto &p : tcp_positions) {
        errors.push_back((p - tcp_mean).norm());
      }

      Frame frame;
      frame.trans = {tcp_in_flange.x(), tcp_in_flange.y(), tcp_in_flange.z()};
      frame.rpy = {0.0, 0.0, 0.0};
      frame.syncPoseMatrix();
      result.frame = frame;
      result.errors = stats_from_errors(errors);
      result.success = true;

      {
        std::lock_guard<std::mutex> lock(this->session_->mutex);
        this->session_->toolset_cache.end = result.frame;
      }
      ec.clear();
      detail::remember_error(this->session_, ec);
      return result;
    }

    std::vector<Eigen::Vector3d> tcp_points;
    tcp_points.reserve(points.size());
    Eigen::Vector3d tcp_offset = Eigen::Vector3d::Zero();
    {
      std::lock_guard<std::mutex> lock(this->session_->mutex);
      tcp_offset = Eigen::Vector3d(this->session_->toolset_cache.end.trans[0],
                                   this->session_->toolset_cache.end.trans[1],
                                   this->session_->toolset_cache.end.trans[2]);
    }
    for (size_t i = 0; i < flange_positions.size(); ++i) {
      tcp_points.push_back(flange_positions[i] + rotations[i] * tcp_offset);
    }

    const Eigen::Vector3d origin = tcp_points[0];
    const Eigen::Vector3d x_axis = tcp_points[1] - origin;
    Eigen::Vector3d y_hint = (type == FrameType::base && Eigen::Vector3d(base_aux[0], base_aux[1], base_aux[2]).norm() > 1e-9)
                           ? Eigen::Vector3d(base_aux[0], base_aux[1], base_aux[2]) - origin
                           : (tcp_points[2] - origin);
    result.frame = frame_from_origin_axes(origin, x_axis, y_hint);

    std::vector<double> errors;
    errors.reserve(tcp_points.size());
    Eigen::Vector3d x_dir(result.frame.pos[0], result.frame.pos[4], result.frame.pos[8]);
    Eigen::Vector3d y_dir(result.frame.pos[1], result.frame.pos[5], result.frame.pos[9]);
    Eigen::Vector3d z_dir(result.frame.pos[2], result.frame.pos[6], result.frame.pos[10]);
    const Eigen::Matrix3d rot = (Eigen::Matrix3d() << x_dir.x(), y_dir.x(), z_dir.x(),
                                                      x_dir.y(), y_dir.y(), z_dir.y(),
                                                      x_dir.z(), y_dir.z(), z_dir.z()).finished();
    for (const auto &p : tcp_points) {
      const Eigen::Vector3d local = rot.transpose() * (p - origin);
      errors.push_back(std::abs(local.z()));
    }
    result.errors = stats_from_errors(errors);
    result.success = true;
    ec.clear();
    {
      std::lock_guard<std::mutex> lock(this->session_->mutex);
      if (type == FrameType::wobj || type == FrameType::base || type == FrameType::world) {
        this->session_->toolset_cache.ref = result.frame;
      }
    }
    detail::remember_error(this->session_, ec);
    return result;
  }

  Model_T<DoF> model() noexcept {
    return Model_T<DoF>(this->session_);
  }

  void startReceiveRobotState(std::chrono::steady_clock::duration interval, const std::vector<std::string> &fields) {
    {
      std::lock_guard<std::mutex> lock(this->session_->mutex);
      this->session_->state_interval = interval;
      this->session_->state_fields = fields;
      this->session_->state_cache.clear();
      this->session_->has_previous_state = false;
    }
    error_code refresh_ec;
    detail::refresh_state_cache(this->session_, refresh_ec);
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
    this->session_->robot->writeRegister("xpanel_vout", std::to_string(static_cast<int>(opt)), ec);
    detail::remember_error(this->session_, ec);
  }

  void setRtNetworkTolerance(unsigned percent, error_code &ec) noexcept {
    {
      std::lock_guard<std::mutex> lock(this->session_->mutex);
      this->session_->rt_network_tolerance = percent;
    }
    ec.clear();
    detail::remember_error(this->session_, ec);
  }

  void useRciClient(bool use, error_code &ec) noexcept {
    {
      std::lock_guard<std::mutex> lock(this->session_->mutex);
      this->session_->use_rci_client = use;
    }
    ec.clear();
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

  void getEndTorque(FrameType ref_type,
                    std::array<double, DoF> &joint_torque_measured,
                    std::array<double, DoF> &external_torque_measured,
                    std::array<double, 3> &cart_torque,
                    std::array<double, 3> &cart_force,
                    error_code &ec) noexcept {
    (void)ref_type;
    const auto measured = this->session_->robot->jointTorque(ec);
    const auto wrench = this->session_->robot->getEndTorque(ec);
    if (!ec) {
      std::copy_n(measured.begin(), DoF, joint_torque_measured.begin());
      std::fill(external_torque_measured.begin(), external_torque_measured.end(), 0.0);
      cart_force = {wrench[0], wrench[1], wrench[2]};
      cart_torque = {wrench[3], wrench[4], wrench[5]};
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

class XCORE_API CartMotionGenerator {
public:
  CartMotionGenerator(double speed_factor, double s_goal)
      : speed_factor_(speed_factor), s_goal_(s_goal) {}

  void setMax(double ds_max, double dds_max_start, double dds_max_end) {
    ds_max_ = ds_max;
    dds_max_start_ = dds_max_start;
    dds_max_end_ = dds_max_end;
  }

  double getTime() {
    return std::abs(s_goal_) / std::max(ds_max_ * std::max(speed_factor_, 1e-6), 1e-6);
  }

  bool calculateDesiredValues(double t, double *delta_s_d) const {
    const double total = std::abs(s_goal_) / std::max(ds_max_ * std::max(speed_factor_, 1e-6), 1e-6);
    if (delta_s_d) {
      *delta_s_d = std::clamp(t / std::max(total, 1e-6), 0.0, 1.0) * s_goal_;
    }
    return t >= total;
  }

  void calculateSynchronizedValues(double s_init) {
    s_init_ = s_init;
  }

private:
  double speed_factor_ = 1.0;
  double s_goal_ = 0.0;
  double s_init_ = 0.0;
  double ds_max_ = 0.5;
  double dds_max_start_ = 0.5;
  double dds_max_end_ = 0.5;
};

class XCORE_API JointMotionGenerator {
public:
  JointMotionGenerator(double speed_factor, std::array<double, 6> q_goal)
      : speed_factor_(speed_factor), q_goal_(q_goal) {}

  void setMax(const std::array<double, 6> &dq_max,
              const std::array<double, 6> &ddq_max_start,
              const std::array<double, 6> &ddq_max_end) {
    dq_max_ = dq_max;
    ddq_max_start_ = ddq_max_start;
    ddq_max_end_ = ddq_max_end;
  }

  double getTime() {
    double max_delta = 0.0;
    for (size_t i = 0; i < q_goal_.size(); ++i) {
      max_delta = std::max(max_delta, std::abs(q_goal_[i] - q_init_[i]));
    }
    const double max_speed = std::max(1e-6, *std::max_element(dq_max_.begin(), dq_max_.end()) * std::max(speed_factor_, 1e-6));
    return max_delta / max_speed;
  }

  bool calculateDesiredValues(double t, std::array<double, 6> &delta_q_d) const {
    const double total = std::max(getTimeCached(), 1e-6);
    const double alpha = std::clamp(t / total, 0.0, 1.0);
    for (size_t i = 0; i < delta_q_d.size(); ++i) {
      delta_q_d[i] = (q_goal_[i] - q_init_[i]) * alpha;
    }
    return t >= total;
  }

  void calculateSynchronizedValues(const std::array<double, 6> &q_init) {
    q_init_ = q_init;
  }

private:
  double getTimeCached() const {
    double max_delta = 0.0;
    for (size_t i = 0; i < q_goal_.size(); ++i) {
      max_delta = std::max(max_delta, std::abs(q_goal_[i] - q_init_[i]));
    }
    const double max_speed = std::max(1e-6, *std::max_element(dq_max_.begin(), dq_max_.end()) * std::max(speed_factor_, 1e-6));
    return max_delta / max_speed;
  }

  double speed_factor_ = 1.0;
  std::array<double, 6> q_goal_{};
  std::array<double, 6> q_init_{};
  std::array<double, 6> dq_max_{{1, 1, 1, 1, 1, 1}};
  std::array<double, 6> ddq_max_start_{{1, 1, 1, 1, 1, 1}};
  std::array<double, 6> ddq_max_end_{{1, 1, 1, 1, 1, 1}};
};

template <unsigned short DoF>
class XCORE_API FollowPosition {
public:
  FollowPosition() = default;

  FollowPosition(Cobot<DoF> &robot,
                 xMateModel<DoF> &model,
                 const Eigen::Transform<double, 3, Eigen::Isometry> &endInFlange = Eigen::Transform<double, 3, Eigen::Isometry>::Identity())
      : robot_(&robot), model_(&model), end_in_flange_(endInFlange) {}

  void init(Cobot<DoF> &robot, XMateModel<DoF> &model) {
    robot_ = &robot;
    model_ = &model;
  }

  void start(const Eigen::Transform<double, 3, Eigen::Isometry> &bMe_desire) {
    active_ = true;
    update(bMe_desire);
  }

  void start(const std::array<double, DoF> &jnt_desire) {
    active_ = true;
    update(jnt_desire);
  }

  void stop() {
    active_ = false;
    if (robot_) {
      error_code ec;
      robot_->stop(ec);
    }
  }

  void update(const Eigen::Transform<double, 3, Eigen::Isometry> &bMe_desire) {
    if (!active_ || !robot_) {
      return;
    }
    const Eigen::Vector3d trans = bMe_desire.translation();
    const Eigen::Vector3d rpy = bMe_desire.rotation().eulerAngles(0, 1, 2);
    CartesianPosition pose({trans.x(), trans.y(), trans.z(), rpy.x(), rpy.y(), rpy.z()});
    MoveLCommand cmd(pose);
    error_code ec;
    robot_->executeCommand({cmd}, ec);
  }

  void update(const std::array<double, DoF> &jnt_desired) {
    if (!active_ || !robot_) {
      return;
    }
    MoveAbsJCommand cmd;
    cmd.target = JointPosition(std::vector<double>(jnt_desired.begin(), jnt_desired.end()));
    error_code ec;
    robot_->executeCommand({cmd}, ec);
  }

  void setScale(double scale) {
    scale_ = scale;
  }

private:
  Cobot<DoF> *robot_ = nullptr;
  xMateModel<DoF> *model_ = nullptr;
  Eigen::Transform<double, 3, Eigen::Isometry> end_in_flange_ = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
  bool active_ = false;
  double scale_ = 0.5;
};

}  // namespace rokae

#endif  // ROKAE_SDK_SHIM_HPP
