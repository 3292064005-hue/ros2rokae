#include "compat/internal/compat_shared.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <thread>

#include <Eigen/Geometry>

#include "compat/rt_motion_primitives.hpp"
#include "rokae/motion_control_rt.h"
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "runtime/rt_command_bridge.hpp"
#include "rokae_xmate3_ros2/runtime/rt_semantic_topics.hpp"

namespace rokae {
namespace {

constexpr auto kStrictLoopPeriod = std::chrono::milliseconds(1);
constexpr auto kStrictLoopLatenessTolerance = std::chrono::microseconds(600);

std::string bool_text(bool value) { return value ? "1" : "0"; }

std::string serialize_values(const std::array<double, 6> &values) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) oss << ',';
    oss << values[i];
  }
  return oss.str();
}

std::string serialize_matrix16(const std::array<double, 16> &values) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) oss << ',';
    oss << values[i];
  }
  return oss.str();
}

std::string serialize_load(const Load &load) {
  std::ostringstream oss;
  oss << "mass=" << load.mass << ";cog=" << load.cog[0] << ',' << load.cog[1] << ',' << load.cog[2]
      << ";inertia=" << load.inertia[0] << ',' << load.inertia[1] << ',' << load.inertia[2];
  return oss.str();
}

std::string build_filter_limit_payload(bool limit_rate, double cutoff_frequency) {
  std::ostringstream oss;
  oss << "enabled=" << bool_text(limit_rate) << ";cutoff_frequency=" << cutoff_frequency;
  return oss.str();
}

std::string build_cartesian_limit_payload(const std::array<double, 3> &lengths,
                                          const std::array<double, 16> &frame) {
  std::ostringstream oss;
  oss << "lengths=" << lengths[0] << ',' << lengths[1] << ',' << lengths[2]
      << ";frame=" << serialize_matrix16(frame);
  return oss.str();
}

std::array<double, 6> posture_to_array(const CartesianPosition &pose) {
  return {pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz};
}

bool finite_matrix16(const std::array<double, 16> &values) {
  for (double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool finite_array6(const std::array<double, 6> &values) {
  for (double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool finite_non_negative_array6(const std::array<double, 6> &values) {
  for (double value : values) {
    if (!std::isfinite(value) || value < 0.0) {
      return false;
    }
  }
  return true;
}

bool finite_array3_non_negative(const std::array<double, 3> &values) {
  for (double value : values) {
    if (!std::isfinite(value) || value < 0.0) {
      return false;
    }
  }
  return true;
}

bool valid_force_control_type(FrameType type) {
  return type == FrameType::world || type == FrameType::tool || type == FrameType::path;
}

std::array<double, 6> read_joint_state(detail::CompatRobotHandle &handle, error_code &ec) {
  (void)handle.backend->updateRobotState(std::chrono::milliseconds(1));
  std::array<double, 6> joints{};
  if (handle.backend->getStateDataArray6(RtSupportedFields::jointPos_m, joints) == 0) {
    ec.clear();
    return joints;
  }
  joints = handle.backend->jointPos(ec);
  if (!ec) {
    return joints;
  }
  ec = std::make_error_code(std::errc::resource_unavailable_try_again);
  return {};
}

std::array<double, 16> read_pose_state(detail::CompatRobotHandle &handle, error_code &ec) {
  (void)handle.backend->updateRobotState(std::chrono::milliseconds(1));
  std::array<double, 16> pose{};
  if (handle.backend->getStateDataMatrix16(RtSupportedFields::tcpPose_m, pose) == 0) {
    ec.clear();
    return pose;
  }
  const auto live_posture = handle.backend->cartPosture(CoordinateType::flangeInBase, ec);
  if (!ec) {
    Utils::postureToTransArray(live_posture, pose);
    return pose;
  }
  ec = std::make_error_code(std::errc::resource_unavailable_try_again);
  return {};
}

void publish_custom(detail::CompatRobotHandle &handle,
                    const std::string &topic,
                    const std::string &payload,
                    error_code &ec) {
  handle.backend->sendCustomData(topic, payload, ec);
}


void publish_joint_command(detail::CompatRtControllerHandle6 &ctx,
                           const std::array<double, 6> &values,
                           bool finished,
                           error_code &ec) {
  rokae_xmate3_ros2::runtime::rt_command_bridge::publishCommand(
      *ctx.robot->backend,
      ctx.loop->sequence,
      rokae_xmate3_ros2::runtime::rt_command_bridge::CommandKind::JointPosition,
      values,
      finished,
      ec,
      "independent_rt",
      static_cast<int>(ctx.loop->current_mode));
}

void publish_cartesian_command(detail::CompatRtControllerHandle6 &ctx,
                               const std::array<double, 6> &values,
                               bool finished,
                               error_code &ec) {
  rokae_xmate3_ros2::runtime::rt_command_bridge::publishCommand(
      *ctx.robot->backend,
      ctx.loop->sequence,
      rokae_xmate3_ros2::runtime::rt_command_bridge::CommandKind::CartesianPosition,
      values,
      finished,
      ec,
      "independent_rt",
      static_cast<int>(ctx.loop->current_mode));
}

void publish_torque_command(detail::CompatRtControllerHandle6 &ctx,
                            const std::array<double, 6> &values,
                            bool finished,
                            error_code &ec) {
  rokae_xmate3_ros2::runtime::rt_command_bridge::publishCommand(
      *ctx.robot->backend,
      ctx.loop->sequence,
      rokae_xmate3_ros2::runtime::rt_command_bridge::CommandKind::Torque,
      values,
      finished,
      ec,
      "independent_rt",
      static_cast<int>(ctx.loop->current_mode));
}

void wait_cartesian_target(detail::CompatRobotHandle &handle,
                           const std::array<double, 6> &target,
                           std::chrono::milliseconds timeout,
                           double translation_tolerance,
                           double angular_tolerance,
                           error_code &ec) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto pose = read_pose_state(handle, ec);
    if (ec) return;
    std::array<double, 6> posture{};
    Utils::transArrayToPosture(pose, posture);
    if (compat_rt::cartesianPoseWithinTolerance(posture, target, translation_tolerance, angular_tolerance)) {
      ec.clear();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ec = std::make_error_code(std::errc::timed_out);
}

void set_loop_exception(detail::CompatLoopState &loop, std::exception_ptr eptr) {
  std::lock_guard<std::mutex> lock(loop.exception_mutex);
  loop.loop_exception = std::move(eptr);
}

std::exception_ptr take_loop_exception(detail::CompatLoopState &loop) {
  std::lock_guard<std::mutex> lock(loop.exception_mutex);
  auto eptr = loop.loop_exception;
  loop.loop_exception = nullptr;
  return eptr;
}

void rethrow_stored_loop_exception(detail::CompatLoopState &loop) {
  auto eptr = take_loop_exception(loop);
  if (eptr) {
    std::rethrow_exception(eptr);
  }
}


/**
 * @brief Convert a row-major 4x4 transform array into an Eigen matrix.
 * @param values Row-major homogeneous transform.
 * @return Eigen::Matrix4d equivalent of @p values.
 * @throws No exception.
 * @note The helper assumes the input passed earlier finite-value validation.
 */
Eigen::Matrix4d array16_to_matrix4d(const std::array<double, 16> &values) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      matrix(row, col) = values[static_cast<std::size_t>(row) * 4 + static_cast<std::size_t>(col)];
    }
  }
  return matrix;
}

/**
 * @brief Validate that a Cartesian target remains inside the configured RT safety box.
 * @param target Cartesian target expressed as [x, y, z, rx, ry, rz].
 * @param lengths Safety-box dimensions in metres.
 * @param frame Safety-box frame expressed as a row-major homogeneous transform.
 * @return `true` when the target is inside the configured box or when no box is configured.
 * @throws No exception.
 * @note A zero-sized box is treated as disabled to preserve compatibility with the official-style API default.
 */
bool cartesian_target_within_limit(const std::array<double, 6> &target,
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

}  // namespace

BaseMotionControl::BaseMotionControl(std::shared_ptr<detail::CompatRtControllerHandle6> impl) noexcept
    : impl_(std::move(impl)) {}

BaseMotionControl::~BaseMotionControl() = default;

MotionControl<MotionControlMode::RtCommand>::MotionControl(std::shared_ptr<detail::CompatRtControllerHandle6> impl) noexcept
    : BaseMotionControl(std::move(impl)) {}

RtMotionControl<WorkType::collaborative, 6>::RtMotionControl(
    std::shared_ptr<detail::CompatRtControllerHandle6> impl) noexcept
    : MotionControl<MotionControlMode::RtCommand>(std::move(impl)) {}

RtMotionControlCobot<6>::RtMotionControlCobot(std::shared_ptr<detail::CompatRtControllerHandle6> impl) noexcept
    : RtMotionControl<WorkType::collaborative, 6>(std::move(impl)) {}

void MotionControl<MotionControlMode::RtCommand>::reconnectNetwork(error_code &ec) noexcept {
  rokae_xmate3_ros2::runtime::rt_command_bridge::publishMetadata(*impl_->robot->backend, "independent_rt", ec);
}

void MotionControl<MotionControlMode::RtCommand>::disconnectNetwork() noexcept {
  if (!impl_) return;
  error_code ec;
  rokae_xmate3_ros2::runtime::rt_command_bridge::publishStop(*impl_->robot->backend, ec);
  impl_->loop->move_started.store(false);
}

template <class Command>
void MotionControl<MotionControlMode::RtCommand>::setControlLoop(const std::function<Command(void)> &callback,
                                                                 int,
                                                                 bool useStateDataInLoop) noexcept {
  if (!impl_) {
    return;
  }
  impl_->loop->callback = [callback]() { return std::any(callback()); };
  impl_->loop->use_state_data_in_loop = useStateDataInLoop;
}

void MotionControl<MotionControlMode::RtCommand>::startLoop(bool blocking) {
  rethrow_stored_loop_exception(*impl_->loop);
  if (!impl_->loop->callback) {
    throw RealtimeControlException("RtMotionControlCobot::startLoop requires a configured callback");
  }
  if (!impl_->loop->move_started.load()) {
    throw RealtimeStateException("RtMotionControlCobot::startLoop requires startMove() before entering loop");
  }
  if (impl_->loop->running.load()) {
    throw RealtimeStateException("RtMotionControlCobot::startLoop called while loop is already running");
  }
  bool strict_loop_timing = false;
  {
    std::string active_profile;
    std::vector<RuntimeProfileCapability> profiles;
    std::vector<RuntimeOptionDescriptor> options;
    error_code profile_ec;
    if (impl_->robot->backend->getProfileCapabilities(active_profile, profiles, options, profile_ec) &&
        active_profile == "hard_1khz") {
      strict_loop_timing = true;
    }
  }
  auto body = [ctx = impl_, strict_loop_timing]() {
    auto next_tick = std::chrono::steady_clock::now();
    int loop_cycle_count = 0;
    while (ctx->loop->running.load()) {
      try {
        const auto now = std::chrono::steady_clock::now();
        if (strict_loop_timing &&
            loop_cycle_count > 10 &&
            now > next_tick + kStrictLoopLatenessTolerance) {
          error_code ec = std::make_error_code(std::errc::timed_out);
          rokae_xmate3_ros2::runtime::rt_command_bridge::publishStop(*ctx->robot->backend, ec);
          ctx->loop->running.store(false);
          throw RealtimeControlException("RtMotionControlCobot::startLoop deadline miss (>1kHz strict window)", ec);
        }
        if (ctx->loop->use_state_data_in_loop) {
          ctx->robot->backend->updateRobotState(std::chrono::milliseconds(1));
        }
        error_code ec;
        const auto value = ctx->loop->callback();
        bool finished = false;
        if (value.type() == typeid(JointPosition)) {
          const auto &command = std::any_cast<const JointPosition &>(value);
          std::array<double, 6> out{};
          for (std::size_t i = 0; i < 6 && i < command.joints.size(); ++i) out[i] = command.joints[i];
          finished = command.isFinished() != 0;
          publish_joint_command(*ctx, out, finished, ec);
        } else if (value.type() == typeid(CartesianPosition)) {
          const auto &command = std::any_cast<const CartesianPosition &>(value);
          finished = command.isFinished() != 0;
          publish_cartesian_command(*ctx, posture_to_array(command), finished, ec);
        } else if (value.type() == typeid(Torque)) {
          const auto &command = std::any_cast<const Torque &>(value);
          std::array<double, 6> out{};
          for (std::size_t i = 0; i < 6 && i < command.tau.size(); ++i) out[i] = command.tau[i];
          finished = command.isFinished() != 0;
          publish_torque_command(*ctx, out, finished, ec);
        } else {
          ec = std::make_error_code(std::errc::invalid_argument);
        }
        if (ec || finished) {
          rokae_xmate3_ros2::runtime::rt_command_bridge::publishStop(*ctx->robot->backend, ec);
          ctx->loop->running.store(false);
          if (ec) {
            throw RealtimeControlException("RtMotionControlCobot::startLoop command dispatch failed", ec);
          }
          break;
        }
        next_tick += kStrictLoopPeriod;
        ++loop_cycle_count;
        std::this_thread::sleep_until(next_tick);
      } catch (const RealtimeControlException &) {
        set_loop_exception(*ctx->loop, std::current_exception());
        ctx->loop->running.store(false);
        break;
      } catch (const Exception &) {
        set_loop_exception(*ctx->loop, std::current_exception());
        error_code ignore_ec;
        rokae_xmate3_ros2::runtime::rt_command_bridge::publishStop(*ctx->robot->backend, ignore_ec);
        ctx->loop->running.store(false);
        break;
      } catch (...) {
        set_loop_exception(*ctx->loop, std::make_exception_ptr(
            RealtimeControlException("RtMotionControlCobot::startLoop unexpected runtime failure")));
        error_code ignore_ec;
        rokae_xmate3_ros2::runtime::rt_command_bridge::publishStop(*ctx->robot->backend, ignore_ec);
        ctx->loop->running.store(false);
        break;
      }
    }
  };
  impl_->loop->running.store(true);
  if (blocking) {
    body();
    rethrow_stored_loop_exception(*impl_->loop);
  } else {
    impl_->loop->worker = std::thread(body);
  }
}

void MotionControl<MotionControlMode::RtCommand>::stopLoop() {
  if (!impl_) return;
  impl_->loop->running.store(false);
  if (impl_->loop->worker.joinable()) {
    impl_->loop->worker.join();
  }
  rethrow_stored_loop_exception(*impl_->loop);
}

void RtMotionControl<WorkType::collaborative, 6>::startMove(RtControllerMode rtMode) {
  rethrow_stored_loop_exception(*impl_->loop);
  if (impl_->loop->move_started.load()) {
    throw RealtimeStateException("RtMotionControl::startMove repeated while a realtime move session is active");
  }
  error_code ec;
  impl_->robot->backend->setRtControlMode(rtMode, ec);
  throw_if_error<RealtimeControlException>(ec, "RtMotionControl::startMove");
  impl_->loop->current_mode = rtMode;
  impl_->loop->move_started.store(true);
}

void MotionControl<MotionControlMode::RtCommand>::stopMove() {
  if (!impl_) return;
  error_code ec;
  rokae_xmate3_ros2::runtime::rt_command_bridge::publishStop(*impl_->robot->backend, ec);
  impl_->loop->move_started.store(false);
}

void RtMotionControl<WorkType::collaborative, 6>::MoveJ(double speed,
                                                        const std::array<double, 6> &start,
                                                        const std::array<double, 6> &target) {
  if (!compat_rt::isSpeedFactorValid(speed)) {
    throw RealtimeParameterException("RtMotionControlCobot::MoveJ invalid speed",
                                     std::make_error_code(std::errc::invalid_argument));
  }
  startMove(RtControllerMode::jointPosition);
  error_code ec;
  const auto live = read_joint_state(*impl_->robot, ec);
  throw_if_error<RealtimeParameterException>(ec, "RtMotionControlCobot::MoveJ read state");
  double max_joint_error = 0.0;
  for (std::size_t i = 0; i < live.size(); ++i) {
    max_joint_error = std::max(max_joint_error, std::fabs(live[i] - start[i]));
  }
  if (max_joint_error > 1e-2) {
    throw RealtimeParameterException("RtMotionControlCobot::MoveJ start validation failed",
                                     std::make_error_code(std::errc::invalid_argument));
  }
  const auto timeout = std::chrono::milliseconds(
      static_cast<int>(std::clamp(30000.0 / std::max(speed, 0.05), 5000.0, 60000.0)));
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto next_tick = std::chrono::steady_clock::now();
  bool reached = false;
  while (std::chrono::steady_clock::now() < deadline) {
    publish_joint_command(*impl_, target, false, ec);
    throw_if_error<RealtimeMotionException>(ec, "RtMotionControlCobot::MoveJ publish");
    const auto current = read_joint_state(*impl_->robot, ec);
    throw_if_error<RealtimeMotionException>(ec, "RtMotionControlCobot::MoveJ read back state");
    double current_error = 0.0;
    for (std::size_t i = 0; i < current.size(); ++i) {
      current_error = std::max(current_error, std::fabs(current[i] - target[i]));
    }
    if (current_error <= 2e-2) {
      reached = true;
      break;
    }
    next_tick += std::chrono::milliseconds(1);
    std::this_thread::sleep_until(next_tick);
  }
  if (!reached) {
    ec = std::make_error_code(std::errc::timed_out);
    throw_if_error<RealtimeMotionException>(ec, "RtMotionControlCobot::MoveJ");
  }
}

void RtMotionControl<WorkType::collaborative, 6>::MoveL(double speed,
                                                        CartesianPosition &start,
                                                        CartesianPosition &target) {
  if (!compat_rt::isSpeedFactorValid(speed)) {
    throw RealtimeParameterException("RtMotionControlCobot::MoveL invalid speed",
                                     std::make_error_code(std::errc::invalid_argument));
  }
  startMove(RtControllerMode::cartesianPosition);
  error_code ec;
  const auto live_pose = read_pose_state(*impl_->robot, ec);
  throw_if_error<RealtimeParameterException>(ec, "RtMotionControlCobot::MoveL read state");
  std::array<double, 6> live{};
  Utils::transArrayToPosture(live_pose, live);
  const auto start_pose = posture_to_array(start);
  const auto target_pose = posture_to_array(target);
  if (!compat_rt::cartesianPoseWithinTolerance(live, start_pose, 3e-2, 6e-2)) {
    throw RealtimeParameterException("RtMotionControlCobot::MoveL start validation failed",
                                     std::make_error_code(std::errc::invalid_argument));
  }
  publish_cartesian_command(*impl_, target_pose, false, ec);
  throw_if_error<RealtimeMotionException>(ec, "RtMotionControlCobot::MoveL publish");
  wait_cartesian_target(*impl_->robot,
                        target_pose,
                        std::chrono::milliseconds(static_cast<int>(std::clamp(30000.0 / std::max(speed, 0.05), 5000.0, 60000.0))),
                        3e-2,
                        6e-2,
                        ec);
  throw_if_error<RealtimeMotionException>(ec, "RtMotionControlCobot::MoveL");
}

void RtMotionControl<WorkType::collaborative, 6>::MoveC(double speed,
                                                        CartesianPosition &start,
                                                        CartesianPosition &aux,
                                                        CartesianPosition &target) {
  if (!compat_rt::isSpeedFactorValid(speed)) {
    throw RealtimeParameterException("RtMotionControlCobot::MoveC invalid speed",
                                     std::make_error_code(std::errc::invalid_argument));
  }
  startMove(RtControllerMode::cartesianPosition);

  error_code ec;
  const auto live_pose = read_pose_state(*impl_->robot, ec);
  throw_if_error<RealtimeParameterException>(ec, "RtMotionControlCobot::MoveC read state");

  std::array<double, 6> live{};
  Utils::transArrayToPosture(live_pose, live);
  const auto start_pose = posture_to_array(start);
  const auto aux_pose = posture_to_array(aux);
  const auto target_pose = posture_to_array(target);
  if (!finite_array6(start_pose) || !finite_array6(aux_pose) || !finite_array6(target_pose)) {
    throw RealtimeParameterException("RtMotionControlCobot::MoveC pose contains NaN/Inf",
                                     std::make_error_code(std::errc::invalid_argument));
  }
  if (!compat_rt::cartesianPoseWithinTolerance(live, start_pose, 4e-2, 8e-2)) {
    throw RealtimeParameterException("RtMotionControlCobot::MoveC start validation failed",
                                     std::make_error_code(std::errc::invalid_argument));
  }
  if (!cartesian_target_within_limit(aux_pose, impl_->loop->cartesian_limit_lengths, impl_->loop->cartesian_limit_frame) ||
      !cartesian_target_within_limit(target_pose, impl_->loop->cartesian_limit_lengths, impl_->loop->cartesian_limit_frame)) {
    throw RealtimeParameterException("RtMotionControlCobot::MoveC cartesian limit violated",
                                     std::make_error_code(std::errc::result_out_of_range));
  }

  const auto geometry = compat_rt::fitCircularArc(start_pose, aux_pose, target_pose, ec);
  if (!geometry) {
    if (ec == std::make_error_code(std::errc::invalid_argument)) {
      const Eigen::Vector3d p1(start_pose[0], start_pose[1], start_pose[2]);
      const Eigen::Vector3d p2(aux_pose[0], aux_pose[1], aux_pose[2]);
      const Eigen::Vector3d p3(target_pose[0], target_pose[1], target_pose[2]);
      const double cross_norm = (p2 - p1).cross(p3 - p1).squaredNorm();
      if (cross_norm < 1e-10) {
        throw RealtimeParameterException("RtMotionControlCobot::MoveC degenerate arc", ec);
      }
      throw RealtimeParameterException("RtMotionControlCobot::MoveC invalid radius", ec);
    }
    throw RealtimeParameterException("RtMotionControlCobot::MoveC sweep angle is invalid", ec);
  }

  const int steps = compat_rt::estimateArcInterpolationSteps(speed, geometry->radius, geometry->sweep_angle);
  const Eigen::Quaterniond q_start = rokae_xmate3_ros2::runtime::pose_utils::rpyToQuaternion(start_pose[3], start_pose[4], start_pose[5]);
  const Eigen::Quaterniond q_target = rokae_xmate3_ros2::runtime::pose_utils::rpyToQuaternion(target_pose[3], target_pose[4], target_pose[5]);
  const Eigen::Vector3d start_point(start_pose[0], start_pose[1], start_pose[2]);

  for (int i = 1; i <= steps; ++i) {
    const double alpha = static_cast<double>(i) / static_cast<double>(steps);
    const Eigen::AngleAxisd rot(alpha * geometry->sweep_angle, geometry->axis);
    const Eigen::Vector3d point = geometry->center + rot * (start_point - geometry->center);
    const Eigen::Quaterniond q = q_start.slerp(alpha, q_target);
    const auto pose = compat_rt::quaternionToPose(q, point);
    if (!cartesian_target_within_limit(pose, impl_->loop->cartesian_limit_lengths, impl_->loop->cartesian_limit_frame)) {
      throw RealtimeParameterException("RtMotionControlCobot::MoveC cartesian limit violated during arc",
                                       std::make_error_code(std::errc::result_out_of_range));
    }
    publish_cartesian_command(*impl_, pose, i == steps, ec);
    throw_if_error<RealtimeMotionException>(ec, "RtMotionControlCobot::MoveC publish");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  wait_cartesian_target(*impl_->robot,
                        target_pose,
                        std::chrono::milliseconds(static_cast<int>(std::clamp(40000.0 / std::max(speed, 0.05), 8000.0, 80000.0))),
                        4e-2,
                        8e-2,
                        ec);
  throw_if_error<RealtimeMotionException>(ec, "RtMotionControlCobot::MoveC");
}

bool RtMotionControl<WorkType::collaborative, 6>::setFilterLimit(bool limit_rate, double cutoff_frequency) noexcept {
  if (!std::isfinite(cutoff_frequency) || cutoff_frequency < 0.0 || cutoff_frequency > 1000.0) {
    return false;
  }
  error_code ec;
  publish_custom(*impl_->robot,
                 rokae_xmate3_ros2::runtime::rt_topics::kConfigFilterLimit,
                 build_filter_limit_payload(limit_rate, cutoff_frequency),
                 ec);
  return !ec;
}

void RtMotionControl<WorkType::collaborative, 6>::setCartesianLimit(const std::array<double, 3> &lengths,
                                                                    const std::array<double, 16> &frame,
                                                                    error_code &ec) noexcept {
  if (!finite_array3_non_negative(lengths) || !finite_matrix16(frame)) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return;
  }
  impl_->loop->cartesian_limit_lengths = lengths;
  impl_->loop->cartesian_limit_frame = frame;
  publish_custom(*impl_->robot,
                 rokae_xmate3_ros2::runtime::rt_topics::kConfigCartesianLimit,
                 build_cartesian_limit_payload(lengths, frame),
                 ec);
}

void RtMotionControlCobot<6>::setJointImpedance(const std::array<double, 6> &factor, error_code &ec) noexcept {
  if (!finite_non_negative_array6(factor)) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return;
  }
  impl_->joint_impedance = factor;
  publish_custom(*impl_->robot, rokae_xmate3_ros2::runtime::rt_topics::kConfigJointImpedance, serialize_values(factor), ec);
}

void RtMotionControlCobot<6>::setCartesianImpedance(const std::array<double, 6> &factor, error_code &ec) noexcept {
  if (!finite_non_negative_array6(factor)) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return;
  }
  impl_->cartesian_impedance = factor;
  publish_custom(*impl_->robot, rokae_xmate3_ros2::runtime::rt_topics::kConfigCartesianImpedance, serialize_values(factor), ec);
}

void RtMotionControlCobot<6>::setCollisionBehaviour(const std::array<double, 6> &torqueThresholds, error_code &ec) noexcept {
  if (!finite_non_negative_array6(torqueThresholds)) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return;
  }
  impl_->collision_thresholds = torqueThresholds;
  publish_custom(*impl_->robot,
                 rokae_xmate3_ros2::runtime::rt_topics::kConfigCollisionBehaviourThresholds,
                 serialize_values(torqueThresholds),
                 ec);
}

void RtMotionControl<WorkType::collaborative, 6>::setEndEffectorFrame(const std::array<double, 16> &frame, error_code &ec) noexcept {
  if (!finite_matrix16(frame)) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(impl_->robot->mutex);
    impl_->robot->model_f_t_ee = frame;
  }
  publish_custom(*impl_->robot, rokae_xmate3_ros2::runtime::rt_topics::kConfigEndEffectorFrame, serialize_matrix16(frame), ec);
}

void RtMotionControl<WorkType::collaborative, 6>::setLoad(const Load &load, error_code &ec) noexcept {
  if (!std::isfinite(load.mass) || load.mass < 0.0 ||
      !std::isfinite(load.cog[0]) || !std::isfinite(load.cog[1]) || !std::isfinite(load.cog[2]) ||
      !std::isfinite(load.inertia[0]) || !std::isfinite(load.inertia[1]) || !std::isfinite(load.inertia[2]) ||
      load.inertia[0] < 0.0 || load.inertia[1] < 0.0 || load.inertia[2] < 0.0) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(impl_->robot->mutex);
    impl_->robot->model_load_cache = load;
  }
  publish_custom(*impl_->robot, rokae_xmate3_ros2::runtime::rt_topics::kConfigLoad, serialize_load(load), ec);
}

void MotionControl<MotionControlMode::RtCommand>::startReceiveRobotState(const std::vector<std::string> &) {
  throw RealtimeControlException(
      "MotionControl<RtCommand>::startReceiveRobotState is not supported on xMate6 public lane",
      make_error_code(SdkError::not_implemented));
}

void MotionControl<MotionControlMode::RtCommand>::stopReceiveRobotState() noexcept {}

void MotionControl<MotionControlMode::RtCommand>::updateRobotState() {
  throw RealtimeStateException(
      "MotionControl<RtCommand>::updateRobotState is not supported on xMate6 public lane",
      make_error_code(SdkError::not_implemented));
}

void MotionControl<MotionControlMode::RtCommand>::automaticErrorRecovery(error_code &ec) noexcept {
  detail::setPublicLaneUnsupported(impl_->robot, ec, "rt.deprecated_recovery");
}

void RtMotionControlCobot<6>::setFilterFrequency(double jointFrequency,
                                                 double cartesianFrequency,
                                                 double torqueFrequency,
                                                 error_code &ec) noexcept {
  const std::array<double, 6> values = {jointFrequency, cartesianFrequency, torqueFrequency, 0.0, 0.0, 0.0};
  if (!std::isfinite(jointFrequency) || !std::isfinite(cartesianFrequency) || !std::isfinite(torqueFrequency) ||
      jointFrequency < 1.0 || jointFrequency > 1000.0 ||
      cartesianFrequency < 1.0 || cartesianFrequency > 1000.0 ||
      torqueFrequency < 1.0 || torqueFrequency > 1000.0) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return;
  }
  impl_->filter_frequencies = {jointFrequency, cartesianFrequency, torqueFrequency};
  publish_custom(*impl_->robot,
                 rokae_xmate3_ros2::runtime::rt_topics::kConfigFilterFrequency,
                 serialize_values(values),
                 ec);
}

void RtMotionControlCobot<6>::setCartesianImpedanceDesiredTorque(const std::array<double, 6> &torque, error_code &ec) noexcept {
  constexpr std::array<double, 6> kDesiredWrenchLimit = {60.0, 60.0, 60.0, 30.0, 30.0, 30.0};
  if (!finite_array6(torque)) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return;
  }
  for (std::size_t i = 0; i < torque.size(); ++i) {
    if (std::fabs(torque[i]) > kDesiredWrenchLimit[i]) {
      ec = std::make_error_code(std::errc::invalid_argument);
      return;
    }
  }
  impl_->desired_cartesian_torque = torque;
  publish_custom(*impl_->robot, rokae_xmate3_ros2::runtime::rt_topics::kConfigCartesianDesiredWrench, serialize_values(torque), ec);
}

void RtMotionControlCobot<6>::setTorqueFilterCutOffFrequency(double frequency, error_code &ec) noexcept {
  if (!std::isfinite(frequency) || frequency < 1.0 || frequency > 1000.0) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return;
  }
  impl_->torque_cutoff_frequency = frequency;
  publish_custom(*impl_->robot, rokae_xmate3_ros2::runtime::rt_topics::kConfigTorqueCutoffFrequency, std::to_string(frequency), ec);
}

void RtMotionControlCobot<6>::setFcCoor(const std::array<double, 16> &frame,
                                        FrameType type,
                                        error_code &ec) noexcept {
  if (!finite_matrix16(frame) || !valid_force_control_type(type)) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return;
  }
  impl_->force_control_frame = frame;
  impl_->force_control_type = type;
  publish_custom(*impl_->robot,
                 rokae_xmate3_ros2::runtime::rt_topics::kConfigForceControlFrame,
                 std::string{"type="} + std::to_string(static_cast<int>(type)) + ";values=" + serialize_matrix16(frame),
                 ec);
}

std::weak_ptr<RtMotionControlCobot<6>> BaseCobot::getRtMotionController() {
  std::lock_guard<std::mutex> lock(handle_->mutex);
  if (handle_->rt_controller_owner) {
    return handle_->rt_controller_owner;
  }
  if (auto existing = handle_->rt_controller.lock()) {
    handle_->rt_controller_owner = existing;
    return existing;
  }
  auto controller = std::shared_ptr<RtMotionControlCobot<6>>(new RtMotionControlCobot<6>(std::make_shared<detail::CompatRtControllerHandle6>(handle_)));
  handle_->rt_controller_owner = controller;
  handle_->rt_controller = controller;
  return controller;
}

template void MotionControl<MotionControlMode::RtCommand>::setControlLoop<JointPosition>(
    const std::function<JointPosition(void)> &callback,
    int priority,
    bool useStateDataInLoop) noexcept;
template void MotionControl<MotionControlMode::RtCommand>::setControlLoop<CartesianPosition>(
    const std::function<CartesianPosition(void)> &callback,
    int priority,
    bool useStateDataInLoop) noexcept;
template void MotionControl<MotionControlMode::RtCommand>::setControlLoop<Torque>(
    const std::function<Torque(void)> &callback,
    int priority,
    bool useStateDataInLoop) noexcept;

}  // namespace rokae
