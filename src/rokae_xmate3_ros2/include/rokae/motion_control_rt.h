#ifndef ROKAE_MOTION_CONTROL_RT_H
#define ROKAE_MOTION_CONTROL_RT_H

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "rokae/base.h"
#include "rokae/data_types.h"
#include "rokae/exception.h"

namespace rokae {

namespace detail {
struct CompatRtControllerHandle6;
}

class BaseCobot;
template <unsigned short DoF>
class Cobot;
template <MotionControlMode ControlMode>
class MotionControl;
template <WorkType Wt, unsigned short DoF>
class RtMotionControl;
template <unsigned short DoF>
class RtMotionControlCobot;

/**
 * @brief Official-shaped RT motion control base for the xMateER3 compatibility lane.
 */
class XCORE_API BaseMotionControl {
 public:
  BaseMotionControl(const BaseMotionControl &) = delete;
  BaseMotionControl &operator=(const BaseMotionControl &) = delete;
  BaseMotionControl(BaseMotionControl &&) noexcept = default;
  BaseMotionControl &operator=(BaseMotionControl &&) noexcept = default;
  virtual ~BaseMotionControl();

 protected:
  explicit BaseMotionControl(std::shared_ptr<detail::CompatRtControllerHandle6> impl) noexcept;
  std::shared_ptr<detail::CompatRtControllerHandle6> impl_;
};

/**
 * @brief Official-shaped realtime command specialization.
 * @note Deprecated helpers are retained for source compatibility and now forward to the robot-scoped RT state stream on the xMateER3 public lane. They remain deprecated because the authoritative API lives on BaseRobot/Robot_T.
 */
template <>
class XCORE_API MotionControl<MotionControlMode::RtCommand> : public BaseMotionControl {
 public:
  void disconnectNetwork() noexcept;
  void reconnectNetwork(error_code &ec) noexcept;

  template <class Command>
  void setControlLoop(const std::function<Command(void)> &callback, int priority = 0, bool useStateDataInLoop = false) noexcept;
  template <class Callback>
  void setControlLoop(Callback callback, int priority = 0, bool useStateDataInLoop = false) noexcept {
    using Command = std::invoke_result_t<Callback>;
    setControlLoop<Command>(
        std::function<Command(void)>(std::move(callback)),
        priority,
        useStateDataInLoop);
  }

  void startLoop(bool blocking = true);
  void stopLoop();
  void stopMove();

  /**
   * @brief Deprecated RT state-stream helper that forwards to the robot-scoped state channel.
   * @param fields Requested RT field names.
   * @throws RealtimeControlException If the backend rejects the subscription.
   * @note This wrapper always uses a 1 ms interval so legacy RT loops preserve their timing contract.
   */
  [[deprecated("Use Robot_T::startReceiveRobotState(interval, fields) instead")]]
  void startReceiveRobotState(const std::vector<std::string> &fields);
  /**
   * @brief Stops the robot-scoped RT state stream used by deprecated RT helper code.
   * @note Safe to call when the stream is already inactive.
   */
  [[deprecated("Use BaseRobot::stopReceiveRobotState() instead")]]
  void stopReceiveRobotState() noexcept;
  /**
   * @brief Pulls one RT state frame through the robot-scoped state channel.
   * @throws RealtimeControlException If transport/update fails.
   * @note The legacy wrapper keeps a fixed 1 ms timeout to match the public RT compatibility lane.
   */
  [[deprecated("Use BaseRobot::updateRobotState() instead")]]
  void updateRobotState();
  template <typename R>
  [[deprecated("Use BaseRobot::getStateData(fieldName, data) instead")]]
  int getStateData(const std::string &field_name, R &data) noexcept {
    if constexpr (std::is_same_v<R, std::array<double, 6>>) {
      return getStateDataArray6(field_name, data);
    } else if constexpr (std::is_same_v<R, std::array<double, 3>>) {
      return getStateDataArray3(field_name, data);
    } else if constexpr (std::is_same_v<R, std::array<double, 16>>) {
      return getStateDataMatrix16(field_name, data);
    } else if constexpr (std::is_same_v<R, double>) {
      return getStateDataScalarDouble(field_name, data);
    } else if constexpr (std::is_same_v<R, bool>) {
      return getStateDataBool(field_name, data);
    }
    return -1;
  }
  [[deprecated("no longer maintained function")]]
  void automaticErrorRecovery(error_code &ec) noexcept;

 protected:
  explicit MotionControl(std::shared_ptr<detail::CompatRtControllerHandle6> impl) noexcept;
  /** Typed legacy readers used by deprecated getStateData(field, data) dispatch.
   *  Return value: 0 on success, -1 when the field is absent or the requested type does not match.
   *  Boundary behavior: these helpers never throw for simple cache misses; transport failures are surfaced
   *  by the state update calls that precede them.
   */
  int getStateDataArray6(const std::string &field_name, std::array<double, 6> &data) noexcept;
  int getStateDataArray3(const std::string &field_name, std::array<double, 3> &data) noexcept;
  int getStateDataMatrix16(const std::string &field_name, std::array<double, 16> &data) noexcept;
  int getStateDataScalarDouble(const std::string &field_name, double &data) noexcept;
  int getStateDataBool(const std::string &field_name, bool &data) noexcept;
};

/**
 * @brief xMateER3-only RT motion control template specialization.
 */
template <>
class XCORE_API RtMotionControl<WorkType::collaborative, 6> : public MotionControl<MotionControlMode::RtCommand> {
 public:
  void startMove(RtControllerMode rtMode);
  bool setFilterLimit(bool limit_rate, double cutoff_frequency) noexcept;
  void setCartesianLimit(const std::array<double, 3> &lengths,
                         const std::array<double, 16> &frame,
                         error_code &ec) noexcept;
  void setEndEffectorFrame(const std::array<double, 16> &frame, error_code &ec) noexcept;
  void setLoad(const Load &load, error_code &ec) noexcept;
  void MoveJ(double speed,
             const std::array<double, 6> &start,
             const std::array<double, 6> &target);
  void MoveL(double speed,
             CartesianPosition &start,
             CartesianPosition &target);
  void MoveC(double speed,
             CartesianPosition &start,
             CartesianPosition &aux,
             CartesianPosition &target);

 protected:
  explicit RtMotionControl(std::shared_ptr<detail::CompatRtControllerHandle6> impl) noexcept;
};

template <>
class XCORE_API RtMotionControlCobot<6> : public RtMotionControl<WorkType::collaborative, 6> {
 public:
  void setJointImpedance(const std::array<double, 6> &factor, error_code &ec) noexcept;
  void setCartesianImpedance(const std::array<double, 6> &factor, error_code &ec) noexcept;
  void setFilterFrequency(double jointFrequency,
                          double cartesianFrequency,
                          double torqueFrequency,
                          error_code &ec) noexcept;
  void setCartesianImpedanceDesiredTorque(const std::array<double, 6> &torque, error_code &ec) noexcept;
  void setTorqueFilterCutOffFrequency(double frequency, error_code &ec) noexcept;
  void setFcCoor(const std::array<double, 16> &frame,
                 FrameType type,
                 error_code &ec) noexcept;
  void setCollisionBehaviour(const std::array<double, 6> &torqueThresholds, error_code &ec) noexcept;

 private:
  explicit RtMotionControlCobot(std::shared_ptr<detail::CompatRtControllerHandle6> impl) noexcept;
  friend class BaseCobot;
  friend class Cobot<6>;
};

}  // namespace rokae

#endif  // ROKAE_MOTION_CONTROL_RT_H
