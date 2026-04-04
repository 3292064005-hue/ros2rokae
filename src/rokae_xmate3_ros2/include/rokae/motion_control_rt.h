#ifndef ROKAE_MOTION_CONTROL_RT_H
#define ROKAE_MOTION_CONTROL_RT_H

#include <array>
#include <chrono>
#include <functional>
#include <memory>

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
template <unsigned short DoF>
class RtMotionControlCobot;
template <WorkType Wt, unsigned short DoF>
using RtMotionControl = RtMotionControlCobot<DoF>;

/**
 * @brief xMate 6-axis realtime control facade.
 * @details The compatibility layer keeps the public control surface free of ROS transport details.
 *          Runtime commands are routed to the existing semantic RT bridge inside the backend package.
 */
template <>
class XCORE_API RtMotionControlCobot<6> {
 public:
  RtMotionControlCobot(const RtMotionControlCobot &) = delete;
  RtMotionControlCobot &operator=(const RtMotionControlCobot &) = delete;
  RtMotionControlCobot(RtMotionControlCobot &&) noexcept;
  RtMotionControlCobot &operator=(RtMotionControlCobot &&) noexcept;
  ~RtMotionControlCobot();

  void reconnectNetwork(error_code &ec) noexcept;
  void disconnectNetwork() noexcept;

  /**
   * @brief Install a 1 kHz joint-position control callback.
   * @param callback User callback that returns the next joint command.
   * @param priority Optional RT thread priority hint.
   * @param useStateDataInLoop Whether state data should be refreshed before each callback.
   * @throws RealtimeControlException when the control loop cannot be armed.
   * @note The callback is executed by the compatibility layer thread and must be non-blocking.
   */
  void setControlLoop(const std::function<JointPosition(void)> &callback,
                      int priority = 0,
                      bool useStateDataInLoop = false);
  void setControlLoop(const std::function<CartesianPosition(void)> &callback,
                      int priority = 0,
                      bool useStateDataInLoop = false);
  void setControlLoop(const std::function<Torque(void)> &callback,
                      int priority = 0,
                      bool useStateDataInLoop = false);

  void startLoop(bool blocking = true);
  void stopLoop();
  void startMove(RtControllerMode rtMode);
  void stopMove();

  void MoveJ(double speed,
             const std::array<double, 6> &start,
             const std::array<double, 6> &target);
  void MoveL(double speed,
             CartesianPosition &start,
             CartesianPosition &target);
  /**
   * @brief Request a realtime circular Cartesian motion on the xMate6 compatibility lane.
   * @param speed Relative motion speed factor. The value must be finite and strictly positive.
   * @param start Expected current Cartesian start pose used for live-state validation.
   * @param aux Auxiliary arc point defining the circle together with @p start and @p target.
   * @param target Target Cartesian pose.
   * @throws RealtimeParameterException When the speed is invalid, the live pose does not match @p start,
   *         the three points do not form a valid circle, or a configured Cartesian limit would be violated.
   * @throws RealtimeMotionException When command publication fails or the target is not reached before timeout.
   * @note The implementation performs simulation-grade geometric circle fitting and direct RT Cartesian dispatch;
   *       it preserves the official surface but does not claim controller-grade UDP/1 kHz fidelity.
   */
  void MoveC(double speed,
             CartesianPosition &start,
             CartesianPosition &aux,
             CartesianPosition &target);

  /**
   * @brief Configure the direct-RT filter limiter state exposed to the runtime bridge.
   * @param limit_rate Enables or disables the additional limiter.
   * @param cutoff_frequency Cutoff frequency in hertz used when the limiter is enabled.
   * @return `true` when the runtime configuration topic accepted the update; `false` on range/NaN validation failure or publish rejection.
   * @throws No exception.
   * @note Disabling the limiter keeps the command channel valid and instructs the runtime bridge to bypass the
   *       extra low-pass clamp.
   */
  bool setFilterLimit(bool limit_rate, double cutoff_frequency) noexcept;
  void setCartesianLimit(const std::array<double, 3> &lengths,
                         const std::array<double, 16> &frame,
                         error_code &ec) noexcept;
  void setJointImpedance(const std::array<double, 6> &factor, error_code &ec) noexcept;
  void setCartesianImpedance(const std::array<double, 6> &factor, error_code &ec) noexcept;
  /**
   * @brief Configure joint-space collision thresholds for the runtime semantic snapshot.
   * @param torqueThresholds Six-axis threshold vector.
   * @param ec Output error code.
   * @throws No exception.
   * @note The thresholds are published to the dedicated collision-behaviour topic and are no longer multiplexed
   *       through the Cartesian desired-wrench channel.
   */
  void setCollisionBehaviour(const std::array<double, 6> &torqueThresholds, error_code &ec) noexcept;
  void setEndEffectorFrame(const std::array<double, 16> &frame, error_code &ec) noexcept;
  void setLoad(const Load &load, error_code &ec) noexcept;
  void setFilterFrequency(double jointFrequency,
                          double cartesianFrequency,
                          double torqueFrequency,
                          error_code &ec) noexcept;
  void setCartesianImpedanceDesiredTorque(const std::array<double, 6> &torque, error_code &ec) noexcept;
  void setTorqueFilterCutOffFrequency(double frequency, error_code &ec) noexcept;
  void setFcCoor(const std::array<double, 16> &frame,
                 FrameType type,
                 error_code &ec) noexcept;
  void automaticErrorRecovery(error_code &ec) noexcept;

 private:
  explicit RtMotionControlCobot(std::shared_ptr<detail::CompatRtControllerHandle6> impl);
  std::shared_ptr<detail::CompatRtControllerHandle6> impl_;

  friend class BaseCobot;
  friend class Cobot<6>;
};

}  // namespace rokae

#endif  // ROKAE_MOTION_CONTROL_RT_H
