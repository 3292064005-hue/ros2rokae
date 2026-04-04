#ifndef ROKAE_ROBOT_H
#define ROKAE_ROBOT_H

#include <array>
#include <chrono>
#include <cfloat>
#include <initializer_list>
#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <vector>

#include "rokae/base.h"
#include "rokae/data_types.h"
#include "rokae/exception.h"
#include "rokae/model.h"
#include "rokae/motion_control_rt.h"
#include "rokae/planner.h"

namespace rokae {

namespace detail {
struct CompatRobotHandle;
}

template <WorkType Wt, unsigned short DoF>
class Robot_T;

template <unsigned short DoF>
class Cobot;

class BaseCobot;
class xMateRobot;

/**
 * @brief Official-SDK-like robot facade base for the xMate6 compatibility lane.
 * @details This class owns an opaque backend handle and exposes the installed ABI surface.
 *          The concrete implementation is compiled into the compatibility library so that the
 *          public headers stay free of ROS2/Gazebo transport types.
 */
class XCORE_API BaseRobot : public Base<BaseRobot> {
 public:
  BaseRobot();
  explicit BaseRobot(const std::string &remoteIP, const std::string &localIP = "");
  BaseRobot(const BaseRobot &) = default;
  BaseRobot(BaseRobot &&other) noexcept;
  BaseRobot &operator=(const BaseRobot &) = default;
  BaseRobot &operator=(BaseRobot &&other) noexcept;
  virtual ~BaseRobot();

  static std::string sdkVersion();

  /**
   * @brief Establish a transport/session connection to the robot configured for this facade.
   * @param ec Output error code. Cleared on success.
   * @throws No exception.
   * @note Repeated calls on an already connected robot are forwarded to the backend, which keeps
   *       the historical idempotent behaviour.
   */
  void connectToRobot(error_code &ec) noexcept;
  /**
   * @brief Rebind the facade to a remote/local endpoint pair and attempt connection.
   * @param remoteIP Remote robot/controller address.
   * @param localIP Local interface address used by realtime transports. Empty keeps backend default selection.
   * @throws No exception.
   * @note This is the primary compatibility entry used by the explicit-error-code lane.
   */
  void connectToRobot(const std::string &remoteIP,
                      const std::string &localIP,
                      error_code &ec) noexcept;
  /**
   * @brief Official-SDK-shaped convenience overload without an explicit error-code out parameter.
   * @param remoteIP Remote robot/controller address.
   * @param localIP Local interface address used by realtime transports. Empty keeps backend default selection.
   * @throws No exception.
   * @note Errors are still recorded through the backend `lastErrorCode()` path; callers that need direct
   *       branching should continue to use the `error_code&` overload.
   */
  void connectToRobot(const std::string &remoteIP, const std::string &localIP = "");
  void disconnectFromRobot(error_code &ec) noexcept;

  Info robotInfo(error_code &ec) const noexcept;
  PowerState powerState(error_code &ec) const noexcept;
  void setPowerState(bool on, error_code &ec) noexcept;
  OperateMode operateMode(error_code &ec) const noexcept;
  void setOperateMode(OperateMode mode, error_code &ec) noexcept;
  OperationState operationState(error_code &ec) const noexcept;
  std::array<double, 6> jointPos(error_code &ec) const noexcept;
  std::array<double, 6> jointVel(error_code &ec) const noexcept;
  std::array<double, 6> jointTorques(error_code &ec) const noexcept;
  [[deprecated("Use jointTorques() instead")]]
  std::array<double, 6> jointTorque(error_code &ec) const noexcept;
  std::array<double, 6> posture(CoordinateType ct, error_code &ec) const noexcept;
  CartesianPosition cartPosture(CoordinateType ct, error_code &ec) const noexcept;
  std::array<double, 6> baseFrame(error_code &ec) const noexcept;
  Toolset toolset(error_code &ec) const noexcept;
  void setToolset(const Toolset &toolset, error_code &ec) noexcept;
  void setToolset(const std::string &toolName, const std::string &wobjName, error_code &ec) noexcept;
  void clearServoAlarm(error_code &ec) noexcept;
  std::vector<LogInfo> queryControllerLog(unsigned count,
                                          const std::set<LogInfo::Level> &level,
                                          error_code &ec) const noexcept;

  void setMotionControlMode(MotionControlMode mode, error_code &ec) noexcept;
  void moveReset(error_code &ec) noexcept;
  void stop(error_code &ec) noexcept;
  void moveStart(error_code &ec) noexcept;
  void setDefaultSpeed(int speed, error_code &ec) noexcept;
  void setDefaultZone(int zone, error_code &ec) noexcept;
  void setDefaultConfOpt(bool forced, error_code &ec) noexcept;
  void adjustSpeedOnline(double scale, error_code &ec) noexcept;
  void setEventWatcher(Event eventType, const EventCallback &callback, error_code &ec) noexcept;
  EventInfo queryEventInfo(Event eventType, error_code &ec) noexcept;
  void setMaxCacheSize(int number, error_code &ec) noexcept;
  std::error_code lastErrorCode() noexcept;

  void moveAppend(const std::vector<MoveAbsJCommand> &cmds, std::string &cmdID, error_code &ec) noexcept;
  void moveAppend(const std::vector<MoveJCommand> &cmds, std::string &cmdID, error_code &ec) noexcept;
  void moveAppend(const std::vector<MoveLCommand> &cmds, std::string &cmdID, error_code &ec) noexcept;
  void moveAppend(const std::vector<MoveCCommand> &cmds, std::string &cmdID, error_code &ec) noexcept;
  void moveAppend(const std::vector<MoveCFCommand> &cmds, std::string &cmdID, error_code &ec) noexcept;
  void moveAppend(const std::vector<MoveSPCommand> &cmds, std::string &cmdID, error_code &ec) noexcept;
  void executeCommand(const std::vector<MoveAbsJCommand> &cmds, error_code &ec) noexcept;
  void executeCommand(const std::vector<MoveJCommand> &cmds, error_code &ec) noexcept;
  void executeCommand(const std::vector<MoveLCommand> &cmds, error_code &ec) noexcept;
  /**
   * @brief Execute Cartesian arc commands immediately using the non-realtime motion queue path.
   * @param cmds One or more MoveC commands. The vector must be non-empty and homogeneously typed.
   * @param ec Output error code. Set when the backend rejects the request or motion cannot start.
   * @throws No exception.
   * @note This preserves the official SDK surface for direct `MoveC` execution on the xMate6 compatibility lane.
   */
  void executeCommand(const std::vector<MoveCCommand> &cmds, error_code &ec) noexcept;

  template <class Command>
  void moveAppend(std::initializer_list<Command> cmds, std::string &cmdID, error_code &ec) noexcept {
    moveAppend(std::vector<Command>(cmds.begin(), cmds.end()), cmdID, ec);
  }

  template <class Command>
  void executeCommand(std::initializer_list<Command> cmds, error_code &ec) noexcept {
    executeCommand(std::vector<Command>(cmds.begin(), cmds.end()), ec);
  }

/**
   * @brief Read a digital input on the historical SDK surface.
   * @details The install-facing xMate6 compatibility lane intentionally excludes IO, typed registers, RL,
   *          and calibration workflows. The symbol is preserved for source compatibility, but the public lane
   *          returns deterministic `not_implemented` instead of advertising IO as a supported contract.
   * @param board IO board index.
   * @param port Port index.
   * @param ec Output error code.
   * @throws No exception.
   * @note Boundary behavior: always returns `false` and sets `SdkError::not_implemented` on the public lane.
   */
  bool getDI(unsigned board, unsigned port, error_code &ec) noexcept;
  bool getDO(unsigned board, unsigned port, error_code &ec) noexcept;
  void setDI(unsigned board, unsigned port, bool state, error_code &ec) noexcept;
  void setDO(unsigned board, unsigned port, bool state, error_code &ec) noexcept;
  double getAI(unsigned board, unsigned port, error_code &ec) noexcept;
  void setAO(unsigned board, unsigned port, double value, error_code &ec) noexcept;
  void setSimulationMode(bool state, error_code &ec) noexcept;
  void setxPanelVout(xPanelOpt::Vout opt, error_code &ec) noexcept;

  std::string readRegisterRaw(const std::string &name, unsigned index, error_code &ec) noexcept;
  void writeRegisterRaw(const std::string &name, unsigned index, const std::string &value, error_code &ec) noexcept;

  template <typename T>
  void readRegister(const std::string &name, unsigned index, T &value, error_code &ec) noexcept;

  template <typename T>
  void writeRegister(const std::string &name, unsigned index, T value, error_code &ec) noexcept;

  std::vector<RLProjectInfo> projectsInfo(error_code &ec) noexcept;
  void loadProject(const std::string &name, const std::vector<std::string> &tasks, error_code &ec) noexcept;
  void ppToMain(error_code &ec) noexcept;
  void runProject(error_code &ec) noexcept;
  void pauseProject(error_code &ec) noexcept;
  void setProjectRunningOpt(double rate, bool loop, error_code &ec) noexcept;
  std::vector<WorkToolInfo> toolsInfo(error_code &ec) noexcept;
  std::vector<WorkToolInfo> wobjsInfo(error_code &ec) noexcept;

  void startReceiveRobotState(std::chrono::steady_clock::duration interval,
                              const std::vector<std::string> &fields);
  void stopReceiveRobotState() noexcept;
  unsigned updateRobotState(std::chrono::steady_clock::duration timeout);
  int getStateDataArray6(const std::string &fieldName, std::array<double, 6> &data) noexcept;
  int getStateDataArray3(const std::string &fieldName, std::array<double, 3> &data) noexcept;
  int getStateDataMatrix16(const std::string &fieldName, std::array<double, 16> &data) noexcept;
  int getStateDataScalarDouble(const std::string &fieldName, double &data) noexcept;
  int getStateDataBool(const std::string &fieldName, bool &data) noexcept;

  template <typename R>
  int getStateData(const std::string &fieldName, R &data) noexcept {
    if constexpr (std::is_same_v<R, std::array<double, 6>>) {
      return getStateDataArray6(fieldName, data);
    } else if constexpr (std::is_same_v<R, std::array<double, 3>>) {
      return getStateDataArray3(fieldName, data);
    } else if constexpr (std::is_same_v<R, std::array<double, 16>>) {
      return getStateDataMatrix16(fieldName, data);
    } else if constexpr (std::is_same_v<R, double>) {
      return getStateDataScalarDouble(fieldName, data);
    } else if constexpr (std::is_same_v<R, bool>) {
      return getStateDataBool(fieldName, data);
    }
    return -1;
  }

  /**
   * @brief Read the current soft-limit configuration using the official SDK array type.
   * @param limits Output joint limit pairs `[lower, upper]` in radians.
   * @param ec Output error code.
   * @return `true` when soft limits are enabled; `false` when disabled or on error.
   * @throws No exception.
   * @note This overload is the install-facing compatibility contract. The nested-array overload is retained
   *       only to avoid breaking existing in-tree call sites.
   */
  bool getSoftLimit(std::array<double[2], 6> &limits, error_code &ec) noexcept;
  void setSoftLimit(bool enable,
                    error_code &ec,
                    const std::array<double[2], 6> &limits = {{DBL_MAX, DBL_MAX}}) noexcept;
  bool getSoftLimit(std::array<std::array<double, 2>, 6> &limits, error_code &ec) noexcept;
  void setSoftLimit(bool enable,
                    error_code &ec,
                    const std::array<std::array<double, 2>, 6> &limits) noexcept;
  // NOTE: frame calibration is not part of the install-facing xMate6 public lane and returns
  // deterministic not_implemented errors when invoked through the public SDK facade.
  FrameCalibrationResult calibrateFrame(FrameType type,
                                        const std::vector<std::array<double, 6>> &points,
                                        bool is_held,
                                        error_code &ec,
                                        const std::array<double, 3> &base_aux = {}) noexcept;

 protected:
  explicit BaseRobot(std::shared_ptr<detail::CompatRobotHandle> handle);
  std::shared_ptr<detail::CompatRobotHandle> handle_;
};

class XCORE_API BaseCobot : public BaseRobot {
 public:
  using BaseRobot::BaseRobot;
  virtual ~BaseCobot();

  void enableCollisionDetection(const std::array<double, 6> &sensitivity,
                                StopLevel behaviour,
                                double fallback_compliance,
                                error_code &ec) noexcept;
  void disableCollisionDetection(error_code &ec) noexcept;
  void enableDrag(DragParameter::Space space, DragParameter::Type type, error_code &ec) noexcept;
  void disableDrag(error_code &ec) noexcept;
  void startJog(JogOpt::Space space,
                double rate,
                double step,
                unsigned index,
                bool direction,
                error_code &ec) noexcept;
  void setAvoidSingularity(bool enable, error_code &ec) noexcept;
  bool getAvoidSingularity(error_code &ec) noexcept;
  void getEndTorque(FrameType ref_type,
                    std::array<double, 6> &joint_torque_measured,
                    std::array<double, 6> &external_torque_measured,
                    std::array<double, 3> &cart_torque,
                    std::array<double, 3> &cart_force,
                    error_code &ec) noexcept;
  void startRecordPath(std::chrono::seconds duration, error_code &ec) noexcept;
  void startRecordPath(int duration_seconds, error_code &ec) noexcept;
  void stopRecordPath(error_code &ec) noexcept;
  void cancelRecordPath(error_code &ec) noexcept;
  void saveRecordPath(const std::string &name,
                      error_code &ec,
                      const std::string &saveAs = "") noexcept;
  void replayPath(const std::string &name, double rate, error_code &ec) noexcept;
  void removePath(const std::string &name,
                  error_code &ec,
                  bool removeAll = false) noexcept;
  std::vector<std::string> queryPathLists(error_code &ec) noexcept;

/** @brief Return the xMate6 model facade bound to the current tool/load context. */
  xMateModel<6> model() const;
  /**
   * @brief Get the shared RT controller facade for the xMate6 compatibility lane.
   * @return Weak reference to the cached RT controller object.
   * @throws No exception.
   * @note The weak handle follows the official SDK ownership shape: the facade is cached on the robot object
   *       and may outlive individual mode switches, but later RT calls can still fail if the robot lifecycle no
   *       longer permits RT control.
   */
  std::weak_ptr<RtMotionControlCobot<6>> getRtMotionController();
  void setRtNetworkTolerance(unsigned percent, error_code &ec) noexcept;
  void useRciClient(bool use, error_code &ec) noexcept;
};

template <unsigned short DoF>
class Cobot;

template <>
class XCORE_API Cobot<6> : public BaseCobot {
 public:
  Cobot();
  explicit Cobot(const std::string &remoteIP, const std::string &localIP = "");
  ~Cobot() override;
};

template <>
class XCORE_API Robot_T<WorkType::collaborative, 6> : public Cobot<6> {
 public:
  Robot_T();
  explicit Robot_T(const std::string &remoteIP, const std::string &localIP = "");
  ~Robot_T() override;
};

class XCORE_API xMateRobot : public Robot_T<WorkType::collaborative, 6> {
 public:
  xMateRobot();
  explicit xMateRobot(const std::string &remoteIP, const std::string &localIP = "");
  ~xMateRobot() override;
};


template <typename T>
inline void BaseRobot::readRegister(const std::string &name, unsigned index, T &value, error_code &ec) noexcept {
  const std::string payload = readRegisterRaw(name, index, ec);
  if (ec) {
    return;
  }
  if constexpr (std::is_same_v<T, bool>) {
    if (payload == "1" || payload == "true" || payload == "TRUE") {
      value = true;
      ec.clear();
      return;
    }
    if (payload == "0" || payload == "false" || payload == "FALSE") {
      value = false;
      ec.clear();
      return;
    }
    ec = std::make_error_code(std::errc::invalid_argument);
  } else if constexpr (std::is_integral_v<T>) {
    try {
      value = static_cast<T>(std::stoll(payload));
      ec.clear();
    } catch (...) {
      ec = std::make_error_code(std::errc::invalid_argument);
    }
  } else if constexpr (std::is_floating_point_v<T>) {
    try {
      value = static_cast<T>(std::stod(payload));
      ec.clear();
    } catch (...) {
      ec = std::make_error_code(std::errc::invalid_argument);
    }
  } else if constexpr (std::is_same_v<T, std::vector<int>> || std::is_same_v<T, std::vector<float>> || std::is_same_v<T, std::vector<double>>) {
    value.clear();
    if (payload.empty()) {
      ec.clear();
      return;
    }
    std::size_t start_pos = 0;
    while (start_pos <= payload.size()) {
      const auto sep = payload.find(',', start_pos);
      const std::string token = payload.substr(start_pos, sep == std::string::npos ? std::string::npos : sep - start_pos);
      try {
        if constexpr (std::is_same_v<T, std::vector<int>>) {
          value.push_back(static_cast<int>(std::stoll(token)));
        } else if constexpr (std::is_same_v<T, std::vector<float>>) {
          value.push_back(static_cast<float>(std::stod(token)));
        } else {
          value.push_back(std::stod(token));
        }
      } catch (...) {
        ec = std::make_error_code(std::errc::invalid_argument);
        value.clear();
        return;
      }
      if (sep == std::string::npos) {
        break;
      }
      start_pos = sep + 1;
    }
    ec.clear();
  } else {
    ec = std::make_error_code(std::errc::invalid_argument);
  }
}

template <typename T>
inline void BaseRobot::writeRegister(const std::string &name, unsigned index, T value, error_code &ec) noexcept {
  std::string payload;
  if constexpr (std::is_same_v<T, bool>) {
    payload = value ? "1" : "0";
  } else if constexpr (std::is_integral_v<T>) {
    payload = std::to_string(value);
  } else if constexpr (std::is_floating_point_v<T>) {
    payload = std::to_string(static_cast<double>(value));
  } else {
    ec = std::make_error_code(std::errc::invalid_argument);
    return;
  }
  writeRegisterRaw(name, index, payload, ec);
}

}  // namespace rokae

#endif  // ROKAE_ROBOT_H
