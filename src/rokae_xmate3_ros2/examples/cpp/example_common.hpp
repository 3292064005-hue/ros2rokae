#pragma once

#include <algorithm>
#include <any>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <type_traits>
#include <vector>

#include "rokae/robot.h"

namespace example {

using namespace rokae;
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr auto kRtControlPeriod = std::chrono::milliseconds(1);
inline constexpr double kRtControlDtSec = 0.001;
inline constexpr std::array<double, 6> kXMate3DragPose{0.0, kPi / 6.0, kPi / 3.0, 0.0, kPi / 2.0, 0.0};
inline std::ostream &os = std::cout;

inline void printHeader(const std::string &title, const std::string &subtitle = {}) {
  os << "==========================================\n";
  os << "  " << title << '\n';
  if (!subtitle.empty()) {
    os << "  (" << subtitle << ")\n";
  }
  os << "==========================================\n";
}

inline void printSection(const std::string &title) {
  os << "\n[" << title << "]\n";
}

inline const char *toString(PowerState state) {
  switch (state) {
    case PowerState::on: return "on";
    case PowerState::off: return "off";
    case PowerState::estop: return "estop";
    case PowerState::gstop: return "gstop";
    default: return "unknown";
  }
}

inline const char *toString(OperateMode mode) {
  switch (mode) {
    case OperateMode::manual: return "manual";
    case OperateMode::automatic: return "automatic";
    default: return "unknown";
  }
}

inline const char *toString(OperationState state) {
  switch (state) {
    case OperationState::idle: return "idle";
    case OperationState::moving: return "moving";
    case OperationState::jog: return "jog";
    case OperationState::drag: return "drag";
    default: return "unknown";
  }
}

inline std::string toLowerCopy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

inline bool reportError(const std::string &action, const error_code &ec) {
  if (!ec) {
    return false;
  }
  std::cerr << "! " << action << " failed: " << ec.message() << std::endl;
  return true;
}

inline bool checkLastRobotError(BaseRobot &robot, const std::string &action, error_code &ec) {
  const auto last_error = robot.lastErrorCode();
  if (!last_error) {
    ec.clear();
    return true;
  }
  ec = last_error;
  return !reportError(action, ec);
}

inline void printCapabilityStatus(const std::string &kind, const std::string &detail) {
  os << "[" << kind << "] " << detail << std::endl;
}

inline bool isSimulationOnlyCapabilityError(const error_code &ec) {
  if (!ec) {
    return false;
  }
  const auto message = toLowerCopy(ec.message());
  static const std::array<std::string_view, 8> kNeedles{
      "not supported",
      "unsupported",
      "not available",
      "unavailable",
      "simulation-only",
      "not implemented",
      "controller unavailable",
      "action server unavailable",
  };
  return std::any_of(kNeedles.begin(), kNeedles.end(), [&](std::string_view needle) {
    return message.find(needle) != std::string::npos;
  });
}

template <typename Fn>
inline bool retryRobotAction(const std::string &action,
                             error_code &ec,
                             Fn &&fn,
                             int attempts = 5,
                             std::chrono::milliseconds initial_backoff = std::chrono::milliseconds(250)) {
  error_code last_error;
  auto backoff = initial_backoff;
  for (int attempt = 1; attempt <= attempts; ++attempt) {
    error_code attempt_error;
    fn(attempt_error);
    if (!attempt_error) {
      ec.clear();
      return true;
    }
    last_error = attempt_error;
    if (attempt < attempts) {
      printCapabilityStatus("retry", action + " attempt " + std::to_string(attempt) + "/" +
                                         std::to_string(attempts) + " -> " + attempt_error.message());
      std::this_thread::sleep_for(backoff);
      backoff += initial_backoff;
    }
  }
  ec = last_error;
  return !reportError(action, ec);
}

template <typename T, size_t N>
inline void printArray(const std::string &label,
                       const std::array<T, N> &data,
                       int precision = 4,
                       const char *suffix = "") {
  os << label << ": [";
  for (size_t i = 0; i < N; ++i) {
    if constexpr (std::is_floating_point_v<T>) {
      os << std::fixed << std::setprecision(precision) << data[i];
    } else {
      os << data[i];
    }
    if (i + 1 < N) {
      os << ", ";
    }
  }
  os << "]" << suffix << std::endl;
}

template <typename T>
inline void printVector(const std::string &label,
                        const std::vector<T> &data,
                        int precision = 4,
                        const char *suffix = "") {
  os << label << ": [";
  for (size_t i = 0; i < data.size(); ++i) {
    if constexpr (std::is_floating_point_v<T>) {
      os << std::fixed << std::setprecision(precision) << data[i];
    } else {
      os << data[i];
    }
    if (i + 1 < data.size()) {
      os << ", ";
    }
  }
  os << "]" << suffix << std::endl;
}

inline void printPose(const std::string &label, const CartesianPosition &pose) {
  os << label << ": ["
     << std::fixed << std::setprecision(4)
     << pose.x << ", " << pose.y << ", " << pose.z << ", "
     << pose.rx << ", " << pose.ry << ", " << pose.rz << "]" << std::endl;
}

inline void printToolset(const Toolset &toolset) {
  os << "tool: " << toolset.tool_name << ", wobj: " << toolset.wobj_name << std::endl;
  printArray("tool pose", toolset.tool_pose);
  printArray("wobj pose", toolset.wobj_pose);
}

inline void printMoveEvent(const EventInfo &info) {
  using namespace EventInfoKey::MoveExecution;
  try {
    const auto id = std::any_cast<std::string>(info.at(ID));
    const auto index = std::any_cast<int>(info.at(WaypointIndex));
    const auto reach = std::any_cast<bool>(info.at(ReachTarget));
    const auto ec = std::any_cast<error_code>(info.at(Error));
    const auto remark = std::any_cast<std::string>(info.at(Remark));
    os << "[moveExecution] id=" << id
       << " index=" << index
       << " reachTarget=" << (reach ? "YES" : "NO")
       << " error=" << (ec ? ec.message() : "ok")
       << " remark=" << remark << std::endl;
  } catch (const std::exception &) {
    os << "[moveExecution] event updated" << std::endl;
  }
}

inline const char *toString(LogInfo::Level level) {
  switch (level) {
    case LogInfo::Level::debug: return "debug";
    case LogInfo::Level::info: return "info";
    case LogInfo::Level::warning: return "warning";
    case LogInfo::Level::error: return "error";
    case LogInfo::Level::fatal: return "fatal";
    default: return "unknown";
  }
}

inline void printLogInfo(const LogInfo &log) {
  os << "[" << toString(static_cast<LogInfo::Level>(log.level)) << "] "
     << log.timestamp << " :: " << log.content;
  if (!log.repair.empty()) {
    os << " | repair: " << log.repair;
  }
  os << std::endl;
}

inline void printSafetyEvent(const EventInfo &info) {
  try {
    const auto collided = std::any_cast<bool>(info.at(EventInfoKey::Safety::Collided));
    os << "[safety] collided=" << (collided ? "YES" : "NO") << std::endl;
  } catch (const std::exception &) {
    os << "[safety] event updated" << std::endl;
  }
}

inline bool waitRobot(BaseRobot &robot,
                      error_code &ec,
                      std::chrono::steady_clock::duration timeout = std::chrono::seconds(60)) {
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout) {
    const auto state = robot.operationState(ec);
    if (ec) {
      return false;
    }
    if (state == OperationState::idle || state == OperationState::unknown) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ec = std::make_error_code(std::errc::timed_out);
  return false;
}

inline bool waitMotionCycle(BaseRobot &robot,
                            error_code &ec,
                            std::chrono::steady_clock::duration timeout = std::chrono::seconds(60)) {
  const auto start = std::chrono::steady_clock::now();
  bool seen_active_state = false;
  while (std::chrono::steady_clock::now() - start < timeout) {
    const auto state = robot.operationState(ec);
    if (ec) {
      return false;
    }
    if (state != OperationState::idle && state != OperationState::unknown) {
      seen_active_state = true;
    }
    if (seen_active_state && (state == OperationState::idle || state == OperationState::unknown)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ec = std::make_error_code(std::errc::timed_out);
  return false;
}

inline bool waitForCommandResult(BaseRobot &robot,
                                 const std::string &action,
                                 error_code &ec,
                                 std::chrono::steady_clock::duration timeout = std::chrono::seconds(90),
                                 std::chrono::steady_clock::duration activation_window = std::chrono::seconds(2)) {
  const auto start = std::chrono::steady_clock::now();
  bool seen_active_state = false;
  while (std::chrono::steady_clock::now() - start < timeout) {
    const auto state = robot.operationState(ec);
    if (ec) {
      return false;
    }
    if (state != OperationState::idle && state != OperationState::unknown) {
      seen_active_state = true;
    }
    if (seen_active_state && (state == OperationState::idle || state == OperationState::unknown)) {
      return checkLastRobotError(robot, action, ec);
    }
    if (!seen_active_state &&
        std::chrono::steady_clock::now() - start >= activation_window &&
        (state == OperationState::idle || state == OperationState::unknown)) {
      return checkLastRobotError(robot, action, ec);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ec = std::make_error_code(std::errc::timed_out);
  return !reportError(action + " wait", ec);
}

inline bool waitForFinish(BaseRobot &robot,
                          const std::string &traj_id,
                          int index,
                          error_code &ec,
                          std::chrono::steady_clock::duration timeout = std::chrono::seconds(90)) {
  using namespace EventInfoKey::MoveExecution;
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout) {
    const auto info = robot.queryEventInfo(Event::moveExecution, ec);
    if (ec) {
      return false;
    }
    try {
      const auto current_id = std::any_cast<std::string>(info.at(ID));
      const auto current_index = std::any_cast<int>(info.at(WaypointIndex));
      const auto move_ec = std::any_cast<error_code>(info.at(Error));
      if (move_ec) {
        ec = move_ec;
        return false;
      }
      if (current_id == traj_id && current_index == index && std::any_cast<bool>(info.at(ReachTarget))) {
        return true;
      }
    } catch (const std::exception &) {
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ec = std::make_error_code(std::errc::timed_out);
  return false;
}

inline bool waitForCommandOrIdle(BaseRobot &robot,
                                 const std::string &traj_id,
                                 int index,
                                 error_code &ec,
                                 std::chrono::steady_clock::duration timeout = std::chrono::seconds(90)) {
  using namespace EventInfoKey::MoveExecution;
  const auto start = std::chrono::steady_clock::now();
  bool seen_active_state = false;
  while (std::chrono::steady_clock::now() - start < timeout) {
    const auto info = robot.queryEventInfo(Event::moveExecution, ec);
    if (ec) {
      return false;
    }
    try {
      const auto current_id = std::any_cast<std::string>(info.at(ID));
      const auto current_index = std::any_cast<int>(info.at(WaypointIndex));
      const auto move_ec = std::any_cast<error_code>(info.at(Error));
      if (move_ec) {
        ec = move_ec;
        return false;
      }
      if (current_id == traj_id && current_index >= index && std::any_cast<bool>(info.at(ReachTarget))) {
        return true;
      }
    } catch (const std::exception &) {
    }

    const auto state = robot.operationState(ec);
    if (ec) {
      return false;
    }
    if (state != OperationState::idle && state != OperationState::unknown) {
      seen_active_state = true;
    }
    if (seen_active_state && (state == OperationState::idle || state == OperationState::unknown)) {
      return true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ec = std::make_error_code(std::errc::timed_out);
  return false;
}

inline bool connectRobot(xMateRobot &robot, error_code &ec) {
  const char *remote_env = std::getenv("ROKAE_REMOTE_IP");
  const char *local_env = std::getenv("ROKAE_LOCAL_IP");
  const std::string remote_ip = (remote_env != nullptr && *remote_env != '\0') ? remote_env : "127.0.0.1";
  const std::string local_ip = (local_env != nullptr && *local_env != '\0') ? local_env : "127.0.0.1";
  return retryRobotAction("connectToRobot", ec, [&](error_code &attempt_error) {
    robot.connectToRobot(remote_ip, local_ip, attempt_error);
  });
}

inline bool prepareAutomaticNrt(xMateRobot &robot,
                                error_code &ec,
                                int speed = 20,
                                int zone = 5) {
  auto resetSimulationContext = [&](const char *label) {
    error_code reset_error;
    robot.disableCollisionDetection(reset_error);
    reset_error.clear();
    robot.setAvoidSingularity(false, reset_error);
    reset_error.clear();
    robot.setSimulationMode(false, reset_error);
    reset_error.clear();
    robot.setDefaultConfOpt(false, reset_error);
    reset_error.clear();
    Toolset default_toolset;
    robot.setToolset(default_toolset, reset_error);
    if (reset_error) {
      printCapabilityStatus("approximate", std::string(label) +
                                               " resetToolset(tool0/wobj0) failed: " +
                                               reset_error.message());
    }
  };

  error_code ignore_error;
  robot.stop(ignore_error);
  ignore_error.clear();
  robot.moveReset(ignore_error);

  if (!retryRobotAction("setMotionControlMode", ec, [&](error_code &attempt_error) {
        robot.setMotionControlMode(MotionControlMode::NrtCommand, attempt_error);
      })) {
    return false;
  }
  if (!retryRobotAction("setOperateMode", ec, [&](error_code &attempt_error) {
        robot.setOperateMode(OperateMode::automatic, attempt_error);
      })) {
    return false;
  }
  if (!retryRobotAction("setPowerState", ec, [&](error_code &attempt_error) {
        robot.setPowerState(true, attempt_error);
      })) {
    return false;
  }
  resetSimulationContext("prepareAutomaticNrt");
  if (!retryRobotAction("setDefaultSpeed", ec, [&](error_code &attempt_error) {
        robot.setDefaultSpeed(speed, attempt_error);
      })) {
    return false;
  }
  return retryRobotAction("setDefaultZone", ec, [&](error_code &attempt_error) {
    robot.setDefaultZone(zone, attempt_error);
  });
}

inline bool prepareAutomaticRt(xMateRobot &robot,
                               error_code &ec,
                               unsigned tolerance = 20) {
  auto resetSimulationContext = [&](const char *label) {
    error_code reset_error;
    robot.disableCollisionDetection(reset_error);
    reset_error.clear();
    robot.setAvoidSingularity(false, reset_error);
    reset_error.clear();
    robot.setSimulationMode(false, reset_error);
    reset_error.clear();
    robot.setDefaultConfOpt(false, reset_error);
    reset_error.clear();
    Toolset default_toolset;
    robot.setToolset(default_toolset, reset_error);
    if (reset_error) {
      printCapabilityStatus("approximate", std::string(label) +
                                               " resetToolset(tool0/wobj0) failed: " +
                                               reset_error.message());
    }
  };

  error_code ignore_error;
  robot.stop(ignore_error);
  ignore_error.clear();
  robot.moveReset(ignore_error);

  if (!retryRobotAction("setRtNetworkTolerance", ec, [&](error_code &attempt_error) {
        robot.setRtNetworkTolerance(tolerance, attempt_error);
      }, 3, std::chrono::milliseconds(150))) {
    return false;
  }
  if (!retryRobotAction("setMotionControlMode", ec, [&](error_code &attempt_error) {
        robot.setMotionControlMode(MotionControlMode::RtCommand, attempt_error);
      }, 3, std::chrono::milliseconds(200))) {
    return false;
  }
  if (!retryRobotAction("setOperateMode", ec, [&](error_code &attempt_error) {
        robot.setOperateMode(OperateMode::automatic, attempt_error);
      })) {
    return false;
  }
  resetSimulationContext("prepareAutomaticRt");
  return retryRobotAction("setPowerState", ec, [&](error_code &attempt_error) {
    robot.setPowerState(true, attempt_error);
  });
}

template <typename T>
inline bool ensurePtr(const std::shared_ptr<T> &ptr, const std::string &name) {
  if (ptr) {
    return true;
  }
  std::cerr << "! " << name << " unavailable" << std::endl;
  return false;
}

inline void cleanupRobot(xMateRobot &robot);

inline int skipExample(xMateRobot &robot, const std::string &detail) {
  printCapabilityStatus("skipped", detail);
  cleanupRobot(robot);
  return 0;
}

inline int skipExample(const std::string &detail) {
  printCapabilityStatus("skipped", detail);
  return 0;
}

inline void cleanupRobot(xMateRobot &robot) {
  error_code ec;
  robot.stop(ec);
  ec.clear();
  robot.moveReset(ec);
  ec.clear();
  robot.setPowerState(false, ec);
  ec.clear();
  robot.disconnectFromRobot(ec);
}

}  // namespace example
