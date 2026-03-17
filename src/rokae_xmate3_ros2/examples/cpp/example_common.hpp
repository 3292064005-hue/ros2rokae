#pragma once

#include <algorithm>
#include <any>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <set>
#include <string>
#include <system_error>
#include <thread>
#include <type_traits>
#include <vector>

#include "rokae/robot.h"

namespace example {

using namespace rokae;
inline constexpr double kPi = 3.14159265358979323846;
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

inline bool reportError(const std::string &action, const error_code &ec) {
  if (!ec) {
    return false;
  }
  std::cerr << "! " << action << " failed: " << ec.message() << std::endl;
  return true;
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
  if (waitForFinish(robot, traj_id, index, ec, timeout)) {
    return true;
  }
  if (ec != std::make_error_code(std::errc::timed_out)) {
    return false;
  }
  ec.clear();
  return waitMotionCycle(robot, ec, timeout);
}

inline bool connectRobot(xMateRobot &robot, error_code &ec) {
  robot.connectToRobot(ec);
  return !reportError("connectToRobot", ec);
}

inline bool prepareAutomaticNrt(xMateRobot &robot,
                                error_code &ec,
                                int speed = 20,
                                int zone = 5) {
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  if (reportError("setMotionControlMode", ec)) {
    return false;
  }
  robot.setOperateMode(OperateMode::automatic, ec);
  if (reportError("setOperateMode", ec)) {
    return false;
  }
  robot.setPowerState(true, ec);
  if (reportError("setPowerState", ec)) {
    return false;
  }
  robot.setDefaultSpeed(speed, ec);
  if (reportError("setDefaultSpeed", ec)) {
    return false;
  }
  robot.setDefaultZone(zone, ec);
  return !reportError("setDefaultZone", ec);
}

inline void cleanupRobot(xMateRobot &robot) {
  error_code ec;
  robot.setPowerState(false, ec);
  robot.disconnectFromRobot(ec);
}

}  // namespace example
