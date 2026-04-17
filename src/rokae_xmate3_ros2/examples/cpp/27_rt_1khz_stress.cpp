/**
 * @file 27_rt_1khz_stress.cpp
 * @brief xMate6 RT 1kHz stress runner (official SDK calling style).
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "rokae/motion_control_rt.h"
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

namespace {

struct StressConfig {
  double duration_sec = 600.0;
  double warmup_sec = 5.0;
  double amplitude_rad = 0.05;
  double frequency_hz = 0.25;
};

struct StressStats {
  std::size_t samples = 0;
  double avg_ms = 0.0;
  double p95_ms = 0.0;
  double p99_ms = 0.0;
  double min_ms = 0.0;
  double max_ms = 0.0;
  double avg_hz = 0.0;
};

double clampPositive(double value, double fallback) {
  if (!std::isfinite(value) || value <= 0.0) {
    return fallback;
  }
  return value;
}

bool parseDouble(const char *raw, double &out) {
  if (raw == nullptr) {
    return false;
  }
  try {
    out = std::stod(raw);
    return true;
  } catch (...) {
    return false;
  }
}

StressConfig parseArgs(int argc, char **argv) {
  StressConfig cfg;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == "--duration" || arg == "-d") && i + 1 < argc) {
      double parsed = cfg.duration_sec;
      if (parseDouble(argv[++i], parsed)) {
        cfg.duration_sec = clampPositive(parsed, cfg.duration_sec);
      }
      continue;
    }
    if ((arg == "--warmup" || arg == "-w") && i + 1 < argc) {
      double parsed = cfg.warmup_sec;
      if (parseDouble(argv[++i], parsed)) {
        cfg.warmup_sec = std::max(0.0, parsed);
      }
      continue;
    }
    if ((arg == "--amplitude" || arg == "-a") && i + 1 < argc) {
      double parsed = cfg.amplitude_rad;
      if (parseDouble(argv[++i], parsed)) {
        cfg.amplitude_rad = std::max(0.001, std::fabs(parsed));
      }
      continue;
    }
    if ((arg == "--frequency" || arg == "-f") && i + 1 < argc) {
      double parsed = cfg.frequency_hz;
      if (parseDouble(argv[++i], parsed)) {
        cfg.frequency_hz = clampPositive(parsed, cfg.frequency_hz);
      }
      continue;
    }
  }
  return cfg;
}

double percentileMs(std::vector<double> values, double q) {
  if (values.empty()) {
    return 0.0;
  }
  q = std::clamp(q, 0.0, 1.0);
  std::sort(values.begin(), values.end());
  const double idx = q * static_cast<double>(values.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(idx));
  const auto hi = static_cast<std::size_t>(std::ceil(idx));
  if (lo == hi) {
    return values[lo];
  }
  const double mix = idx - static_cast<double>(lo);
  return values[lo] * (1.0 - mix) + values[hi] * mix;
}

StressStats computeStats(const std::vector<double> &samples_ms) {
  StressStats stats;
  stats.samples = samples_ms.size();
  if (samples_ms.empty()) {
    return stats;
  }
  double sum = 0.0;
  stats.min_ms = samples_ms.front();
  stats.max_ms = samples_ms.front();
  for (const double sample : samples_ms) {
    sum += sample;
    stats.min_ms = std::min(stats.min_ms, sample);
    stats.max_ms = std::max(stats.max_ms, sample);
  }
  stats.avg_ms = sum / static_cast<double>(samples_ms.size());
  stats.p95_ms = percentileMs(samples_ms, 0.95);
  stats.p99_ms = percentileMs(samples_ms, 0.99);
  if (stats.avg_ms > 1e-9) {
    stats.avg_hz = 1000.0 / stats.avg_ms;
  }
  return stats;
}

std::string toJson(const StressConfig &cfg, const StressStats &stats) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);
  oss << "{";
  oss << "\"duration_sec\":" << cfg.duration_sec << ",";
  oss << "\"warmup_sec\":" << cfg.warmup_sec << ",";
  oss << "\"amplitude_rad\":" << cfg.amplitude_rad << ",";
  oss << "\"frequency_hz\":" << cfg.frequency_hz << ",";
  oss << "\"samples\":" << stats.samples << ",";
  oss << "\"avg_ms\":" << stats.avg_ms << ",";
  oss << "\"p95_ms\":" << stats.p95_ms << ",";
  oss << "\"p99_ms\":" << stats.p99_ms << ",";
  oss << "\"min_ms\":" << stats.min_ms << ",";
  oss << "\"max_ms\":" << stats.max_ms << ",";
  oss << "\"avg_hz\":" << stats.avg_hz;
  oss << "}";
  return oss.str();
}

}  // namespace

int main(int argc, char **argv) {
  const auto cfg = parseArgs(argc, argv);
  printHeader("示例 27: RT 1kHz 压测", "官方 SDK 风格");
  os << "note: 当前示例运行于 ROS2/Gazebo 仿真环境，调用方式与官方 SDK 保持一致" << std::endl;
  os << "rt profile: 1kHz control period (" << kRtControlPeriod.count() << " ms)" << std::endl;
  os << "config: duration=" << cfg.duration_sec << "s warmup=" << cfg.warmup_sec
     << "s amplitude=" << cfg.amplitude_rad << "rad frequency=" << cfg.frequency_hz << "Hz" << std::endl;

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticRt(robot, ec, 20)) {
    if (isSimulationOnlyCapabilityError(ec)) {
      return skipExample(robot, "RT stress unavailable in current simulation backend: " + ec.message());
    }
    cleanupRobot(robot);
    return 1;
  }

  auto rt = robot.getRtMotionController().lock();
  if (!rt) {
    return skipExample(robot, "RT controller unavailable in current simulation backend");
  }

  printSection("1 RT 预检查");
  const auto current = robot.jointPos(ec);
  if (reportError("jointPos", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  (void)current;

  printSection("2 启动 1kHz 压测回调");
  robot.startReceiveRobotState(kRtControlPeriod, {RtSupportedFields::jointPos_m});
  std::vector<double> periods_ms;
  periods_ms.reserve(static_cast<std::size_t>(cfg.duration_sec * 1100.0) + 1024u);

  std::array<double, 6> start_joints{};
  bool initialized = false;
  using Clock = std::chrono::steady_clock;
  Clock::time_point t_start{};
  Clock::time_point t_prev{};

  rt->startMove(RtControllerMode::jointPosition);
  rt->setControlLoop(std::function<JointPosition(void)>([&]() {
    const auto now = Clock::now();
    if (!initialized) {
      robot.updateRobotState(kRtControlPeriod);
      robot.getStateData(RtSupportedFields::jointPos_m, start_joints);
      initialized = true;
      t_start = now;
      t_prev = now;
    }

    const double elapsed_sec = std::chrono::duration<double>(now - t_start).count();
    const double delta_ms = std::chrono::duration<double, std::milli>(now - t_prev).count();
    t_prev = now;
    if (elapsed_sec >= cfg.warmup_sec && std::isfinite(delta_ms) && delta_ms > 0.0) {
      periods_ms.push_back(delta_ms);
    }

    const double wave = cfg.amplitude_rad * std::sin(2.0 * kPi * cfg.frequency_hz * elapsed_sec);
    JointPosition cmd(6);
    cmd.joints = {
        start_joints[0] + wave,
        start_joints[1] + wave * 0.5,
        start_joints[2] - wave * 0.5,
        start_joints[3] + wave * 0.25,
        start_joints[4],
        start_joints[5] - wave * 0.25,
    };
    if (elapsed_sec >= cfg.duration_sec) {
      cmd.setFinished();
    }
    return cmd;
  }), 80, true);
  rt->startLoop(true);

  robot.stopReceiveRobotState();
  if (const auto loop_ec = robot.lastErrorCode(); loop_ec) {
    if (isSimulationOnlyCapabilityError(loop_ec)) {
      return skipExample(robot, "RT loop unavailable in current simulation backend: " + loop_ec.message());
    }
    reportError("rt stress loop", loop_ec);
    cleanupRobot(robot);
    return 1;
  }

  const auto stats = computeStats(periods_ms);
  os << "samples=" << stats.samples
     << " avg_hz=" << std::fixed << std::setprecision(3) << stats.avg_hz
     << " avg_ms=" << stats.avg_ms
     << " p95_ms=" << stats.p95_ms
     << " p99_ms=" << stats.p99_ms
     << " min_ms=" << stats.min_ms
     << " max_ms=" << stats.max_ms << std::endl;
  os << "RT_STRESS_JSON " << toJson(cfg, stats) << std::endl;

  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  cleanupRobot(robot);
  return stats.samples > 100 ? 0 : 1;
}
