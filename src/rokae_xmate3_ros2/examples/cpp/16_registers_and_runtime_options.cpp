/**
 * @file 16_registers_and_runtime_options.cpp
 * @brief 寄存器、xPanel 与运行时选项示例
 */

#include <vector>

#include "rokae/robot.h"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 16: 寄存器与运行时选项", "readRegister / writeRegister / xPanel");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }

  printSection("1 标量寄存器");
  bool enable_flag = true;
  int counter = 7;
  float gain = 0.75F;
  robot.writeRegister("demo_flag", 0, enable_flag, ec);
  if (reportError("writeRegister(demo_flag)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.writeRegister("demo_counter", 0, counter, ec);
  if (reportError("writeRegister(demo_counter)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.writeRegister("demo_gain", 0, gain, ec);
  if (reportError("writeRegister(demo_gain)", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  enable_flag = false;
  counter = 0;
  gain = 0.0F;
  robot.readRegister("demo_flag", 0, enable_flag, ec);
  if (reportError("readRegister(demo_flag)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.readRegister("demo_counter", 0, counter, ec);
  if (reportError("readRegister(demo_counter)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.readRegister("demo_gain", 0, gain, ec);
  if (reportError("readRegister(demo_gain)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "demo_flag[0] = " << std::boolalpha << enable_flag << std::noboolalpha << std::endl;
  os << "demo_counter[0] = " << counter << std::endl;
  os << "demo_gain[0] = " << gain << std::endl;

  printSection("2 向量寄存器");
  std::vector<int> steps{2, 4, 6, 8};
  std::vector<float> blend{0.10F, 0.25F, 0.50F};
  robot.writeRegister("demo_steps", 0, steps, ec);
  if (reportError("writeRegister(demo_steps)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.writeRegister("demo_blend", 0, blend, ec);
  if (reportError("writeRegister(demo_blend)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  steps.clear();
  blend.clear();
  robot.readRegister("demo_steps", 0, steps, ec);
  if (reportError("readRegister(demo_steps)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.readRegister("demo_blend", 0, blend, ec);
  if (reportError("readRegister(demo_blend)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printVector("demo_steps", steps, 0);
  printVector("demo_blend", blend, 3);

  printSection("3 xPanel 与运行时开关");
  robot.setxPanelVout(xPanelOpt::supply24v, ec);
  if (reportError("setxPanelVout(24V)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.setRtNetworkTolerance(25, ec);
  if (reportError("setRtNetworkTolerance", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.useRciClient(true, ec);
  if (reportError("useRciClient(true)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.useRciClient(false, ec);
  if (reportError("useRciClient(false)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "xPanel set to 24V, rt tolerance=25%, rci toggled" << std::endl;

  printSection("4 最近错误码与断开连接");
  const auto last_error = robot.lastErrorCode();
  os << "last error: " << (last_error ? last_error.message() : "ok") << std::endl;
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
