/**
 * @file 06_io_control.cpp
 * @brief 官方 SDK 风格 - IO 与寄存器接口
 */

#include <vector>

#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 6: IO 与寄存器控制", "官方 SDK 风格");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }

  printSection("1 打开仿真输入模式");
  robot.setSimulationMode(true, ec);
  if (reportError("setSimulationMode(true)", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("2 DI / DO 接口");
  robot.setDI(0, 2, true, ec);
  if (reportError("setDI", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "DI[0][2] = " << (robot.getDI(0, 2, ec) ? "ON" : "OFF") << std::endl;
  if (reportError("getDI", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  robot.setDO(0, 0, true, ec);
  if (reportError("setDO", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "DO[0][0] = " << (robot.getDO(0, 0, ec) ? "ON" : "OFF") << std::endl;
  if (reportError("getDO", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("3 AI / AO 接口");
  os << "AI[0][0] = " << robot.getAI(0, 0, ec) << " V" << std::endl;
  if (reportError("getAI", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.setAO(0, 0, 2.5, ec);
  if (reportError("setAO", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "AO[0][0] set to 2.5 V" << std::endl;

  printSection("4 读写 float / bool / vector 寄存器");
  float reg_f_write = 1.25F;
  float reg_f_read = 0.0F;
  robot.writeRegister("register0", 0, reg_f_write, ec);
  if (reportError("writeRegister(register0)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.readRegister("register0", 0, reg_f_read, ec);
  if (reportError("readRegister(register0)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "register0[0] = " << reg_f_read << std::endl;

  bool reg_b_write = true;
  bool reg_b_read = false;
  robot.writeRegister("register2", 0, reg_b_write, ec);
  if (reportError("writeRegister(register2)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.readRegister("register2", 0, reg_b_read, ec);
  if (reportError("readRegister(register2)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "register2[0] = " << (reg_b_read ? "true" : "false") << std::endl;

  const std::vector<int> reg_i_write{10, 20, 30, 40};
  std::vector<int> reg_i_read;
  robot.writeRegister("register1", 0, reg_i_write, ec);
  if (reportError("writeRegister(register1)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.readRegister("register1", 0, reg_i_read, ec);
  if (reportError("readRegister(register1)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printVector("register1", reg_i_read);

  const std::vector<float> reg_vec_write{0.10F, 0.20F, 0.30F};
  std::vector<float> reg_vec_read;
  robot.writeRegister("register3", 0, reg_vec_write, ec);
  if (reportError("writeRegister(register3)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.readRegister("register3", 0, reg_vec_read, ec);
  if (reportError("readRegister(register3)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printVector("register3", reg_vec_read, 4);

  printSection("5 关闭仿真输入模式并断开连接");
  robot.setSimulationMode(false, ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
