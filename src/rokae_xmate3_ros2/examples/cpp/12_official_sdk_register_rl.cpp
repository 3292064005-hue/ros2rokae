#include <iostream>
#include <vector>

#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  error_code ec;
  robot.connectToRobot(ec);

  robot.writeRegister("register0", 0, true, ec);
  robot.writeRegister("register1", 0, 42, ec);
  robot.writeRegister("register2", 0, 3.14f, ec);
  robot.writeRegister("register3", 0, std::vector<int>{1, 2, 3}, ec);

  bool b = false;
  int i = 0;
  float f = 0.0f;
  std::vector<int> vec;
  robot.readRegister("register0", 0, b, ec);
  robot.readRegister("register1", 0, i, ec);
  robot.readRegister("register2", 0, f, ec);
  robot.readRegister("register3", 0, vec, ec);

  robot.loadProject("demo_project", {"main_task"}, ec);
  robot.ppToMain(ec);
  robot.setProjectRunningOpt(0.5, true, ec);
  auto projects = robot.projectsInfo(ec);
  auto tools = robot.toolsInfo(ec);
  auto wobjs = robot.wobjsInfo(ec);
  robot.runProject(ec);
  robot.pauseProject(ec);

  std::cout << "Registers: " << b << ", " << i << ", " << f << ", vec_size=" << vec.size() << "\n";
  std::cout << "Projects=" << projects.size() << ", tools=" << tools.size() << ", wobjs=" << wobjs.size() << "\n";
  return 0;
}
