#include <iostream>
#include <set>

#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  error_code ec;

  robot.connectToRobot(ec);
  auto info = robot.robotInfo(ec);
  auto power = robot.powerState(ec);
  auto mode = robot.operateMode(ec);
  auto logs = robot.queryControllerLog(5, {rokae::LogInfo::Level::info, rokae::LogInfo::Level::warning}, ec);

  std::cout << "SDK Version: " << rokae::BaseRobot::sdkVersion() << "\n";
  std::cout << "Robot Type: " << info.type << ", joints=" << info.joint_num << "\n";
  std::cout << "Power=" << static_cast<int>(power) << ", mode=" << static_cast<int>(mode)
            << ", logs=" << logs.size() << "\n";
  return 0;
}
