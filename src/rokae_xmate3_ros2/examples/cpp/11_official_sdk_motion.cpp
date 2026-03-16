#include <array>
#include <iostream>

#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  error_code ec;
  robot.connectToRobot(ec);
  robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
  robot.moveReset(ec);

  rokae::MoveAbsJCommand home({0.0, 0.1, 0.0, 0.2, 0.0, 0.3});
  home.speed = 200;
  home.zone = 5;

  std::string cmd_id;
  robot.moveAppend({home}, cmd_id, ec);
  robot.moveStart(ec);
  robot.stop(ec);

  rokae::CartesianPosition target({0.35, 0.0, 0.45, 3.14159, 0.0, 0.0});
  rokae::MoveLCommand line(target);
  robot.executeCommand({line}, ec);

  std::cout << "Last error: " << robot.lastErrorCode().message() << "\n";
  return 0;
}
