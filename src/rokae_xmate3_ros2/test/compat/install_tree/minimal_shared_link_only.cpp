#include <rokae/robot.h>

int main() {
  rokae::xMateRobot robot;
  std::error_code ec;
  robot.connectToRobot(ec);
  robot.disconnectFromRobot(ec);
  return 0;
}
