#include <system_error>
#include "rokae/robot.h"

int main() {
  std::error_code ec;
  rokae::xMateRobot robot;
  const auto version = rokae::BaseRobot::sdkVersion();
  (void)version;
  (void)robot.lastErrorCode();
  robot.robotInfo(ec);
  return 0;
}
