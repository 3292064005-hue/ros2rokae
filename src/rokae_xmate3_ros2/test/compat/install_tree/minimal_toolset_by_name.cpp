#include <system_error>

#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  std::error_code ec;
  const rokae::Toolset selected = robot.setToolset("tool0", "wobj0", ec);
  (void)selected;
  return 0;
}
