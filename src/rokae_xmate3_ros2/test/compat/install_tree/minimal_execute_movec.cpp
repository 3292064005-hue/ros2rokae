#include <system_error>
#include <vector>

#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  std::error_code ec;
  rokae::MoveCCommand cmd;
  std::vector<rokae::MoveCCommand> cmds{cmd};
  robot.executeCommand(cmds, ec);
  return 0;
}
