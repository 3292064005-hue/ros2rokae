#include <string>

#include "rokae/exception.h"
#include "rokae/robot.h"

int main() {
  bool ctor_caught = false;
  try {
    rokae::xMateRobot should_throw("", "");
  } catch (const rokae::Exception &) {
    ctor_caught = true;
  }
  if (!ctor_caught) {
    return 1;
  }

  rokae::xMateRobot robot;
  bool connect_caught = false;
  try {
    robot.connectToRobot("", "");
  } catch (const rokae::Exception &) {
    connect_caught = true;
  }
  return connect_caught ? 0 : 2;
}
