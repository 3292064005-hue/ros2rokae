#include "rokae/exception.h"
#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  bool caught_single_arg = false;
  try {
    robot.connectToRobot("");
  } catch (const rokae::Exception &) {
    caught_single_arg = true;
  }

  bool caught_two_args = false;
  try {
    robot.connectToRobot("", "192.168.0.22");
  } catch (const rokae::Exception &) {
    caught_two_args = true;
  }

  return (caught_single_arg && caught_two_args) ? 0 : 1;
}
