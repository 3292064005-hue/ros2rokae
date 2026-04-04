#include <system_error>

#include "rokae/robot.h"

int main() {
  rokae::xMateRobot robot;
  std::error_code ec;

  robot.enableDrag(rokae::DragParameter::cartesianSpace, rokae::DragParameter::freely, ec);
  robot.disableDrag(ec);

  robot.startRecordPath(30, ec);
  robot.stopRecordPath(ec);
  robot.saveRecordPath("track0", ec);
  (void)robot.queryPathLists(ec);
  robot.replayPath("track0", 1.0, ec);
  robot.removePath("track0", ec, false);

  return 0;
}
