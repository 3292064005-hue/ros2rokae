#include <memory>
#include "rokae/motion_control_rt.h"

int main() {
  std::shared_ptr<rokae::RtMotionControl<rokae::WorkType::collaborative, 6>> controller;
  (void)controller;
  return 0;
}
