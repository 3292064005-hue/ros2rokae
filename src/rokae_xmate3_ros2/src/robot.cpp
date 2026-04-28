#include "sdk/robot_internal.hpp"
#include "rokae_xmate3_ros2/spec/xmate_er3_truth.hpp"

namespace rokae::ros2 {

std::string xMateRobot::sdkVersion() {
    return rokae_xmate3_ros2::spec::xmate_er3_truth::wrapperVersion();
}

} // namespace rokae::ros2
