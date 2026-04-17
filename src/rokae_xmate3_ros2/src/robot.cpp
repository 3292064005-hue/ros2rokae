#include "sdk/robot_internal.hpp"
#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"

namespace rokae::ros2 {

std::string xMateRobot::sdkVersion() {
    return rokae_xmate3_ros2::spec::xmate3::wrapperVersion();
}

} // namespace rokae::ros2
