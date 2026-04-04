#include "compat/internal/compat_shared.hpp"

namespace rokae::detail {

CompatRobotHandle::CompatRobotHandle()
    : backend(std::make_shared<rokae::ros2::xMateRobot>()) {}

CompatRobotHandle::CompatRobotHandle(const std::string &remote, const std::string &local)
    : backend(std::make_shared<rokae::ros2::xMateRobot>(remote, local)),
      remote_ip(remote),
      local_ip(local) {}

void CompatRobotHandle::resetBackend(const std::string &remote, const std::string &local) {
  remote_ip = remote;
  local_ip = local;
  backend = std::make_shared<rokae::ros2::xMateRobot>(remote_ip, local_ip);
  rt_controller.reset();
}



[[nodiscard]] std::string publicLaneUnsupportedMessage(const CompatRobotHandle &handle,
                                                       const char *feature) {
  const auto &caps = handle.sdkCapabilities();
  const std::string scope = caps.xmate6_only ? "public xMate6 compat lane" : "public compat lane";
  return scope + " does not support " + (feature != nullptr ? std::string(feature) : std::string{"the requested feature"});
}

void setPublicLaneUnsupported(const std::shared_ptr<CompatRobotHandle> &handle,
                              std::error_code &ec,
                              const char *feature) noexcept {
  ec = rokae::make_error_code(rokae::SdkError::not_implemented);
  if (handle && handle->backend) {
    handle->backend->rememberCompatError(ec);
  }
}

}  // namespace rokae::detail
