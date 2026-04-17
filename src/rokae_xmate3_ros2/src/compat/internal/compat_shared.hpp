#ifndef ROKAE_COMPAT_INTERNAL_COMPAT_SHARED_HPP
#define ROKAE_COMPAT_INTERNAL_COMPAT_SHARED_HPP

#include <any>
#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <system_error>
#include <thread>
#include <exception>
#include <vector>

#include "rokae/data_types.h"
#include "rokae/error_category.hpp"
#include "rokae_xmate3_ros2/robot.hpp"

namespace rokae {
template <unsigned short DoF>
class RtMotionControlCobot;
}

namespace rokae::detail {

inline std::array<double, 16> identity_matrix16() {
  return {1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0};
}

struct CompatSdkCapabilities {
  bool xmate6_only = true;
  bool supports_io = false;
  bool supports_rl = false;
  bool supports_calibration = false;
  bool supports_avoid_singularity = false;
};

struct CompatLoopState {
  std::function<std::any(void)> callback;
  std::thread worker;
  std::atomic<bool> running{false};
  std::atomic<bool> move_started{false};
  bool use_state_data_in_loop = false;
  std::atomic<std::uint64_t> sequence{1};
  RtControllerMode current_mode = RtControllerMode::jointPosition;
  std::array<double, 3> cartesian_limit_lengths{};
  std::array<double, 16> cartesian_limit_frame{identity_matrix16()};
  mutable std::mutex exception_mutex;
  std::exception_ptr loop_exception;
};

struct CompatRobotHandle {
  std::shared_ptr<rokae::ros2::xMateRobot> backend;
  mutable std::mutex mutex;
  std::string remote_ip;
  std::string local_ip;
  Toolset toolset_cache{};
  Load model_load_cache{};
  std::array<double, 16> model_f_t_ee{identity_matrix16()};
  std::array<double, 16> model_ee_t_k{identity_matrix16()};
  std::vector<RLProjectInfo> projects_cache{};
  RLProjectInfo current_project{};
  std::weak_ptr<RtMotionControlCobot<6>> rt_controller;
  std::shared_ptr<RtMotionControlCobot<6>> rt_controller_owner;
  CompatSdkCapabilities capabilities{};

  [[nodiscard]] const CompatSdkCapabilities &sdkCapabilities() const noexcept { return capabilities; }

  CompatRobotHandle();
  CompatRobotHandle(const std::string &remote, const std::string &local);
  void resetBackend(const std::string &remote, const std::string &local);
};



[[nodiscard]] std::string publicLaneUnsupportedMessage(const CompatRobotHandle &handle,
                                                       const char *feature);
void setPublicLaneUnsupported(const std::shared_ptr<CompatRobotHandle> &handle,
                              std::error_code &ec,
                              const char *feature) noexcept;

struct CompatRtControllerHandle6 {
  std::shared_ptr<CompatRobotHandle> robot;
  std::shared_ptr<CompatLoopState> loop;
  std::array<double, 6> joint_impedance{};
  std::array<double, 6> cartesian_impedance{};
  std::array<double, 6> collision_thresholds{};
  std::array<double, 3> filter_frequencies{};
  std::array<double, 6> desired_cartesian_torque{};
  double torque_cutoff_frequency = 0.0;
  std::array<double, 16> force_control_frame{identity_matrix16()};
  FrameType force_control_type = FrameType::world;

  explicit CompatRtControllerHandle6(std::shared_ptr<CompatRobotHandle> handle)
      : robot(std::move(handle)), loop(std::make_shared<CompatLoopState>()) {}
};

}  // namespace rokae::detail

#endif  // ROKAE_COMPAT_INTERNAL_COMPAT_SHARED_HPP
