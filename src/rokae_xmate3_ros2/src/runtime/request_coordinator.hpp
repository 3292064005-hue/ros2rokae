#ifndef ROKAE_XMATE3_ROS2_RUNTIME_REQUEST_COORDINATOR_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_REQUEST_COORDINATOR_HPP

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "runtime/runtime_state.hpp"
#include "runtime/motion_runtime.hpp"
#include "runtime/session_state.hpp"
#include "rokae_xmate3_ros2/action/move_append.hpp"

namespace rokae_xmate3_ros2::runtime {

struct SubmissionResult {
  bool success = false;
  std::string request_id;
  std::string message;
};

class MotionRequestCoordinator {
 public:
  MotionRequestCoordinator(MotionOptionsState &motion_options_state,
                           ToolingState &tooling_state,
                           SessionState &session_state,
                           MotionRuntime &motion_runtime);

  [[nodiscard]] RuntimeView currentView() const;
  /**
   * @brief Read joint state through the runtime/coordinator authority path.
   * @param pos Output joint position sample.
   * @param vel Output joint velocity sample.
   * @param tau Output joint torque sample.
   * @return true when a live backend snapshot was available; false when a deterministic runtime-owned
   *         synthetic snapshot was used.
   * @throws None.
   */
  [[nodiscard]] bool readAuthorityJointState(std::array<double, 6> &pos,
                                             std::array<double, 6> &vel,
                                             std::array<double, 6> &tau) const;
  [[nodiscard]] bool canAcceptRequest() const;

  /**
   * @brief Convert a public MoveAppend goal into a staged runtime request without starting execution.
   * @param goal Public action goal containing one homogeneous NRT command batch.
   * @param joint_position Current joint snapshot used as the planning seed.
   * @param trajectory_dt Runtime sampling step.
   * @param request_id Stable request identifier exposed to status/action feedback.
   * @return SubmissionResult with success flag and diagnostic message.
   * @throws None.
   */
  [[nodiscard]] SubmissionResult queueMoveAppend(
      const rokae_xmate3_ros2::action::MoveAppend::Goal &goal,
      const std::array<double, 6> &joint_position,
      double trajectory_dt,
      const std::string &request_id);

  [[nodiscard]] SubmissionResult submitReplayPath(
      const ReplayPathAsset &replay_asset,
      double rate,
      const std::array<double, 6> &joint_position,
      double trajectory_dt,
      const std::string &request_id);

  /**
   * @brief Commit the staged NRT request so planning/execution begins or resumes.
   * @return SubmissionResult with success flag and propagated runtime message.
   * @throws None.
   */
  [[nodiscard]] SubmissionResult startQueuedRequest();

  [[nodiscard]] RuntimeStatus waitForUpdate(const std::string &request_id,
                                            std::uint64_t last_revision,
                                            std::chrono::milliseconds timeout) const;

  /**
   * @brief Pause the active request using the latest joint snapshot to build a resumable staged request.
   * @param joint_position Latest authoritative joint sample.
   * @param message Pause reason.
   * @throws None.
   */
  void pause(const std::array<double, 6> &joint_position, const std::string &message);

  /**
   * @brief Abort the active request and clear resumable staged state.
   * @param message Abort reason.
   * @throws None.
   */
  void abort(const std::string &message);
  void stop(const std::string &message);
  void reset();

 private:
  [[nodiscard]] MotionRequestContext buildContext(const std::string &request_id,
                                                  const std::array<double, 6> &joint_position,
                                                  double trajectory_dt) const;

  MotionOptionsState &motion_options_state_;
  ToolingState &tooling_state_;
  SessionState &session_state_;
  MotionRuntime &motion_runtime_;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
