#ifndef ROKAE_XMATE3_ROS2_RUNTIME_MOTION_RUNTIME_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_MOTION_RUNTIME_HPP

#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

#include "runtime/executor_core.hpp"
#include "runtime/runtime_state_machine.hpp"
#include "rokae_xmate3_ros2/runtime/owner_arbiter.hpp"
#include "runtime/planner_core.hpp"
#include "rokae_xmate3_ros2/runtime/shutdown_coordinator.hpp"
#include "runtime/runtime_types.hpp"

namespace rokae_xmate3_ros2::runtime {

class MotionRuntime {
 public:
  MotionRuntime();
  ~MotionRuntime();

  void attachBackend(BackendInterface *backend);
  void setExecutorConfig(const MotionExecutorConfig &config);
  [[nodiscard]] bool canAcceptRequest() const;
  [[nodiscard]] bool queueInitialized() const;
  [[nodiscard]] bool queueHasPendingCommands() const;
  /**
   * @brief Queue a non-realtime motion request without starting execution.
   * @param request Normalized-or-normalizable motion request. `request_id` and at least one command are required.
   * @param message Output status/error text. On success this is cleared.
   * @return true when the request is accepted into the staged public xMate6 queue.
   * @throws None.
   * @note Boundary behavior: requires `moveReset()`-initialized queue state and rejects concurrent active requests.
   */
  [[nodiscard]] bool queue(const MotionRequest &request, std::string &message);

  /**
   * @brief Submit a request directly into planning/execution without staged queue semantics.
   * @param request Motion request.
   * @param message Output status/error text. On success this is cleared.
   * @return true when planning has been armed.
   * @throws None.
   * @note Boundary behavior: kept for runtime-internal flows such as replay-path dispatch; public MoveAppend uses `queue()`.
   */
  [[nodiscard]] bool submit(const MotionRequest &request, std::string &message);

  /**
   * @brief Commit the staged queue entry so planning/execution can begin or resume.
   * @param message Output status/error text.
   * @return true when the staged request was committed, or when the same request was already started.
   * @throws None.
   * @note Boundary behavior: this is the single NRT start authority behind `moveStart()`.
   */
  [[nodiscard]] bool commitQueuedRequest(std::string &message);

  void setActiveSpeedScale(double scale);
  [[nodiscard]] double activeSpeedScale() const;

  /**
   * @brief Pause the active or staged request and synthesize a resumable staged request from the current snapshot.
   * @param snapshot Latest authoritative robot snapshot used to rebuild the resume seed.
   * @param message Pause reason propagated to runtime status.
   * @throws None.
   * @note Boundary behavior: active execution is cancelled/reset, but the request identity is preserved for a later `moveStart()` resume.
   */
  void pause(const RobotSnapshot &snapshot, const std::string &message = "paused");

  /**
   * @brief Pause the active or staged request when no authoritative backend snapshot is currently available.
   * @param message Pause reason propagated to runtime status.
   * @throws None.
   * @note Boundary behavior: a deterministic synthetic snapshot is derived from staged/request seed joints so that
   *       `stop()` keeps pause/resume semantics instead of degrading to terminal stop on backend-less paths.
   */
  void pause(const std::string &message);

  /**
   * @brief Abort the active request and drive the runtime into a terminal stopped state.
   * @param message Abort reason propagated to runtime status.
   * @throws None.
   * @note Boundary behavior: unlike `pause()`, this clears resumable staged state.
   */
  void stop(const std::string &message = "stopped");
  void reset();
  void clearForModeChange(const std::string &message = "motion mode changed");

  [[nodiscard]] RuntimeStatus status() const;
  [[nodiscard]] RuntimeStatus status(const std::string &request_id) const;
  /**
   * @brief Read the latest runtime-owned robot snapshot from the authoritative backend path.
   * @param snapshot Output joint/power snapshot.
   * @return true when a backend snapshot was available; false when a deterministic synthetic snapshot was used.
   * @throws None.
   * @note Boundary behavior: when no backend is attached, a synthetic pause snapshot is returned so
   *       query surfaces still remain on the runtime/coordinator authority path instead of falling back
   *       to external raw joint fetchers.
   */
  [[nodiscard]] bool readAuthoritativeSnapshot(RobotSnapshot &snapshot) const;
  [[nodiscard]] RuntimeView view() const;
  [[nodiscard]] RuntimeView view(const std::string &request_id) const;
  [[nodiscard]] RuntimePhase runtimePhase() const;
  [[nodiscard]] RuntimeContractFacts contractFacts() const;
  [[nodiscard]] std::size_t activeRequestCount() const;
  [[nodiscard]] std::size_t activeGoalCount() const;
  [[nodiscard]] RuntimeStatus waitForUpdate(const std::string &request_id,
                                            std::uint64_t last_revision,
                                            std::chrono::milliseconds timeout) const;
  [[nodiscard]] RuntimeStatus tick(BackendInterface &backend, double dt);

 private:
  void plannerLoop();
  void rememberStatus(RuntimeStatus &status);
  [[nodiscard]] RuntimeView buildViewLocked(const std::string *request_id) const;
  [[nodiscard]] bool setOwnerLocked(ControlOwner owner, const std::string &reason);
  [[nodiscard]] bool syncOwnerLocked(BackendInterface &backend,
                                     ControlOwner owner,
                                     const std::string &reason);
  void setRuntimePhaseLocked(RuntimePhase phase, const std::string &message = {});
  [[nodiscard]] MotionRequest normalizeRequestLocked(MotionRequest request);
  [[nodiscard]] bool startPlanningLocked(const MotionRequest &request, const std::string &message);
  [[nodiscard]] MotionRequest buildResumeRequestLocked(const RobotSnapshot &snapshot) const;
  [[nodiscard]] RobotSnapshot buildSyntheticPauseSnapshotLocked() const;
  [[nodiscard]] std::size_t activeCommandOffsetLocked() const;
  [[nodiscard]] std::size_t activeTotalSegmentsLocked() const;

  mutable std::mutex mutex_;
  std::condition_variable planner_cv_;
  mutable std::condition_variable status_cv_;
  bool shutdown_ = false;
  bool queue_initialized_ = false;
  std::optional<MotionRequest> staged_request_;
  std::optional<MotionRequest> pending_request_;
  std::optional<MotionPlan> queued_plan_;
  std::optional<MotionRequest> active_request_;
  std::string active_request_id_;
  std::uint64_t active_request_token_ = 0;
  std::uint64_t next_request_token_ = 1;
  RuntimeStatus active_status_;
  std::unordered_map<std::string, RuntimeStatus> status_cache_;
  std::deque<std::string> status_order_;
  std::uint64_t next_status_revision_ = 1;
  std::chrono::steady_clock::time_point last_view_update_time_{std::chrono::steady_clock::now()};
  std::thread planner_thread_;
  MotionPlanner planner_;
  MotionExecutor executor_;
  double active_speed_scale_ = 1.0;
  BackendInterface *backend_ = nullptr;
  OwnerArbiter owner_arbiter_;
  bool using_backend_trajectory_ = false;
  std::optional<MotionPlan> active_trajectory_plan_;
  std::optional<TrajectoryExecutionGoal> active_trajectory_goal_;
  double dispatched_speed_scale_ = 1.0;
  RuntimePhase runtime_phase_ = RuntimePhase::idle;
  RuntimeStateMachine state_machine_;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
