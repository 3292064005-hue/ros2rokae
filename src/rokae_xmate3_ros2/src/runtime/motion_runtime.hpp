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
  [[nodiscard]] bool submit(const MotionRequest &request, std::string &message);

  void setActiveSpeedScale(double scale);
  [[nodiscard]] double activeSpeedScale() const;

  void stop(const std::string &message = "stopped");
  void reset();
  void clearForModeChange(const std::string &message = "motion mode changed");

  [[nodiscard]] RuntimeStatus status() const;
  [[nodiscard]] RuntimeStatus status(const std::string &request_id) const;
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

  mutable std::mutex mutex_;
  std::condition_variable planner_cv_;
  mutable std::condition_variable status_cv_;
  bool shutdown_ = false;
  bool queue_initialized_ = false;
  std::optional<MotionRequest> pending_request_;
  std::optional<MotionPlan> queued_plan_;
  std::string active_request_id_;
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
