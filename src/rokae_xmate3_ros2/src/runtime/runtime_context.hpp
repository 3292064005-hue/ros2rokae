#ifndef ROKAE_XMATE3_ROS2_RUNTIME_CONTEXT_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_CONTEXT_HPP

#include <memory>

#include "runtime/controller_state.hpp"
#include "runtime/motion_runtime.hpp"
#include "runtime/request_coordinator.hpp"
#include "runtime/runtime_state.hpp"
#include "runtime/runtime_types.hpp"

namespace rokae_xmate3_ros2::runtime {

class RuntimeContext {
 public:
  RuntimeContext();

  void attachBackend(BackendInterface *backend);
  [[nodiscard]] BackendInterface *backend() const;

  [[nodiscard]] ControllerState &controllerState();
  [[nodiscard]] const ControllerState &controllerState() const;

  [[nodiscard]] MotionRuntime &motionRuntime();
  [[nodiscard]] const MotionRuntime &motionRuntime() const;
  [[nodiscard]] MotionRequestCoordinator &requestCoordinator();
  [[nodiscard]] const MotionRequestCoordinator &requestCoordinator() const;

  [[nodiscard]] SessionState &sessionState();
  [[nodiscard]] const SessionState &sessionState() const;
  [[nodiscard]] MotionOptionsState &motionOptionsState();
  [[nodiscard]] const MotionOptionsState &motionOptionsState() const;
  [[nodiscard]] ToolingState &toolingState();
  [[nodiscard]] const ToolingState &toolingState() const;
  [[nodiscard]] DataStoreState &dataStoreState();
  [[nodiscard]] const DataStoreState &dataStoreState() const;
  [[nodiscard]] ProgramState &programState();
  [[nodiscard]] const ProgramState &programState() const;

  [[nodiscard]] OperationStateContext operationStateContext() const;
  [[nodiscard]] RuntimeView currentRuntimeView() const;

 private:
  std::shared_ptr<SessionState> session_state_;
  std::shared_ptr<MotionOptionsState> motion_options_state_;
  std::shared_ptr<ToolingState> tooling_state_;
  std::shared_ptr<DataStoreState> data_store_state_;
  std::shared_ptr<ProgramState> program_state_;
  ControllerState controller_state_;
  MotionRuntime motion_runtime_;
  MotionRequestCoordinator request_coordinator_;
  BackendInterface *backend_ = nullptr;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
