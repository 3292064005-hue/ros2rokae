#include "runtime/runtime_context.hpp"

namespace rokae_xmate3_ros2::runtime {

RuntimeContext::RuntimeContext()
    : session_state_(std::make_shared<SessionState>()),
      motion_options_state_(std::make_shared<MotionOptionsState>()),
      tooling_state_(std::make_shared<ToolingState>()),
      data_store_state_(std::make_shared<DataStoreState>()),
      program_state_(std::make_shared<ProgramState>()),
      controller_state_(session_state_, motion_options_state_, tooling_state_, data_store_state_, program_state_),
      request_coordinator_(*motion_options_state_, motion_runtime_) {}

void RuntimeContext::attachBackend(BackendInterface *backend) { backend_ = backend; }
BackendInterface *RuntimeContext::backend() const { return backend_; }

ControllerState &RuntimeContext::controllerState() { return controller_state_; }
const ControllerState &RuntimeContext::controllerState() const { return controller_state_; }

MotionRuntime &RuntimeContext::motionRuntime() { return motion_runtime_; }
const MotionRuntime &RuntimeContext::motionRuntime() const { return motion_runtime_; }
MotionRequestCoordinator &RuntimeContext::requestCoordinator() { return request_coordinator_; }
const MotionRequestCoordinator &RuntimeContext::requestCoordinator() const { return request_coordinator_; }

SessionState &RuntimeContext::sessionState() { return *session_state_; }
const SessionState &RuntimeContext::sessionState() const { return *session_state_; }
MotionOptionsState &RuntimeContext::motionOptionsState() { return *motion_options_state_; }
const MotionOptionsState &RuntimeContext::motionOptionsState() const { return *motion_options_state_; }
ToolingState &RuntimeContext::toolingState() { return *tooling_state_; }
const ToolingState &RuntimeContext::toolingState() const { return *tooling_state_; }
DataStoreState &RuntimeContext::dataStoreState() { return *data_store_state_; }
const DataStoreState &RuntimeContext::dataStoreState() const { return *data_store_state_; }
ProgramState &RuntimeContext::programState() { return *program_state_; }
const ProgramState &RuntimeContext::programState() const { return *program_state_; }

OperationStateContext RuntimeContext::operationStateContext() const {
  return session_state_->makeOperationStateContext(program_state_->rlProjectRunning());
}

RuntimeView RuntimeContext::currentRuntimeView() const {
  return request_coordinator_.currentView();
}

}  // namespace rokae_xmate3_ros2::runtime
