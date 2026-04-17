#include "runtime/service_facade.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <utility>

#include <Eigen/Dense>

#include "runtime/planner_core.hpp"
#include "runtime/planning_utils.hpp"
#include "runtime/pose_utils.hpp"
#include "runtime/service_facade_utils.hpp"
#include "runtime/unified_retimer.hpp"
#include "rokae_xmate3_ros2/gazebo/model_facade.hpp"

namespace rokae_xmate3_ros2::runtime {

QueryFacade::QueryFacade(SessionState &session_state,
                         MotionOptionsState &motion_options_state,
                         ToolingState &tooling_state,
                         DataStoreState &data_store_state,
                         ProgramState &program_state,
                         RuntimeDiagnosticsState &diagnostics_state,
                         MotionRuntime &motion_runtime,
                         MotionRequestCoordinator &request_coordinator,
                         gazebo::xMate3Kinematics &kinematics,
                         JointStateFetcher joint_state_fetcher,
                         TimeProvider time_provider,
                         TrajectoryDtProvider trajectory_dt_provider,
                         int joint_num)
    : session_state_(session_state),
      motion_options_state_(motion_options_state),
      tooling_state_(tooling_state),
      data_store_state_(data_store_state),
      program_state_(program_state),
      diagnostics_state_(diagnostics_state),
      motion_runtime_(motion_runtime),
      request_coordinator_(request_coordinator),
      kinematics_(kinematics),
      joint_state_fetcher_(std::move(joint_state_fetcher)),
      time_provider_(std::move(time_provider)),
      trajectory_dt_provider_(std::move(trajectory_dt_provider)),
      joint_num_(joint_num) {}

RuntimeView QueryFacade::authorityView() const {
  return request_coordinator_.currentView();
}

void QueryFacade::readAuthorityJointState(std::array<double, 6> &pos,
                                          std::array<double, 6> &vel,
                                          std::array<double, 6> &tau) const {
  const bool live_backend = request_coordinator_.readAuthorityJointState(pos, vel, tau);
  (void)live_backend;
}

}  // namespace rokae_xmate3_ros2::runtime
