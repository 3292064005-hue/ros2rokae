#ifndef ROKAE_XMATE3_ROS2_SDK_ROBOT_INTERNAL_HPP
#define ROKAE_XMATE3_ROS2_SDK_ROBOT_INTERNAL_HPP

#include "rokae_xmate3_ros2/robot.hpp"
#include "rokae_xmate3_ros2/model.hpp"
#include "rokae_xmate3_ros2/runtime/ros_context_owner.hpp"
#include "rokae_xmate3_ros2/sdk_catalog_policy.hpp"
#include "rokae_xmate3_ros2/utils.hpp"
#include "rokae/error_category.hpp"
#include "../runtime/pose_utils.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <any>
#include <chrono>
#include <future>
#include <mutex>
#include <system_error>
#include <thread>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rokae_xmate3_ros2/action/move_append.hpp"
#include "rokae_xmate3_ros2/msg/cartesian_position.hpp"
#include "rokae_xmate3_ros2/msg/log_info.hpp"
#include "rokae_xmate3_ros2/msg/operation_state.hpp"
#include "rokae_xmate3_ros2/msg/power_state.hpp"
#include "rokae_xmate3_ros2/srv/adjust_speed_online.hpp"
#include "rokae_xmate3_ros2/srv/calc_fk.hpp"
#include "rokae_xmate3_ros2/srv/calc_ik.hpp"
#include "rokae_xmate3_ros2/srv/calc_joint_torque.hpp"
#include "rokae_xmate3_ros2/srv/cancel_record_path.hpp"
#include "rokae_xmate3_ros2/srv/clear_servo_alarm.hpp"
#include "rokae_xmate3_ros2/srv/connect.hpp"
#include "rokae_xmate3_ros2/srv/disable_collision_detection.hpp"
#include "rokae_xmate3_ros2/srv/disable_drag.hpp"
#include "rokae_xmate3_ros2/srv/disconnect.hpp"
#include "rokae_xmate3_ros2/srv/enable_collision_detection.hpp"
#include "rokae_xmate3_ros2/srv/enable_drag.hpp"
#include "rokae_xmate3_ros2/srv/generate_s_trajectory.hpp"
#include "rokae_xmate3_ros2/srv/get_ai.hpp"
#include "rokae_xmate3_ros2/srv/get_avoid_singularity.hpp"
#include "rokae_xmate3_ros2/srv/get_base_frame.hpp"
#include "rokae_xmate3_ros2/srv/get_cart_posture.hpp"
#include "rokae_xmate3_ros2/srv/get_di.hpp"
#include "rokae_xmate3_ros2/srv/get_do.hpp"
#include "rokae_xmate3_ros2/srv/get_end_effector_torque.hpp"
#include "rokae_xmate3_ros2/srv/get_end_wrench.hpp"
#include "rokae_xmate3_ros2/srv/get_info.hpp"
#include "rokae_xmate3_ros2/srv/get_joint_pos.hpp"
#include "rokae_xmate3_ros2/srv/get_joint_torques.hpp"
#include "rokae_xmate3_ros2/srv/get_joint_vel.hpp"
#include "rokae_xmate3_ros2/srv/get_operate_mode.hpp"
#include "rokae_xmate3_ros2/srv/get_posture.hpp"
#include "rokae_xmate3_ros2/srv/get_power_state.hpp"
#include "rokae_xmate3_ros2/srv/get_profile_capabilities.hpp"
#include "rokae_xmate3_ros2/srv/get_rl_project_info.hpp"
#include "rokae_xmate3_ros2/srv/get_runtime_state_snapshot.hpp"
#include "rokae_xmate3_ros2/srv/get_rt_joint_data.hpp"
#include "rokae_xmate3_ros2/srv/get_soft_limit.hpp"
#include "rokae_xmate3_ros2/srv/get_tool_catalog.hpp"
#include "rokae_xmate3_ros2/srv/get_toolset.hpp"
#include "rokae_xmate3_ros2/srv/get_wobj_catalog.hpp"
#include "rokae_xmate3_ros2/srv/load_rl_project.hpp"
#include "rokae_xmate3_ros2/srv/map_cartesian_to_joint_torque.hpp"
#include "rokae_xmate3_ros2/srv/move_reset.hpp"
#include "rokae_xmate3_ros2/srv/move_start.hpp"
#include "rokae_xmate3_ros2/srv/pause_rl_project.hpp"
#include "rokae_xmate3_ros2/srv/query_controller_log.hpp"
#include "rokae_xmate3_ros2/srv/query_path_lists.hpp"
#include "rokae_xmate3_ros2/srv/read_register.hpp"
#include "rokae_xmate3_ros2/srv/read_register_ex.hpp"
#include "rokae_xmate3_ros2/srv/register_data_callback.hpp"
#include "rokae_xmate3_ros2/srv/remove_path.hpp"
#include "rokae_xmate3_ros2/srv/replay_path.hpp"
#include "rokae_xmate3_ros2/srv/save_record_path.hpp"
#include "rokae_xmate3_ros2/srv/send_custom_data.hpp"
#include "rokae_xmate3_ros2/srv/set_ao.hpp"
#include "rokae_xmate3_ros2/srv/set_avoid_singularity.hpp"
#include "rokae_xmate3_ros2/srv/set_default_conf_opt.hpp"
#include "rokae_xmate3_ros2/srv/set_default_speed.hpp"
#include "rokae_xmate3_ros2/srv/set_default_zone.hpp"
#include "rokae_xmate3_ros2/srv/set_di.hpp"
#include "rokae_xmate3_ros2/srv/set_do.hpp"
#include "rokae_xmate3_ros2/srv/set_motion_control_mode.hpp"
#include "rokae_xmate3_ros2/srv/set_operate_mode.hpp"
#include "rokae_xmate3_ros2/srv/set_power_state.hpp"
#include "rokae_xmate3_ros2/srv/set_project_running_opt.hpp"
#include "rokae_xmate3_ros2/srv/set_rt_control_mode.hpp"
#include "rokae_xmate3_ros2/srv/set_simulation_mode.hpp"
#include "rokae_xmate3_ros2/srv/set_soft_limit.hpp"
#include "rokae_xmate3_ros2/srv/set_toolset.hpp"
#include "rokae_xmate3_ros2/srv/set_toolset_by_name.hpp"
#include "rokae_xmate3_ros2/srv/set_x_panel_vout.hpp"
#include "rokae_xmate3_ros2/srv/start_record_path.hpp"
#include "rokae_xmate3_ros2/srv/start_rl_project.hpp"
#include "rokae_xmate3_ros2/srv/stop.hpp"
#include "rokae_xmate3_ros2/srv/stop_record_path.hpp"
#include "rokae_xmate3_ros2/srv/stop_rl_project.hpp"
#include "rokae_xmate3_ros2/srv/write_register.hpp"
#include "rokae_xmate3_ros2/srv/write_register_ex.hpp"

using namespace std::chrono_literals;
using namespace rokae;

namespace rokae::ros2 {

class xMateRobot::Impl {
public:
    explicit Impl(const std::string& node_name);
    Impl(const std::string& remote_ip, const std::string& local_ip);
    explicit Impl(const RosClientOptions& options);
    ~Impl();

    bool wait_for_service(rclcpp::ClientBase::SharedPtr client, std::error_code& ec, int timeout_s = 5);
    bool wait_for_service(rclcpp::ClientBase::SharedPtr client,
                          std::error_code& ec,
                          std::chrono::nanoseconds timeout);
    void start_executor();
    void stop_executor();
    template<typename FutureT>
    rclcpp::FutureReturnCode wait_for_future(
        const FutureT& future,
        std::chrono::nanoseconds timeout = std::chrono::seconds(30)) {
        if (!executor_) {
            std::lock_guard<std::mutex> lock(ros_call_mutex_);
            return rclcpp::spin_until_future_complete(node_, future, timeout);
        }

        const auto status = future.wait_for(timeout);
        if (status == std::future_status::ready) {
            return rclcpp::FutureReturnCode::SUCCESS;
        }
        if (status == std::future_status::timeout) {
            return rclcpp::FutureReturnCode::TIMEOUT;
        }
        return rclcpp::FutureReturnCode::INTERRUPTED;
    }

    /**
     * @brief Translate authoritative runtime failures into visible SDK errors.
     * @param ec Output error code.
     * @param fallback_available Allowed legacy compatibility branch.
     * @param operation Diagnostic operation name.
     * @return true if the caller may continue with legacy cached data.
     */
    bool allowCatalogFallback(std::error_code& ec, bool fallback_available, const char* operation);
    void publishCatalogProvenance(const std::string& provenance) const;

    void applyCatalogPolicyFromEnvironment();
    void applyCatalogPolicyOverride(const std::optional<SdkCatalogConsistencyPolicy>& override_policy);

    void cacheCommand(const rokae_xmate3_ros2::action::MoveAppend::Goal &goal);
    void ensureToolingClients();
    void ensureSafetyClients();
    void ensureProjectClients();
    void ensurePathClients();
    void ensureDynamicsClients();
    bool refreshRuntimeStateSnapshot(std::error_code& ec,
                                     bool force = false,
                                     std::chrono::milliseconds max_age = std::chrono::milliseconds(50));
    void clearRuntimeStateSnapshotCache() noexcept;
    void clearCache();
    void resetMoveAppendState();
    void handleMoveAppendResult(
        const rclcpp_action::ClientGoalHandle<rokae_xmate3_ros2::action::MoveAppend>::WrappedResult &result);
    bool checkMoveAppendFailure(std::error_code &ec);
    bool flushCachedCommands(std::error_code &ec);
    void pump_callbacks();

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::Executor> executor_;
    std::thread executor_thread_;
    std::shared_ptr<rokae_xmate3_ros2::runtime::RosContextOwner::Lease> ros_context_lease_;
    bool owns_node_ = true;
    bool attached_external_executor_ = false;

    std::string remote_ip_ = "192.168.0.160";
    std::string local_ip_ = "";
    bool connected_ = false;
    std::mutex state_mutex_;
    std::mutex ros_call_mutex_;
    sensor_msgs::msg::JointState last_joint_state_;
    rokae_xmate3_ros2::msg::OperationState last_operation_state_;

    std::mutex cache_mutex_;
    rokae_xmate3_ros2::action::MoveAppend::Goal cached_goal_;
    rclcpp_action::Client<rokae_xmate3_ros2::action::MoveAppend>::SharedPtr move_append_action_client_;
    std::vector<rclcpp_action::ClientGoalHandle<rokae_xmate3_ros2::action::MoveAppend>::SharedPtr> active_goal_handles_;
    std::mutex action_mutex_;
    bool move_append_result_ready_ = false;
    rclcpp_action::ResultCode move_append_result_code_ = rclcpp_action::ResultCode::UNKNOWN;
    bool move_append_result_success_ = true;
    bool suppress_next_stopped_move_append_result_ = false;
    std::string move_append_result_message_;

    std::error_code last_error_code_;

    void remember_last_error(const std::error_code &ec) noexcept { last_error_code_ = ec; }
    int max_cache_size_ = 100;
    unsigned long next_cmd_id_ = 1;
    bool nrt_queue_initialized_ = false;
    bool nrt_queue_has_cached_commands_ = false;
    bool nrt_queue_remote_request_known_ = false;
    std::string current_cached_command_family_;

    void resetNrtQueueState() noexcept {
        nrt_queue_initialized_ = false;
        nrt_queue_has_cached_commands_ = false;
        nrt_queue_remote_request_known_ = false;
        current_cached_command_family_.clear();
    }

    void markNrtQueueInitialized() noexcept { nrt_queue_initialized_ = true; }
    void markNrtCachedCommandsPresent() noexcept { nrt_queue_has_cached_commands_ = true; }
    void markNrtRemoteRequestKnown() noexcept { nrt_queue_remote_request_known_ = true; }

    [[nodiscard]] bool supportsIoPublicLane() const noexcept { return false; }
    [[nodiscard]] bool supportsRlPublicLane() const noexcept { return false; }
    [[nodiscard]] bool supportsCalibrationPublicLane() const noexcept { return false; }

    std::mutex event_mutex_;
    rokae::EventCallback move_event_watcher_;
    rokae::EventCallback safety_event_watcher_;
    rokae::EventInfo last_move_event_;
    rokae::EventInfo last_safety_event_;

    std::mutex state_cache_mutex_;
    std::mutex client_init_mutex_;
    std::vector<std::string> state_fields_;
    std::chrono::steady_clock::duration state_interval_{std::chrono::milliseconds(1)};
    std::unordered_map<std::string, std::any> state_cache_;
    bool runtime_state_snapshot_valid_ = false;
    std::chrono::steady_clock::time_point runtime_state_snapshot_stamp_{};
    rokae_xmate3_ros2::srv::GetRuntimeStateSnapshot::Response runtime_state_snapshot_;
    bool has_previous_state_ = false;
    std::chrono::steady_clock::time_point previous_state_time_{};
    std::chrono::steady_clock::time_point rt_state_last_update_time_{};
    std::array<double, 6> previous_joint_velocity_{};
    std::array<double, 6> previous_joint_torque_{};
    std::array<double, 6> previous_pose_abc_{};
    std::array<double, 6> previous_pose_velocity_{};
    bool has_previous_pose_ = false;
    std::string rt_state_plan_summary_ = "inactive";
    bool rt_state_plan_rejected_ = false;
    std::uint32_t rt_state_late_cycle_count_ = 0;
    double rt_state_max_gap_ms_ = 0.0;
    rokae::Toolset toolset_cache_;

    std::shared_ptr<void> rt_controller_;
    std::shared_ptr<void> model_;

    std::vector<rokae::RLProjectInfo> projects_;
    rokae::RLProjectInfo current_project_;
    std::string current_project_path_;
    std::vector<rokae::WorkToolInfo> tools_;
    std::vector<rokae::WorkToolInfo> wobjs_;
    SdkCatalogConsistencyPolicy catalog_policy_{};

    rokae::xPanelOpt::Vout xpanel_vout_ = rokae::xPanelOpt::Vout::off;
    unsigned rt_network_tolerance_ = 10;
    bool use_rci_client_ = false;

    rclcpp::Client<rokae_xmate3_ros2::srv::Connect>::SharedPtr xmate3_robot_connect_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::Disconnect>::SharedPtr xmate3_robot_disconnect_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetInfo>::SharedPtr xmate3_robot_get_info_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetProfileCapabilities>::SharedPtr xmate3_internal_get_profile_capabilities_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetRuntimeStateSnapshot>::SharedPtr xmate3_internal_get_runtime_state_snapshot_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetPowerState>::SharedPtr xmate3_robot_get_power_state_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetPowerState>::SharedPtr xmate3_robot_set_power_state_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetOperateMode>::SharedPtr xmate3_robot_get_operate_mode_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetOperateMode>::SharedPtr xmate3_robot_set_operate_mode_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::QueryControllerLog>::SharedPtr xmate3_robot_query_log_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::ClearServoAlarm>::SharedPtr xmate3_robot_clear_servo_alarm_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetJointPos>::SharedPtr xmate3_robot_get_joint_pos_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetJointVel>::SharedPtr xmate3_robot_get_joint_vel_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetJointTorques>::SharedPtr xmate3_robot_get_joint_torque_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetPosture>::SharedPtr xmate3_robot_get_posture_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetCartPosture>::SharedPtr xmate3_robot_get_cart_posture_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetBaseFrame>::SharedPtr xmate3_robot_get_base_frame_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::CalcIk>::SharedPtr xmate3_robot_calc_ik_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::CalcFk>::SharedPtr xmate3_robot_calc_fk_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetToolset>::SharedPtr xmate3_robot_get_toolset_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetToolset>::SharedPtr xmate3_robot_set_toolset_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetToolsetByName>::SharedPtr xmate3_robot_set_toolset_by_name_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::EnableCollisionDetection>::SharedPtr xmate3_robot_enable_collision_detection_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::DisableCollisionDetection>::SharedPtr xmate3_robot_disable_collision_detection_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetSoftLimit>::SharedPtr xmate3_robot_get_soft_limit_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetSoftLimit>::SharedPtr xmate3_robot_set_soft_limit_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetMotionControlMode>::SharedPtr xmate3_motion_set_control_mode_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::MoveReset>::SharedPtr xmate3_motion_reset_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::MoveStart>::SharedPtr xmate3_motion_start_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::Stop>::SharedPtr xmate3_motion_stop_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetDefaultSpeed>::SharedPtr xmate3_motion_set_default_speed_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetDefaultZone>::SharedPtr xmate3_motion_set_default_zone_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetDefaultConfOpt>::SharedPtr xmate3_motion_set_default_conf_opt_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::AdjustSpeedOnline>::SharedPtr xmate3_motion_adjust_speed_online_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetRtControlMode>::SharedPtr xmate3_rt_set_control_mode_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetRtJointData>::SharedPtr xmate3_rt_get_joint_data_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SendCustomData>::SharedPtr xmate3_comm_send_custom_data_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::RegisterDataCallback>::SharedPtr xmate3_comm_register_data_callback_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::ReadRegister>::SharedPtr xmate3_comm_read_register_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::ReadRegisterEx>::SharedPtr xmate3_comm_read_register_ex_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::WriteRegister>::SharedPtr xmate3_comm_write_register_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::WriteRegisterEx>::SharedPtr xmate3_comm_write_register_ex_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetXPanelVout>::SharedPtr xmate3_comm_set_xpanel_vout_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetDI>::SharedPtr xmate3_io_get_di_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetDO>::SharedPtr xmate3_io_get_do_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetDI>::SharedPtr xmate3_io_set_di_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetDO>::SharedPtr xmate3_io_set_do_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetAI>::SharedPtr xmate3_io_get_ai_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetAO>::SharedPtr xmate3_io_set_ao_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetSimulationMode>::SharedPtr xmate3_io_set_simulation_mode_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::LoadRLProject>::SharedPtr xmate3_rl_load_project_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::StartRLProject>::SharedPtr xmate3_rl_start_project_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::StopRLProject>::SharedPtr xmate3_rl_stop_project_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::PauseRLProject>::SharedPtr xmate3_rl_pause_project_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetProjectRunningOpt>::SharedPtr xmate3_rl_set_running_opt_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetRlProjectInfo>::SharedPtr xmate3_rl_get_project_info_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetToolCatalog>::SharedPtr xmate3_rl_get_tools_info_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetWobjCatalog>::SharedPtr xmate3_rl_get_wobjs_info_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SetAvoidSingularity>::SharedPtr xmate3_cobot_set_avoid_singularity_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetAvoidSingularity>::SharedPtr xmate3_cobot_get_avoid_singularity_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetEndEffectorTorque>::SharedPtr xmate3_cobot_get_end_torque_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GetEndWrench>::SharedPtr xmate3_cobot_get_end_wrench_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::CalcJointTorque>::SharedPtr xmate3_dyn_calc_joint_torque_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::GenerateSTrajectory>::SharedPtr xmate3_dyn_generate_s_trajectory_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::MapCartesianToJointTorque>::SharedPtr xmate3_dyn_map_cartesian_to_joint_torque_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::EnableDrag>::SharedPtr xmate3_cobot_enable_drag_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::DisableDrag>::SharedPtr xmate3_cobot_disable_drag_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::StartRecordPath>::SharedPtr xmate3_cobot_start_record_path_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::StopRecordPath>::SharedPtr xmate3_cobot_stop_record_path_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::CancelRecordPath>::SharedPtr xmate3_cobot_cancel_record_path_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::SaveRecordPath>::SharedPtr xmate3_cobot_save_record_path_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::ReplayPath>::SharedPtr xmate3_cobot_replay_path_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::RemovePath>::SharedPtr xmate3_cobot_remove_path_client_;
    rclcpp::Client<rokae_xmate3_ros2::srv::QueryPathLists>::SharedPtr xmate3_cobot_query_path_lists_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<rokae_xmate3_ros2::msg::OperationState>::SharedPtr operation_state_sub_;

private:
    void init_node();
    void init_clients();
    void init_subscribers();
    void loadSdkPoliciesFromEnvironment();
};

template <typename ImplLike>
class ScopedLastError final {
public:
    ScopedLastError(ImplLike impl, std::error_code& ec) noexcept
        : impl_(std::move(impl)), ec_(ec) {}

    ~ScopedLastError() {
        if (impl_) {
            impl_->remember_last_error(ec_);
        }
    }

    ScopedLastError(const ScopedLastError&) = delete;
    ScopedLastError& operator=(const ScopedLastError&) = delete;

private:
    ImplLike impl_;
    std::error_code& ec_;
};

template <typename ImplLike>
inline ScopedLastError<ImplLike> track_last_error(ImplLike impl, std::error_code& ec) noexcept {
    return ScopedLastError<ImplLike>(std::move(impl), ec);
}

} // namespace rokae::ros2

#endif
