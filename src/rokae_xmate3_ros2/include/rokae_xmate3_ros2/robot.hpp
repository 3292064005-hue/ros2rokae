#ifndef ROKAE_XMATE3_ROS2_ROBOT_H
#define ROKAE_XMATE3_ROS2_ROBOT_H

// 核心依赖头文件
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <array>
#include <vector>
#include <memory>
#include <functional>
#include <chrono>
#include <system_error>
#include <cstdint>
#include <future>
#include <mutex>
#include <thread>

// 珞石类型定义
#include "rokae_xmate3_ros2/types.hpp"

// ROS2 Msg接口
#include "sensor_msgs/msg/joint_state.hpp"
#include "rokae_xmate3_ros2/msg/operation_state.hpp"
#include "rokae_xmate3_ros2/msg/power_state.hpp"
#include "rokae_xmate3_ros2/msg/log_info.hpp"
#include "rokae_xmate3_ros2/msg/cartesian_position.hpp"
// ROS2 Srv接口（严格匹配ROS2命名规范：大驼峰.srv → 下划线小写.hpp）
#include "rokae_xmate3_ros2/srv/connect.hpp"
#include "rokae_xmate3_ros2/srv/disconnect.hpp"
#include "rokae_xmate3_ros2/srv/get_info.hpp"
#include "rokae_xmate3_ros2/srv/get_power_state.hpp"
#include "rokae_xmate3_ros2/srv/set_power_state.hpp"
#include "rokae_xmate3_ros2/srv/get_operate_mode.hpp"
#include "rokae_xmate3_ros2/srv/set_operate_mode.hpp"
#include "rokae_xmate3_ros2/srv/get_joint_pos.hpp"
#include "rokae_xmate3_ros2/srv/get_joint_vel.hpp"
#include "rokae_xmate3_ros2/srv/get_joint_torque.hpp"
#include "rokae_xmate3_ros2/srv/get_posture.hpp"
#include "rokae_xmate3_ros2/srv/get_cart_posture.hpp"
#include "rokae_xmate3_ros2/srv/get_base_frame.hpp"
#include "rokae_xmate3_ros2/srv/get_toolset.hpp"
#include "rokae_xmate3_ros2/srv/set_toolset.hpp"
#include "rokae_xmate3_ros2/srv/set_toolset_by_name.hpp"
#include "rokae_xmate3_ros2/srv/calc_ik.hpp"
#include "rokae_xmate3_ros2/srv/calc_fk.hpp"
#include "rokae_xmate3_ros2/srv/clear_servo_alarm.hpp"
#include "rokae_xmate3_ros2/srv/enable_collision_detection.hpp"
#include "rokae_xmate3_ros2/srv/disable_collision_detection.hpp"
#include "rokae_xmate3_ros2/srv/get_soft_limit.hpp"
#include "rokae_xmate3_ros2/srv/set_soft_limit.hpp"
#include "rokae_xmate3_ros2/srv/set_motion_control_mode.hpp"
#include "rokae_xmate3_ros2/srv/move_reset.hpp"
#include "rokae_xmate3_ros2/srv/move_start.hpp"
#include "rokae_xmate3_ros2/srv/stop.hpp"
#include "rokae_xmate3_ros2/srv/set_default_speed.hpp"
#include "rokae_xmate3_ros2/srv/set_default_zone.hpp"
#include "rokae_xmate3_ros2/srv/set_default_conf_opt.hpp"
#include "rokae_xmate3_ros2/srv/adjust_speed_online.hpp"
#include "rokae_xmate3_ros2/srv/get_di.hpp"
#include "rokae_xmate3_ros2/srv/get_do.hpp"
#include "rokae_xmate3_ros2/srv/set_di.hpp"
#include "rokae_xmate3_ros2/srv/set_do.hpp"
#include "rokae_xmate3_ros2/srv/get_ai.hpp"
#include "rokae_xmate3_ros2/srv/set_ao.hpp"
#include "rokae_xmate3_ros2/srv/set_simulation_mode.hpp"
#include "rokae_xmate3_ros2/srv/enable_drag.hpp"
#include "rokae_xmate3_ros2/srv/disable_drag.hpp"
#include "rokae_xmate3_ros2/srv/start_record_path.hpp"
#include "rokae_xmate3_ros2/srv/stop_record_path.hpp"
#include "rokae_xmate3_ros2/srv/cancel_record_path.hpp"
#include "rokae_xmate3_ros2/srv/save_record_path.hpp"
#include "rokae_xmate3_ros2/srv/replay_path.hpp"
#include "rokae_xmate3_ros2/srv/remove_path.hpp"
#include "rokae_xmate3_ros2/srv/query_path_lists.hpp"
#include "rokae_xmate3_ros2/srv/query_controller_log.hpp"
#include "rokae_xmate3_ros2/srv/set_rt_control_mode.hpp"
#include "rokae_xmate3_ros2/srv/get_rt_joint_data.hpp"
#include "rokae_xmate3_ros2/srv/send_custom_data.hpp"
#include "rokae_xmate3_ros2/srv/register_data_callback.hpp"
#include "rokae_xmate3_ros2/srv/calc_joint_torque.hpp"
#include "rokae_xmate3_ros2/srv/generate_s_trajectory.hpp"
#include "rokae_xmate3_ros2/srv/map_cartesian_to_joint_torque.hpp"
#include "rokae_xmate3_ros2/srv/load_rl_project.hpp"
#include "rokae_xmate3_ros2/srv/start_rl_project.hpp"
#include "rokae_xmate3_ros2/srv/stop_rl_project.hpp"
#include "rokae_xmate3_ros2/srv/read_register.hpp"
#include "rokae_xmate3_ros2/srv/write_register.hpp"
#include "rokae_xmate3_ros2/srv/set_avoid_singularity.hpp"
#include "rokae_xmate3_ros2/srv/get_avoid_singularity.hpp"
#include "rokae_xmate3_ros2/srv/get_end_torque.hpp"
#include "rokae_xmate3_ros2/action/move_append.hpp"

namespace rokae::ros2 {

class xMateRobot {
public:
    // 构造函数：节点名模式（仿真/通用）
    explicit xMateRobot(const std::string& node_name = "xmate3_robot");
    // 构造函数：IP直连模式（真机兼容，与原生SDK接口对齐）
    xMateRobot(const std::string& remote_ip, const std::string& local_ip = "");
    // 析构函数
    ~xMateRobot();

    // ==================== 4.3 机器人基本操作及信息查询 ====================
    // 连接/断开机器人
    void connectToRobot(std::error_code& ec);
    void disconnectFromRobot(std::error_code& ec);
    // 基础信息查询
    static std::string sdkVersion();
    rokae::Info robotInfo(std::error_code& ec);
    rokae::PowerState powerState(std::error_code& ec);
    void setPowerState(bool on, std::error_code& ec);
    rokae::OperateMode operateMode(std::error_code& ec);
    void setOperateMode(rokae::OperateMode mode, std::error_code& ec);
    rokae::OperationState operationState(std::error_code& ec);
    std::vector<rokae::LogInfo> queryControllerLog(unsigned int count, std::error_code& ec);
    void clearServoAlarm(std::error_code& ec);

    // ==================== 4.3 位姿/关节/运动学核心接口 ====================
    std::array<double, 6> jointPos(std::error_code& ec);
    std::array<double, 6> jointVel(std::error_code& ec);
    std::array<double, 6> jointTorque(std::error_code& ec);
    std::array<double, 6> posture(rokae::CoordinateType ct, std::error_code& ec);
    rokae::CartesianPosition cartPosture(rokae::CoordinateType ct, std::error_code& ec);
    std::array<double, 6> baseFrame(std::error_code& ec);
    // 正逆解
    rokae::JointPosition calcIk(const rokae::CartesianPosition& posture, std::error_code& ec);
    rokae::CartesianPosition calcFk(const rokae::JointPosition& joints, std::error_code& ec);

    // ==================== 4.3 工具/坐标系/安全接口 ====================
    rokae::Toolset toolset(std::error_code& ec);
    void setToolset(const rokae::Toolset& toolset, std::error_code& ec);
    void setToolset(const std::string& toolName, const std::string& wobjName, std::error_code& ec);
    // 碰撞检测
    void enableCollisionDetection(const std::array<double, 6>& sensitivity, rokae::StopLevel behaviour, double fallback, std::error_code& ec);
    void disableCollisionDetection(std::error_code& ec);
    // 软限位
    bool getSoftLimit(std::array<std::array<double,2>,6>& limits, std::error_code& ec);
    void setSoftLimit(bool enable, std::error_code& ec, const std::array<std::array<double,2>,6>& limits = {});

    // ==================== 4.4 非实时运动控制接口 ====================
    void setMotionControlMode(rokae::MotionControlMode mode, std::error_code& ec);
    void moveReset(std::error_code& ec);
    void moveStart(std::error_code& ec);
    void stop(std::error_code& ec);
    void setDefaultSpeed(int speed, std::error_code& ec);
    void setDefaultZone(int zone, std::error_code& ec);
    void setDefaultConfOpt(bool forced, std::error_code& ec);
    void adjustSpeedOnline(double scale, std::error_code& ec);
    // 运动指令
    void moveAbsJ(const rokae::MoveAbsJCommand& cmd, std::error_code& ec);
    void moveJ(const rokae::MoveJCommand& cmd, std::error_code& ec);
    void moveL(const rokae::MoveLCommand& cmd, std::error_code& ec);
    void moveC(const rokae::MoveCCommand& cmd, std::error_code& ec);
    void moveCF(const rokae::MoveCFCommand& cmd, std::error_code& ec);
    void moveSP(const rokae::MoveSPCommand& cmd, std::error_code& ec);

    // ==================== 4.5 实时控制/高级数据接口 ====================
    void setRtControlMode(rokae::RtControllerMode mode, std::error_code& ec);
    bool getRtJointData(std::array<double, 6>& position, std::array<double, 6>& velocity, std::array<double, 6>& torque, std::error_code& ec);
    std::string sendCustomData(const std::string& topic, const std::string& payload, std::error_code& ec);
    bool registerDataCallback(const std::string& data_topic, const std::string& callback_id, std::error_code& ec);

    // ==================== 4.6 IO与通信接口 ====================
    bool getDI(unsigned int board, unsigned int port, std::error_code& ec);
    bool getDO(unsigned int board, unsigned int port, std::error_code& ec);
    void setDI(unsigned int board, unsigned int port, bool state, std::error_code& ec);
    void setDO(unsigned int board, unsigned int port, bool state, std::error_code& ec);
    double getAI(unsigned int board, unsigned int port, std::error_code& ec);
    void setAO(unsigned int board, unsigned int port, double value, std::error_code& ec);
    void setSimulationMode(bool state, std::error_code& ec);
    std::string readRegister(const std::string& key, std::error_code& ec);
    void writeRegister(const std::string& key, const std::string& value, std::error_code& ec);

    // ==================== 4.7 RL工程接口 ====================
    bool loadRLProject(const std::string& project_path, std::string& project_name, std::error_code& ec);
    bool startRLProject(const std::string& project_id, int& current_episode, std::error_code& ec);
    bool stopRLProject(const std::string& project_id, int& finished_episode, std::error_code& ec);

    // ==================== 4.8 协作机器人专属接口 ====================
    void enableDrag(rokae::DragParameter::Space space, rokae::DragParameter::Type type, std::error_code& ec);
    void disableDrag(std::error_code& ec);
    void startJog(rokae::JogOpt::Space space, unsigned int index, bool direction, double step, std::error_code& ec);
    void setAvoidSingularity(bool enable, std::error_code& ec);
    bool getAvoidSingularity(std::error_code& ec);
    std::array<double, 6> getEndTorque(std::error_code& ec);
    bool calcJointTorque(const std::array<double, 6>& joint_pos,
                         const std::array<double, 6>& joint_vel,
                         const std::array<double, 6>& joint_acc,
                         const std::array<double, 6>& external_force,
                         std::array<double, 6>& joint_torque,
                         std::array<double, 6>& gravity_torque,
                         std::array<double, 6>& coriolis_torque,
                         std::error_code& ec);
    bool generateSTrajectory(const std::array<double, 6>& start_joint_pos,
                             const std::array<double, 6>& target_joint_pos,
                             std::vector<std::array<double, 6>>& trajectory_points,
                             double& total_time,
                             std::error_code& ec);
    bool mapCartesianToJointTorque(const std::array<double, 6>& cart_force,
                                   const std::array<double, 6>& joint_pos,
                                   std::array<double, 6>& joint_torque,
                                   std::error_code& ec);
    // 路径录制与回放
    void startRecordPath(std::chrono::seconds duration, std::error_code& ec);
    void stopRecordPath(std::error_code& ec);
    void cancelRecordPath(std::error_code& ec);
    void saveRecordPath(const std::string& name, std::error_code& ec, const std::string& saveAs = "");
    void replayPath(const std::string& name, double rate, std::error_code& ec);
    void removePath(const std::string& name, std::error_code& ec, bool removeAll = false);
    std::vector<std::string> queryPathLists(std::error_code& ec);

    // ==================== 扩展SDK兼容接口 ====================
    // 事件与回调
    void setEventWatcher(rokae::Event eventType, const rokae::EventCallback& callback, std::error_code& ec);
    rokae::EventInfo queryEventInfo(rokae::Event eventType, std::error_code& ec);

    // 运动指令批量执行 - 非模板版本，避免链接问题
    void moveAppend(const std::vector<rokae::MoveAbsJCommand>& cmds, std::string& cmdID, std::error_code& ec);
    void moveAppend(const std::vector<rokae::MoveJCommand>& cmds, std::string& cmdID, std::error_code& ec);
    void moveAppend(const std::vector<rokae::MoveLCommand>& cmds, std::string& cmdID, std::error_code& ec);
    void moveAppend(const std::vector<rokae::MoveCCommand>& cmds, std::string& cmdID, std::error_code& ec);
    void moveAppend(const std::vector<rokae::MoveCFCommand>& cmds, std::string& cmdID, std::error_code& ec);
    void moveAppend(const std::vector<rokae::MoveSPCommand>& cmds, std::string& cmdID, std::error_code& ec);

    void executeCommand(const std::vector<rokae::MoveAbsJCommand>& cmds, std::error_code& ec);
    void executeCommand(const std::vector<rokae::MoveJCommand>& cmds, std::error_code& ec);
    void executeCommand(const std::vector<rokae::MoveLCommand>& cmds, std::error_code& ec);

    // 缓存与速度调整
    void setMaxCacheSize(int number, std::error_code& ec);
    std::error_code lastErrorCode() noexcept;

    // 实时状态接收
    void startReceiveRobotState(std::chrono::steady_clock::duration interval, const std::vector<std::string>& fields);
    void stopReceiveRobotState() noexcept;
    unsigned updateRobotState(std::chrono::steady_clock::duration timeout);
    int getStateDataArray6(const std::string& fieldName, std::array<double, 6>& data);

    // 模型与实时控制器访问
    std::shared_ptr<void> model();
    std::weak_ptr<void> getRtMotionController();

    // 坐标系标定
    rokae::FrameCalibrationResult calibrateFrame(rokae::FrameType type,
                                                  const std::vector<std::array<double, 6>>& points,
                                                  bool is_held,
                                                  std::error_code& ec,
                                                  const std::array<double, 3>& base_aux = {});

    // 软限位SDK兼容版本
    bool getSoftLimit(std::array<double[2], 6>& limits, std::error_code& ec);
    void setSoftLimit(bool enable, std::error_code& ec, const std::array<double[2], 6>& limits = {{}});

private:
    // 私有实现类（PIMPL模式，与cpp实现完全对齐）
    class Impl {
    public:
        explicit Impl(const std::string& node_name);
        Impl(const std::string& remote_ip, const std::string& local_ip);
        ~Impl();

        // 通用工具函数：等待服务并处理错误
        bool wait_for_service(rclcpp::ClientBase::SharedPtr client, std::error_code& ec, int timeout_s = 5);
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

        // ROS2核心节点
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
        std::thread executor_thread_;
        // 机器人配置参数
        std::string remote_ip_ = "192.168.0.160"; // 珞石机器人默认IP
        std::string local_ip_ = "";
        // 连接状态
        bool connected_ = false;
        // 线程安全锁
        std::mutex state_mutex_;
        std::mutex ros_call_mutex_;

        // 最新状态缓存（线程安全）
        sensor_msgs::msg::JointState last_joint_state_;
        rokae_xmate3_ros2::msg::OperationState last_operation_state_;

        // 非实时运动命令缓存与动作客户端
        std::mutex cache_mutex_;  // 保护下面两项
        rokae_xmate3_ros2::action::MoveAppend::Goal cached_goal_;  // 单个合并的goal
        rclcpp_action::Client<rokae_xmate3_ros2::action::MoveAppend>::SharedPtr move_append_action_client_;
        std::vector<rclcpp_action::ClientGoalHandle<rokae_xmate3_ros2::action::MoveAppend>::SharedPtr> active_goal_handles_;
        std::mutex action_mutex_;
        bool move_append_result_ready_ = false;
        rclcpp_action::ResultCode move_append_result_code_ = rclcpp_action::ResultCode::UNKNOWN;
        bool move_append_result_success_ = true;
        std::string move_append_result_message_;

        // 缓存相关公用方法（外部由xMateRobot调用）
        void cacheCommand(const rokae_xmate3_ros2::action::MoveAppend::Goal &goal);
        void clearCache();
        void resetMoveAppendState();
        void handleMoveAppendResult(
            const rclcpp_action::ClientGoalHandle<rokae_xmate3_ros2::action::MoveAppend>::WrappedResult &result);
        bool checkMoveAppendFailure(std::error_code &ec);
        // 将本地缓存的命令通过 MoveAppend action 发送到控制器
        bool flushCachedCommands(std::error_code &ec);
        void pump_callbacks();

        // ==================== SDK兼容扩展字段 ====================
        std::error_code last_error_code_;
        int max_cache_size_ = 100;
        unsigned long next_cmd_id_ = 1;

        // 事件回调
        std::mutex event_mutex_;
        rokae::EventCallback move_event_watcher_;
        rokae::EventCallback safety_event_watcher_;
        rokae::EventInfo last_move_event_;
        rokae::EventInfo last_safety_event_;

        // 实时状态接收
        std::mutex state_cache_mutex_;
        std::vector<std::string> state_fields_;
        std::chrono::steady_clock::duration state_interval_{std::chrono::milliseconds(1)};
        std::unordered_map<std::string, std::any> state_cache_;
        bool has_previous_state_ = false;
        std::chrono::steady_clock::time_point previous_state_time_;
        std::array<double, 6> previous_joint_velocity_{};
        std::array<double, 6> previous_joint_torque_{};
        rokae::Toolset toolset_cache_;

        // 模型和实时控制器占位
        std::shared_ptr<void> rt_controller_;
        std::shared_ptr<void> model_;

        // RL工程信息（占位）
        std::vector<rokae::RLProjectInfo> projects_;
        rokae::RLProjectInfo current_project_;
        std::vector<rokae::WorkToolInfo> tools_;
        std::vector<rokae::WorkToolInfo> wobjs_;

        // xPanel设置
        rokae::xPanelOpt::Vout xpanel_vout_ = rokae::xPanelOpt::Vout::off;
        unsigned rt_network_tolerance_ = 10;
        bool use_rci_client_ = false;

        // ==================== 服务客户端（与cpp实现1:1匹配） ====================
        // 基础连接与信息
        rclcpp::Client<rokae_xmate3_ros2::srv::Connect>::SharedPtr xmate3_robot_connect_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::Disconnect>::SharedPtr xmate3_robot_disconnect_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::GetInfo>::SharedPtr xmate3_robot_get_info_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::GetPowerState>::SharedPtr xmate3_robot_get_power_state_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::SetPowerState>::SharedPtr xmate3_robot_set_power_state_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::GetOperateMode>::SharedPtr xmate3_robot_get_operate_mode_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::SetOperateMode>::SharedPtr xmate3_robot_set_operate_mode_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::QueryControllerLog>::SharedPtr xmate3_robot_query_log_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::ClearServoAlarm>::SharedPtr xmate3_robot_clear_servo_alarm_client_;

        // 关节与位姿
        rclcpp::Client<rokae_xmate3_ros2::srv::GetJointPos>::SharedPtr xmate3_robot_get_joint_pos_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::GetJointVel>::SharedPtr xmate3_robot_get_joint_vel_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::GetJointTorque>::SharedPtr xmate3_robot_get_joint_torque_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::GetPosture>::SharedPtr xmate3_robot_get_posture_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::GetCartPosture>::SharedPtr xmate3_robot_get_cart_posture_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::GetBaseFrame>::SharedPtr xmate3_robot_get_base_frame_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::CalcIk>::SharedPtr xmate3_robot_calc_ik_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::CalcFk>::SharedPtr xmate3_robot_calc_fk_client_;

        // 工具与坐标系
        rclcpp::Client<rokae_xmate3_ros2::srv::GetToolset>::SharedPtr xmate3_robot_get_toolset_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::SetToolset>::SharedPtr xmate3_robot_set_toolset_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::SetToolsetByName>::SharedPtr xmate3_robot_set_toolset_by_name_client_;

        // 安全相关
        rclcpp::Client<rokae_xmate3_ros2::srv::EnableCollisionDetection>::SharedPtr xmate3_robot_enable_collision_detection_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::DisableCollisionDetection>::SharedPtr xmate3_robot_disable_collision_detection_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::GetSoftLimit>::SharedPtr xmate3_robot_get_soft_limit_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::SetSoftLimit>::SharedPtr xmate3_robot_set_soft_limit_client_;

        // 非实时运动控制
        rclcpp::Client<rokae_xmate3_ros2::srv::SetMotionControlMode>::SharedPtr xmate3_motion_set_control_mode_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::MoveReset>::SharedPtr xmate3_motion_reset_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::MoveStart>::SharedPtr xmate3_motion_start_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::Stop>::SharedPtr xmate3_motion_stop_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::SetDefaultSpeed>::SharedPtr xmate3_motion_set_default_speed_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::SetDefaultZone>::SharedPtr xmate3_motion_set_default_zone_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::SetDefaultConfOpt>::SharedPtr xmate3_motion_set_default_conf_opt_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::AdjustSpeedOnline>::SharedPtr xmate3_motion_adjust_speed_online_client_;

        // 实时控制/高级数据
        rclcpp::Client<rokae_xmate3_ros2::srv::SetRtControlMode>::SharedPtr xmate3_rt_set_control_mode_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::GetRtJointData>::SharedPtr xmate3_rt_get_joint_data_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::SendCustomData>::SharedPtr xmate3_comm_send_custom_data_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::RegisterDataCallback>::SharedPtr xmate3_comm_register_data_callback_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::ReadRegister>::SharedPtr xmate3_comm_read_register_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::WriteRegister>::SharedPtr xmate3_comm_write_register_client_;

        // IO控制
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
        rclcpp::Client<rokae_xmate3_ros2::srv::SetAvoidSingularity>::SharedPtr xmate3_cobot_set_avoid_singularity_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::GetAvoidSingularity>::SharedPtr xmate3_cobot_get_avoid_singularity_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::GetEndTorque>::SharedPtr xmate3_cobot_get_end_torque_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::CalcJointTorque>::SharedPtr xmate3_dyn_calc_joint_torque_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::GenerateSTrajectory>::SharedPtr xmate3_dyn_generate_s_trajectory_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::MapCartesianToJointTorque>::SharedPtr xmate3_dyn_map_cartesian_to_joint_torque_client_;

        // 拖动与路径录制
        rclcpp::Client<rokae_xmate3_ros2::srv::EnableDrag>::SharedPtr xmate3_cobot_enable_drag_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::DisableDrag>::SharedPtr xmate3_cobot_disable_drag_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::StartRecordPath>::SharedPtr xmate3_cobot_start_record_path_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::StopRecordPath>::SharedPtr xmate3_cobot_stop_record_path_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::CancelRecordPath>::SharedPtr xmate3_cobot_cancel_record_path_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::SaveRecordPath>::SharedPtr xmate3_cobot_save_record_path_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::ReplayPath>::SharedPtr xmate3_cobot_replay_path_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::RemovePath>::SharedPtr xmate3_cobot_remove_path_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::QueryPathLists>::SharedPtr xmate3_cobot_query_path_lists_client_;

        // 话题订阅器
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Subscription<rokae_xmate3_ros2::msg::OperationState>::SharedPtr operation_state_sub_;

    private:
        // 初始化函数
        void init_node();
        void init_clients();
        void init_subscribers();
    };

    // PIMPL实例
    std::shared_ptr<Impl> impl_;
};

} // namespace rokae::ros2

#endif // ROKAE_XMATE3_ROS2_ROBOT_H
