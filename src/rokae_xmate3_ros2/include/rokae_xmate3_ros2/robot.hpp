#ifndef ROKAE_XMATE3_ROS2_ROBOT_H
#define ROKAE_XMATE3_ROS2_ROBOT_H

// 核心依赖头文件
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <array>
#include <vector>
#include <memory>
#include <functional>
#include <chrono>
#include <system_error>
#include <cstdint>
#include <mutex>

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

    // ==================== 4.6 IO与通信接口 ====================
    bool getDI(unsigned int board, unsigned int port, std::error_code& ec);
    bool getDO(unsigned int board, unsigned int port, std::error_code& ec);
    void setDI(unsigned int board, unsigned int port, bool state, std::error_code& ec);
    void setDO(unsigned int board, unsigned int port, bool state, std::error_code& ec);
    double getAI(unsigned int board, unsigned int port, std::error_code& ec);
    void setAO(unsigned int board, unsigned int port, double value, std::error_code& ec);
    void setSimulationMode(bool state, std::error_code& ec);

    // ==================== 4.8 协作机器人专属接口 ====================
    void enableDrag(rokae::DragParameter::Space space, rokae::DragParameter::Type type, std::error_code& ec);
    void disableDrag(std::error_code& ec);
    // 路径录制与回放
    void startRecordPath(std::chrono::seconds duration, std::error_code& ec);
    void stopRecordPath(std::error_code& ec);
    void cancelRecordPath(std::error_code& ec);
    void saveRecordPath(const std::string& name, std::error_code& ec, const std::string& saveAs = "");
    void replayPath(const std::string& name, double rate, std::error_code& ec);
    void removePath(const std::string& name, std::error_code& ec, bool removeAll = false);
    std::vector<std::string> queryPathLists(std::error_code& ec);

private:
    // 私有实现类（PIMPL模式，与cpp实现完全对齐）
    class Impl {
    public:
        explicit Impl(const std::string& node_name);
        Impl(const std::string& remote_ip, const std::string& local_ip);
        ~Impl() = default;

        // 通用工具函数：等待服务并处理错误
        bool wait_for_service(rclcpp::ClientBase::SharedPtr client, std::error_code& ec, int timeout_s = 5);

        // ROS2核心节点
        rclcpp::Node::SharedPtr node_;
        // 机器人配置参数
        std::string remote_ip_ = "192.168.0.160"; // 珞石机器人默认IP
        std::string local_ip_ = "";
        // 连接状态
        bool connected_ = false;
        // 线程安全锁
        std::mutex state_mutex_;

        // 最新状态缓存（线程安全）
        sensor_msgs::msg::JointState last_joint_state_;
        rokae_xmate3_ros2::msg::OperationState last_operation_state_;

        // 非实时运动命令缓存与动作客户端
        std::mutex cache_mutex_;  // 保护下面两项
        rokae_xmate3_ros2::action::MoveAppend::Goal cached_goal_;  // 单个合并的goal
        rclcpp_action::Client<rokae_xmate3_ros2::action::MoveAppend>::SharedPtr move_append_action_client_;
        std::vector<rclcpp_action::ClientGoalHandle<rokae_xmate3_ros2::action::MoveAppend>::SharedPtr> active_goal_handles_;

        // 缓存相关公用方法（外部由xMateRobot调用）
        void cacheCommand(const rokae_xmate3_ros2::action::MoveAppend::Goal &goal);
        void clearCache();
        // 将本地缓存的命令通过 MoveAppend action 发送到控制器
        bool flushCachedCommands(std::error_code &ec);

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

        // IO控制
        rclcpp::Client<rokae_xmate3_ros2::srv::GetDI>::SharedPtr xmate3_io_get_di_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::GetDO>::SharedPtr xmate3_io_get_do_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::SetDI>::SharedPtr xmate3_io_set_di_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::SetDO>::SharedPtr xmate3_io_set_do_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::GetAI>::SharedPtr xmate3_io_get_ai_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::SetAO>::SharedPtr xmate3_io_set_ao_client_;
        rclcpp::Client<rokae_xmate3_ros2::srv::SetSimulationMode>::SharedPtr xmate3_io_set_simulation_mode_client_;

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
