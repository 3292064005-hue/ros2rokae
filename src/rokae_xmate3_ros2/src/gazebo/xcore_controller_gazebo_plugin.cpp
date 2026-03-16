/**
 * @file xcore_controller_gazebo_plugin.cpp
 * @brief xCore SDK Gazebo 仿真控制器插件
 *
 * 这个插件直接控制Gazebo关节来实现xCore SDK功能，
 * 同时配合ros2_control的joint_state_broadcaster提供标准ROS2接口。
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>
#include <queue>
#include <memory>
#include <map>
#include <vector>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <limits>

#include <builtin_interfaces/msg/time.hpp>

#include "rokae_xmate3_ros2/gazebo/motion_types.hpp"
#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "rokae_xmate3_ros2/gazebo/trajectory_planner.hpp"

// ROS2 接口头文件
#include "rokae_xmate3_ros2/msg/operation_state.hpp"
#include "rokae_xmate3_ros2/msg/power_state.hpp"
#include "rokae_xmate3_ros2/msg/operate_mode.hpp"
#include "rokae_xmate3_ros2/msg/info.hpp"
#include "rokae_xmate3_ros2/msg/move_cf_command.hpp"
#include "rokae_xmate3_ros2/msg/move_sp_command.hpp"
#include "rokae_xmate3_ros2/srv/connect.hpp"
#include "rokae_xmate3_ros2/srv/disconnect.hpp"
#include "rokae_xmate3_ros2/srv/get_power_state.hpp"
#include "rokae_xmate3_ros2/srv/set_power_state.hpp"
#include "rokae_xmate3_ros2/srv/get_joint_pos.hpp"
#include "rokae_xmate3_ros2/srv/get_joint_vel.hpp"
#include "rokae_xmate3_ros2/srv/get_joint_torque.hpp"
#include "rokae_xmate3_ros2/srv/get_cart_posture.hpp"
#include "rokae_xmate3_ros2/srv/get_posture.hpp"
#include "rokae_xmate3_ros2/srv/get_base_frame.hpp"
#include "rokae_xmate3_ros2/srv/calc_ik.hpp"
#include "rokae_xmate3_ros2/srv/calc_fk.hpp"
#include "rokae_xmate3_ros2/srv/set_motion_control_mode.hpp"
#include "rokae_xmate3_ros2/srv/move_reset.hpp"
#include "rokae_xmate3_ros2/srv/move_start.hpp"
#include "rokae_xmate3_ros2/srv/stop.hpp"
#include "rokae_xmate3_ros2/srv/set_default_speed.hpp"
#include "rokae_xmate3_ros2/srv/set_default_zone.hpp"
#include "rokae_xmate3_ros2/srv/enable_drag.hpp"
#include "rokae_xmate3_ros2/srv/disable_drag.hpp"
#include "rokae_xmate3_ros2/srv/set_simulation_mode.hpp"
#include "rokae_xmate3_ros2/srv/get_operate_mode.hpp"
#include "rokae_xmate3_ros2/srv/set_operate_mode.hpp"
#include "rokae_xmate3_ros2/srv/get_info.hpp"
#include "rokae_xmate3_ros2/srv/get_toolset.hpp"
#include "rokae_xmate3_ros2/srv/set_toolset.hpp"
#include "rokae_xmate3_ros2/srv/set_toolset_by_name.hpp"
#include "rokae_xmate3_ros2/srv/get_di.hpp"
#include "rokae_xmate3_ros2/srv/get_do.hpp"
#include "rokae_xmate3_ros2/srv/set_di.hpp"
#include "rokae_xmate3_ros2/srv/set_do.hpp"
#include "rokae_xmate3_ros2/srv/get_ai.hpp"
#include "rokae_xmate3_ros2/srv/set_ao.hpp"
#include "rokae_xmate3_ros2/srv/enable_collision_detection.hpp"
#include "rokae_xmate3_ros2/srv/disable_collision_detection.hpp"
#include "rokae_xmate3_ros2/srv/get_soft_limit.hpp"
#include "rokae_xmate3_ros2/srv/set_soft_limit.hpp"
#include "rokae_xmate3_ros2/srv/clear_servo_alarm.hpp"
#include "rokae_xmate3_ros2/srv/query_controller_log.hpp"
#include "rokae_xmate3_ros2/srv/set_default_conf_opt.hpp"
#include "rokae_xmate3_ros2/srv/adjust_speed_online.hpp"
#include "rokae_xmate3_ros2/srv/start_record_path.hpp"
#include "rokae_xmate3_ros2/srv/stop_record_path.hpp"
#include "rokae_xmate3_ros2/srv/cancel_record_path.hpp"
#include "rokae_xmate3_ros2/srv/save_record_path.hpp"
#include "rokae_xmate3_ros2/srv/replay_path.hpp"
#include "rokae_xmate3_ros2/srv/remove_path.hpp"
#include "rokae_xmate3_ros2/srv/query_path_lists.hpp"
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gazebo {

static builtin_interfaces::msg::Time ToBuiltinTime(const rclcpp::Time &time) {
    builtin_interfaces::msg::Time stamp;
    const auto ns = time.nanoseconds();
    stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    return stamp;
}

namespace {

constexpr int kOffsetNone = 0;
constexpr int kOffsetOffs = 1;
constexpr int kOffsetRelTool = 2;
constexpr double kConfAngleStrictToleranceDeg = 45.0;
constexpr double kJointBranchJumpThreshold = 0.75;
constexpr double kTrajectorySampleDt = 0.001;

double NormalizeAngle(double value) {
    while (value > M_PI) {
        value -= 2.0 * M_PI;
    }
    while (value < -M_PI) {
        value += 2.0 * M_PI;
    }
    return value;
}

std::vector<double> MsgPoseToVector(const rokae_xmate3_ros2::msg::CartesianPosition &pose) {
    return {pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz};
}

std::vector<double> ArrayPoseToVector(const std::array<double, 6> &pose) {
    return {pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]};
}

Eigen::Matrix4d PoseToTransform(const std::vector<double> &pose) {
    Eigen::Matrix3d rx;
    Eigen::Matrix3d ry;
    Eigen::Matrix3d rz;
    rx << 1, 0, 0,
          0, std::cos(pose[3]), -std::sin(pose[3]),
          0, std::sin(pose[3]), std::cos(pose[3]);
    ry << std::cos(pose[4]), 0, std::sin(pose[4]),
          0, 1, 0,
          -std::sin(pose[4]), 0, std::cos(pose[4]);
    rz << std::cos(pose[5]), -std::sin(pose[5]), 0,
          std::sin(pose[5]), std::cos(pose[5]), 0,
          0, 0, 1;

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rz * ry * rx;
    transform(0, 3) = pose[0];
    transform(1, 3) = pose[1];
    transform(2, 3) = pose[2];
    return transform;
}

std::vector<double> TransformToPose(const Eigen::Matrix4d &transform) {
    std::vector<double> pose(6, 0.0);
    pose[0] = transform(0, 3);
    pose[1] = transform(1, 3);
    pose[2] = transform(2, 3);
    pose[4] = std::atan2(-transform(2, 0),
                         std::sqrt(transform(0, 0) * transform(0, 0) +
                                   transform(1, 0) * transform(1, 0)));
    if (std::fabs(pose[4] - M_PI / 2.0) < 1e-3) {
        pose[5] = 0.0;
        pose[3] = std::atan2(transform(1, 2), transform(1, 1));
    } else if (std::fabs(pose[4] + M_PI / 2.0) < 1e-3) {
        pose[5] = 0.0;
        pose[3] = std::atan2(-transform(1, 2), transform(1, 1));
    } else {
        pose[5] = std::atan2(transform(1, 0), transform(0, 0));
        pose[3] = std::atan2(transform(2, 1), transform(2, 2));
    }
    pose[3] = NormalizeAngle(pose[3]);
    pose[4] = NormalizeAngle(pose[4]);
    pose[5] = NormalizeAngle(pose[5]);
    return pose;
}

std::vector<double> ApplyOffset(const std::vector<double> &pose, int offset_type, const std::array<double, 6> &offset_pose) {
    if (offset_type == kOffsetNone) {
        return pose;
    }
    const auto offset = ArrayPoseToVector(offset_pose);
    if (offset_type == kOffsetOffs) {
        std::vector<double> result = pose;
        for (size_t i = 0; i < result.size(); ++i) {
            result[i] += offset[i];
        }
        return result;
    }
    return TransformToPose(PoseToTransform(pose) * PoseToTransform(offset));
}

double NormalizeDegree(double value) {
    while (value > 180.0) {
        value -= 360.0;
    }
    while (value < -180.0) {
        value += 360.0;
    }
    return value;
}

double ShortestDegreeDistance(double lhs, double rhs) {
    return std::fabs(NormalizeDegree(lhs - rhs));
}

double MaxJointStep(const std::vector<double> &lhs, const std::vector<double> &rhs) {
    double max_step = 0.0;
    for (size_t i = 0; i < 6 && i < lhs.size() && i < rhs.size(); ++i) {
        max_step = std::max(max_step, std::fabs(lhs[i] - rhs[i]));
    }
    return max_step;
}

struct TrajectoryTrackingSample {
    std::vector<double> position;
    std::vector<double> velocity;
    bool finished = false;
};

TrajectoryTrackingSample SampleJointTrajectory(
    const std::vector<std::vector<double>> &trajectory,
    double elapsed_time,
    double sample_dt) {

    TrajectoryTrackingSample sample;
    if (trajectory.empty()) {
        sample.finished = true;
        return sample;
    }

    sample.position = trajectory.back();
    sample.velocity.assign(sample.position.size(), 0.0);

    if (trajectory.size() == 1 || sample_dt <= 1e-9) {
        sample.finished = true;
        return sample;
    }

    const double max_time = static_cast<double>(trajectory.size() - 1) * sample_dt;
    const double clamped_time = std::clamp(elapsed_time, 0.0, max_time);
    const double sample_index = clamped_time / sample_dt;
    const size_t index = static_cast<size_t>(std::floor(sample_index));
    if (index >= trajectory.size() - 1) {
        sample.finished = clamped_time >= max_time;
        return sample;
    }

    const double alpha = sample_index - static_cast<double>(index);
    sample.position.resize(trajectory[index].size(), 0.0);
    sample.velocity.resize(trajectory[index].size(), 0.0);
    for (size_t i = 0; i < trajectory[index].size(); ++i) {
        const double p0 = trajectory[index][i];
        const double p1 = trajectory[index + 1][i];
        sample.position[i] = p0 + alpha * (p1 - p0);
        sample.velocity[i] = (p1 - p0) / sample_dt;
    }
    sample.finished = clamped_time >= max_time;
    return sample;
}

struct ConfScore {
    double penalty = 0.0;
    bool strict_match = true;
    bool has_request = false;
};

ConfScore ScoreRequestedConf(const std::vector<double> &candidate, const std::vector<int> &requested_conf) {
    ConfScore score;
    for (size_t i = 0; i < 6 && i < candidate.size() && i < requested_conf.size(); ++i) {
        const int requested = requested_conf[i];
        if (requested == 0) {
            continue;
        }

        score.has_request = true;
        if (std::abs(requested) <= 2) {
            const int sign = std::fabs(candidate[i]) < 1e-6 ? 0 : (candidate[i] > 0.0 ? 1 : -1);
            if (sign != requested) {
                score.penalty += 180.0;
                score.strict_match = false;
            }
            continue;
        }

        const double candidate_deg = candidate[i] * 180.0 / M_PI;
        const double delta_deg = ShortestDegreeDistance(candidate_deg, static_cast<double>(requested));
        score.penalty += delta_deg;
        if (delta_deg > kConfAngleStrictToleranceDeg) {
            score.strict_match = false;
        }
    }
    return score;
}

bool ViolatesSoftLimit(const std::vector<double> &joints,
                       const std::array<std::array<double, 2>, 6> &soft_limits) {
    for (size_t i = 0; i < 6 && i < joints.size(); ++i) {
        if (joints[i] < soft_limits[i][0] || joints[i] > soft_limits[i][1]) {
            return true;
        }
    }
    return false;
}

double CartesianErrorScore(const std::vector<double> &expected, const std::vector<double> &actual) {
    const double pos_error = std::sqrt(
        std::pow(actual[0] - expected[0], 2) +
        std::pow(actual[1] - expected[1], 2) +
        std::pow(actual[2] - expected[2], 2));
    const double ori_error =
        std::fabs(NormalizeAngle(actual[3] - expected[3])) +
        std::fabs(NormalizeAngle(actual[4] - expected[4])) +
        std::fabs(NormalizeAngle(actual[5] - expected[5]));
    return pos_error * 1000.0 + ori_error * 100.0;
}

struct IkSelection {
    bool success = false;
    std::vector<double> joints;
    std::string message;
};

IkSelection SelectIkSolution(
    xMate3Kinematics &kinematics,
    const std::vector<std::vector<double>> &candidates,
    const std::vector<double> &target_pose,
    const std::vector<double> &seed_joints,
    const std::vector<int> &requested_conf,
    bool strict_conf,
    bool avoid_singularity,
    bool soft_limit_enabled,
    const std::array<std::array<double, 2>, 6> &soft_limits) {

    IkSelection best;
    IkSelection fallback_singular;
    double best_score = std::numeric_limits<double>::infinity();
    double fallback_singular_score = std::numeric_limits<double>::infinity();

    for (const auto &candidate : candidates) {
        if (candidate.size() < 6) {
            continue;
        }
        if (soft_limit_enabled && ViolatesSoftLimit(candidate, soft_limits)) {
            continue;
        }

        const auto fk_pose = kinematics.forwardKinematicsRPY(candidate);
        const auto conf_score = ScoreRequestedConf(candidate, requested_conf);
        if (conf_score.has_request && strict_conf && !conf_score.strict_match) {
            continue;
        }

        double joint_distance = 0.0;
        for (size_t i = 0; i < 6 && i < seed_joints.size(); ++i) {
            joint_distance += std::fabs(candidate[i] - seed_joints[i]);
        }
        const bool near_singularity = kinematics.isNearSingularity(candidate);
        const double singularity_measure = kinematics.computeSingularityMeasure(candidate);
        const double score =
            CartesianErrorScore(target_pose, fk_pose) +
            joint_distance * 2.0 +
            conf_score.penalty +
            singularity_measure * 40.0;

        if (avoid_singularity && near_singularity) {
            if (score < fallback_singular_score) {
                fallback_singular.success = true;
                fallback_singular.joints = candidate;
                fallback_singular_score = score;
            }
            continue;
        }

        if (score < best_score) {
            best.success = true;
            best.joints = candidate;
            best_score = score;
        }
    }

    if (!best.success && fallback_singular.success) {
        fallback_singular.message = "selected a near-singular IK branch because no better branch was available";
        return fallback_singular;
    }

    if (!best.success) {
        best.message = (!requested_conf.empty() && strict_conf)
                           ? "no IK solution matches requested confData"
                           : "no valid IK solution";
    }
    return best;
}

bool BuildJointTrajectoryFromCartesian(
    xMate3Kinematics &kinematics,
    const std::vector<std::vector<double>> &cartesian_trajectory,
    const std::vector<double> &initial_seed,
    const std::vector<int> &requested_conf,
    bool strict_conf,
    bool avoid_singularity,
    bool soft_limit_enabled,
    const std::array<std::array<double, 2>, 6> &soft_limits,
    std::vector<std::vector<double>> &joint_trajectory,
    std::vector<double> &last_joints,
    std::string &error_message) {

    joint_trajectory.clear();
    last_joints = initial_seed;

    for (const auto &pose : cartesian_trajectory) {
        const auto candidates = kinematics.inverseKinematicsMultiSolution(pose, last_joints);
        const auto selected = SelectIkSolution(
            kinematics, candidates, pose, last_joints, requested_conf, strict_conf,
            avoid_singularity, soft_limit_enabled, soft_limits);
        if (!selected.success) {
            error_message = selected.message;
            return false;
        }
        if (!joint_trajectory.empty() &&
            MaxJointStep(last_joints, selected.joints) > kJointBranchJumpThreshold) {
            error_message = "IK branch changed discontinuously along Cartesian path";
            return false;
        }
        joint_trajectory.push_back(selected.joints);
        last_joints = selected.joints;
    }

    return !joint_trajectory.empty();
}

}  // namespace

class XCoreControllerPlugin : public ModelPlugin {
public:
    XCoreControllerPlugin() : ModelPlugin() {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
    void InitROS2();
    void InitPublishers();
    void InitServices();
    void InitActionServers();
    void OnUpdate();
    void ExecuteMotion();
    void PublishJointStates();
    void PublishOperationState();
    void ExecuteMoveAppend(const std::shared_ptr<rclcpp_action::ServerGoalHandle<rokae_xmate3_ros2::action::MoveAppend>> goal_handle);

    // Gazebo 对象 - 直接控制关节
    physics::ModelPtr model_;
    sdf::ElementPtr sdf_;
    std::vector<physics::JointPtr> joints_;
    std::vector<std::string> joint_names_;
    int joint_num_ = 0;
    event::ConnectionPtr update_conn_;

    // ROS2 对象
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<xMate3Kinematics> kinematics_;

    // 状态变量
    bool connected_ = false;
    bool power_on_ = false;
    bool motion_running_ = false;
    bool drag_mode_ = false;
    bool simulation_mode_ = true;
    bool collision_detection_enabled_ = false;
    bool soft_limit_enabled_ = false;
    bool hold_position_initialized_ = false;
    bool joint_positions_initialized_ = false;
    std::string remote_ip_;
    int motion_mode_ = 0;
    rokae_xmate3_ros2::msg::OperateMode operate_mode_;
    int default_speed_ = 50;
    int default_zone_ = 0;
    double speed_scale_ = 1.0;
    bool default_conf_opt_forced_ = false;
    bool avoid_singularity_enabled_ = true;
    int rt_control_mode_ = -1;
    bool rl_project_loaded_ = false;
    bool rl_project_running_ = false;
    int rl_current_episode_ = 0;
    std::string loaded_rl_project_name_;
    std::string loaded_rl_project_path_;
    std::map<std::string, std::string> register_bank_;
    std::map<std::string, std::string> custom_data_bank_;
    std::map<std::string, std::string> callback_registry_;

    // ========== 【新增】初始姿态控制 ==========
    bool initial_pose_set_ = false; // 初始姿态是否已设置完成
    bool is_joint_locked_ = false;  // 关节是否已抱闸锁定
    // 保存URDF原始关节限位
    std::array<std::pair<double, double>, 6> original_joint_limits_;
    // 6关节初始角度（弧度），你可以直接在这里修改成目标初始姿态
    const std::vector<double> default_initial_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // IO状态模拟
    std::map<std::pair<unsigned int, unsigned int>, bool> di_state_;
    std::map<std::pair<unsigned int, unsigned int>, bool> do_state_;
    std::map<std::pair<unsigned int, unsigned int>, double> ai_state_;
    std::map<std::pair<unsigned int, unsigned int>, double> ao_state_;

    // 工具坐标系
    std::string current_tool_name_;
    std::string current_wobj_name_;
    std::vector<double> current_tool_pose_;
    std::vector<double> current_wobj_pose_;

    // 软限位
    std::array<std::array<double, 2>, 6> soft_limits_;

    // 日志缓存
    std::vector<rokae_xmate3_ros2::msg::LogInfo> log_buffer_;

    // 路径录制
    bool is_recording_path_ = false;
    std::vector<std::vector<double>> recorded_path_;
    std::map<std::string, std::vector<std::vector<double>>> saved_paths_;

    std::queue<MotionCommand> motion_queue_;
    std::mutex motion_queue_mutex_;
    std::unique_ptr<MotionCommand> current_cmd_;
    std::vector<double> hold_position_;
    double current_update_dt_ = kTrajectorySampleDt;
    double last_sim_time_ = -1.0;

    // ROS2 发布器
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<rokae_xmate3_ros2::msg::OperationState>::SharedPtr operation_state_pub_;

    // ==================== 所有服务端声明 ====================
    // 基础连接与信息
    rclcpp::Service<rokae_xmate3_ros2::srv::Connect>::SharedPtr connect_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::Disconnect>::SharedPtr disconnect_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::GetInfo>::SharedPtr get_info_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::GetPowerState>::SharedPtr get_power_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::SetPowerState>::SharedPtr set_power_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::GetOperateMode>::SharedPtr get_operate_mode_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::SetOperateMode>::SharedPtr set_operate_mode_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::QueryControllerLog>::SharedPtr query_log_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::ClearServoAlarm>::SharedPtr clear_servo_alarm_srv_;

    // 关节与位姿
    rclcpp::Service<rokae_xmate3_ros2::srv::GetJointPos>::SharedPtr get_joint_pos_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::GetJointVel>::SharedPtr get_joint_vel_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::GetJointTorque>::SharedPtr get_joint_torque_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::GetPosture>::SharedPtr get_posture_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::GetCartPosture>::SharedPtr get_cart_posture_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::GetBaseFrame>::SharedPtr get_base_frame_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::CalcIk>::SharedPtr calc_ik_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::CalcFk>::SharedPtr calc_fk_srv_;

    // 工具与坐标系
    rclcpp::Service<rokae_xmate3_ros2::srv::GetToolset>::SharedPtr get_toolset_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::SetToolset>::SharedPtr set_toolset_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::SetToolsetByName>::SharedPtr set_toolset_by_name_srv_;

    // 安全相关
    rclcpp::Service<rokae_xmate3_ros2::srv::EnableCollisionDetection>::SharedPtr enable_collision_detection_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::DisableCollisionDetection>::SharedPtr disable_collision_detection_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::GetSoftLimit>::SharedPtr get_soft_limit_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::SetSoftLimit>::SharedPtr set_soft_limit_srv_;

    // 非实时运动控制
    rclcpp::Service<rokae_xmate3_ros2::srv::SetMotionControlMode>::SharedPtr set_motion_control_mode_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::MoveReset>::SharedPtr move_reset_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::MoveStart>::SharedPtr move_start_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::Stop>::SharedPtr stop_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::SetDefaultSpeed>::SharedPtr set_default_speed_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::SetDefaultZone>::SharedPtr set_default_zone_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::SetDefaultConfOpt>::SharedPtr set_default_conf_opt_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::AdjustSpeedOnline>::SharedPtr adjust_speed_online_srv_;

    // 实时控制/高级数据/扩展兼容
    rclcpp::Service<rokae_xmate3_ros2::srv::SetRtControlMode>::SharedPtr set_rt_control_mode_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::GetRtJointData>::SharedPtr get_rt_joint_data_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::SendCustomData>::SharedPtr send_custom_data_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::RegisterDataCallback>::SharedPtr register_data_callback_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::ReadRegister>::SharedPtr read_register_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::WriteRegister>::SharedPtr write_register_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::LoadRLProject>::SharedPtr load_rl_project_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::StartRLProject>::SharedPtr start_rl_project_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::StopRLProject>::SharedPtr stop_rl_project_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::SetAvoidSingularity>::SharedPtr set_avoid_singularity_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::GetAvoidSingularity>::SharedPtr get_avoid_singularity_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::GetEndTorque>::SharedPtr get_end_torque_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::CalcJointTorque>::SharedPtr calc_joint_torque_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::GenerateSTrajectory>::SharedPtr generate_s_trajectory_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::MapCartesianToJointTorque>::SharedPtr map_cartesian_to_joint_torque_srv_;

    // IO控制
    rclcpp::Service<rokae_xmate3_ros2::srv::GetDI>::SharedPtr get_di_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::GetDO>::SharedPtr get_do_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::SetDI>::SharedPtr set_di_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::SetDO>::SharedPtr set_do_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::GetAI>::SharedPtr get_ai_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::SetAO>::SharedPtr set_ao_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::SetSimulationMode>::SharedPtr set_simulation_mode_srv_;

    // 拖动与路径录制
    rclcpp::Service<rokae_xmate3_ros2::srv::EnableDrag>::SharedPtr enable_drag_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::DisableDrag>::SharedPtr disable_drag_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::StartRecordPath>::SharedPtr start_record_path_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::StopRecordPath>::SharedPtr stop_record_path_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::CancelRecordPath>::SharedPtr cancel_record_path_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::SaveRecordPath>::SharedPtr save_record_path_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::ReplayPath>::SharedPtr replay_path_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::RemovePath>::SharedPtr remove_path_srv_;
    rclcpp::Service<rokae_xmate3_ros2::srv::QueryPathLists>::SharedPtr query_path_lists_srv_;

    // ROS2 Action Server
    rclcpp_action::Server<rokae_xmate3_ros2::action::MoveAppend>::SharedPtr move_append_action_server_;

    // 兼容性别名服务（例如 /xmate3/cobot/get_di -> /xmate3/io/get_di）
    std::vector<rclcpp::ServiceBase::SharedPtr> compatibility_services_;
};

void XCoreControllerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    gzmsg << "[xCore Controller] Loading plugin for model: " << _model->GetName() << std::endl;

    model_ = _model;
    sdf_ = _sdf;
    operate_mode_.mode = rokae_xmate3_ros2::msg::OperateMode::UNKNOWN;

    // 初始化工具坐标系默认值
    current_tool_name_ = "";
    current_wobj_name_ = "";
    current_tool_pose_.resize(16, 0.0);
    current_tool_pose_[0] = 1.0;
    current_tool_pose_[5] = 1.0;
    current_tool_pose_[10] = 1.0;
    current_tool_pose_[15] = 1.0;
    current_wobj_pose_.resize(16, 0.0);
    current_wobj_pose_[0] = 1.0;
    current_wobj_pose_[5] = 1.0;
    current_wobj_pose_[10] = 1.0;
    current_wobj_pose_[15] = 1.0;

    // 初始化软限位默认值
    for (int i = 0; i < 6; ++i) {
        soft_limits_[i][0] = -3.14;
        soft_limits_[i][1] = 3.14;
    }

    // 获取关节 - 直接用于控制
    joint_names_ = {"xmate_joint_1", "xmate_joint_2", "xmate_joint_3",
                    "xmate_joint_4", "xmate_joint_5", "xmate_joint_6"};
    for (const auto& name : joint_names_) {
        auto joint = _model->GetJoint(name);
        if (joint) {
            joints_.push_back(joint);
        } else {
            gzerr << "[xCore] Joint not found: " << name << std::endl;
        }
    }
    joint_num_ = joints_.size();
    hold_position_.resize(6, 0.0);

    // 保存URDF原始关节限位
    const std::array<std::pair<double, double>, 6> default_limits = {
        std::make_pair(-3.0527, 3.0527), // joint1
        std::make_pair(-2.0933, 2.0933), // joint2
        std::make_pair(-2.0933, 2.0933), // joint3
        std::make_pair(-3.0527, 3.0527), // joint4
        std::make_pair(-2.0933, 2.0933), // joint5
        std::make_pair(-6.1082, 6.1082)  // joint6
    };
    for (int i = 0; i < 6 && i < joint_num_; ++i) {
        original_joint_limits_[i] = default_limits[i];
    }

    // ========== 删除原来这里的初始位置设置代码 ==========
    joint_positions_initialized_ = false;
    hold_position_initialized_ = false;

    InitROS2();

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&XCoreControllerPlugin::OnUpdate, this));

    gzmsg << "[xCore Controller] Plugin loaded successfully, joints: " << joint_num_ << std::endl;
}

void XCoreControllerPlugin::InitROS2() {
    if (!rclcpp::ok()) {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>("xcore_gazebo_controller");
    kinematics_ = std::make_unique<xMate3Kinematics>();

    InitPublishers();
    InitServices();
    InitActionServers();
}

void XCoreControllerPlugin::InitPublishers() {
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/xmate3/joint_states", 10);
    operation_state_pub_ = node_->create_publisher<rokae_xmate3_ros2::msg::OperationState>("/xmate3/cobot/operation_state", 10);
}

void XCoreControllerPlugin::InitServices() {
    // ==================== 基础连接与信息服务 ====================
    connect_srv_ = node_->create_service<rokae_xmate3_ros2::srv::Connect>(
        "/xmate3/cobot/connect",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::Connect::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::Connect::Response> res) {
            this->connected_ = true;
            this->remote_ip_ = req->remote_ip;
            res->success = true;
            res->message = "Connected to xCore Gazebo Controller";
        });

    disconnect_srv_ = node_->create_service<rokae_xmate3_ros2::srv::Disconnect>(
        "/xmate3/cobot/disconnect",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::Disconnect::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::Disconnect::Response> res) {
            (void)req;
            this->connected_ = false;
            this->power_on_ = false;
            res->success = true;
        });

    set_power_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SetPowerState>(
        "/xmate3/cobot/set_power_state",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetPowerState::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetPowerState::Response> res) {
            if (!connected_) {
                res->success = false;
                res->message = "Robot not connected";
                return;
            }
            this->power_on_ = req->on;
            if (this->power_on_) {
                for (int i = 0; i < 6 && i < joint_num_; ++i) {
                    hold_position_[i] = joints_[i]->Position(0);
                }
            }
            res->success = true;
        });

    get_power_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetPowerState>(
        "/xmate3/cobot/get_power_state",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetPowerState::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetPowerState::Response> res) {
            (void)req;
            res->state.state = power_on_ ? rokae_xmate3_ros2::msg::PowerState::ON : rokae_xmate3_ros2::msg::PowerState::OFF;
            res->success = true;
        });

    get_info_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetInfo>(
        "/xmate3/cobot/get_info",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetInfo::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetInfo::Response> res) {
            (void)req;
            res->model = "xMate3";
            res->robot_type = "3";
            res->serial_number = "SIMULATION";
            res->firmware_version = "2.1.0";
            res->sdk_version = "2.1.0";
            res->joint_num = 6;
            res->success = true;
        });

    get_operate_mode_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetOperateMode>(
        "/xmate3/cobot/get_operate_mode",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetOperateMode::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetOperateMode::Response> res) {
            (void)req;
            res->mode = operate_mode_;
            res->success = true;
        });

    set_operate_mode_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SetOperateMode>(
        "/xmate3/cobot/set_operate_mode",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetOperateMode::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetOperateMode::Response> res) {
            operate_mode_.mode = req->mode;
            res->success = true;
        });

    query_log_srv_ = node_->create_service<rokae_xmate3_ros2::srv::QueryControllerLog>(
        "/xmate3/cobot/query_controller_log",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::QueryControllerLog::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::QueryControllerLog::Response> res) {
            size_t count = std::min(req->count, static_cast<unsigned int>(log_buffer_.size()));
            for (size_t i = 0; i < count; ++i) {
                res->logs.push_back(log_buffer_[i]);
            }
            res->success = true;
        });

    clear_servo_alarm_srv_ = node_->create_service<rokae_xmate3_ros2::srv::ClearServoAlarm>(
        "/xmate3/cobot/clear_servo_alarm",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::ClearServoAlarm::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::ClearServoAlarm::Response> res) {
            (void)req;
            res->success = true;
        });

    // ==================== 关节与位姿服务 ====================
    get_joint_pos_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetJointPos>(
        "/xmate3/cobot/get_joint_pos",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetJointPos::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetJointPos::Response> res) {
            (void)req;
            for (int i = 0; i < 6 && i < joint_num_; ++i) {
                res->joint_positions[i] = joints_[i]->Position(0);
            }
            res->success = true;
        });

    get_joint_vel_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetJointVel>(
        "/xmate3/cobot/get_joint_vel",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetJointVel::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetJointVel::Response> res) {
            (void)req;
            for (int i = 0; i < 6 && i < joint_num_; ++i) {
                res->joint_vel[i] = joints_[i]->GetVelocity(0);
            }
            res->success = true;
        });

    get_joint_torque_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetJointTorque>(
        "/xmate3/cobot/get_joint_torque",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetJointTorque::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetJointTorque::Response> res) {
            (void)req;
            for (int i = 0; i < 6 && i < joint_num_; ++i) {
                res->joint_torque[i] = joints_[i]->GetForce(0);
            }
            res->success = true;
        });

    get_posture_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetPosture>(
        "/xmate3/cobot/get_posture",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetPosture::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetPosture::Response> res) {
            (void)req;
            std::vector<double> joints(6);
            for (int i = 0; i < 6 && i < joint_num_; ++i) {
                joints[i] = joints_[i]->Position(0);
            }
            auto pose = kinematics_->forwardKinematicsRPY(joints);
            for (int i = 0; i < 6; ++i) res->posture[i] = pose[i];
            res->success = true;
        });

    get_cart_posture_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetCartPosture>(
        "/xmate3/cobot/get_cart_posture",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetCartPosture::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetCartPosture::Response> res) {
            (void)req;
            std::vector<double> joints(6);
            for (int i = 0; i < 6 && i < joint_num_; ++i) {
                joints[i] = joints_[i]->Position(0);
            }
            auto pose = kinematics_->forwardKinematicsRPY(joints);
            res->x = pose[0]; res->y = pose[1]; res->z = pose[2];
            res->rx = pose[3]; res->ry = pose[4]; res->rz = pose[5];
            res->success = true;
        });

    get_base_frame_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetBaseFrame>(
        "/xmate3/cobot/get_base_frame",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetBaseFrame::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetBaseFrame::Response> res) {
            (void)req;
            res->base_frame = {0, 0, 0, 0, 0, 0};
            res->success = true;
        });

    calc_fk_srv_ = node_->create_service<rokae_xmate3_ros2::srv::CalcFk>(
        "/xmate3/cobot/calc_fk",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::CalcFk::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::CalcFk::Response> res) {
            std::vector<double> joints(req->joint_positions.begin(), req->joint_positions.end());
            auto pose = kinematics_->forwardKinematicsRPY(joints);
            for (int i = 0; i < 6; ++i) res->posture[i] = pose[i];
            res->success = true;
        });

    calc_ik_srv_ = node_->create_service<rokae_xmate3_ros2::srv::CalcIk>(
        "/xmate3/cobot/calc_ik",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::CalcIk::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::CalcIk::Response> res) {
            std::vector<double> target(req->target_posture.begin(), req->target_posture.end());
            std::vector<double> current(6);
            for (int i = 0; i < 6 && i < joint_num_; ++i) {
                current[i] = joints_[i]->Position(0);
            }
            const auto candidates = kinematics_->inverseKinematicsMultiSolution(target, current);
            const auto selected = SelectIkSolution(
                *kinematics_, candidates, target, current, req->conf_data,
                default_conf_opt_forced_, true, soft_limit_enabled_, soft_limits_);
            if (!selected.success) {
                res->success = false;
                res->message = selected.message;
                return;
            }
            for (size_t i = 0; i < 6 && i < selected.joints.size(); ++i) {
                res->joint_positions[i] = selected.joints[i];
            }
            res->success = true;
            res->message.clear();
        });

    // ==================== 工具与坐标系服务 ====================
    get_toolset_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetToolset>(
        "/xmate3/cobot/get_toolset",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetToolset::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetToolset::Response> res) {
            (void)req;
            res->tool_name = current_tool_name_;
            res->wobj_name = current_wobj_name_;
            res->tool_pose = current_tool_pose_;
            res->wobj_pose = current_wobj_pose_;
            res->success = true;
        });

    set_toolset_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SetToolset>(
        "/xmate3/cobot/set_toolset",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetToolset::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetToolset::Response> res) {
            current_tool_name_ = req->tool_name;
            current_wobj_name_ = req->wobj_name;
            current_tool_pose_ = req->tool_pose;
            current_wobj_pose_ = req->wobj_pose;
            res->success = true;
        });

    set_toolset_by_name_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SetToolsetByName>(
        "/xmate3/cobot/set_toolset_by_name",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetToolsetByName::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetToolsetByName::Response> res) {
            (void)req;
            res->success = true;
        });

    // ==================== 安全相关服务 ====================
    enable_collision_detection_srv_ = node_->create_service<rokae_xmate3_ros2::srv::EnableCollisionDetection>(
        "/xmate3/cobot/enable_collision_detection",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::EnableCollisionDetection::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::EnableCollisionDetection::Response> res) {
            (void)req;
            collision_detection_enabled_ = true;
            res->success = true;
        });

    disable_collision_detection_srv_ = node_->create_service<rokae_xmate3_ros2::srv::DisableCollisionDetection>(
        "/xmate3/cobot/disable_collision_detection",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::DisableCollisionDetection::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::DisableCollisionDetection::Response> res) {
            (void)req;
            collision_detection_enabled_ = false;
            res->success = true;
        });

    get_soft_limit_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetSoftLimit>(
        "/xmate3/cobot/get_soft_limit",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetSoftLimit::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetSoftLimit::Response> res) {
            (void)req;
            res->enable = soft_limit_enabled_;
            for (int i = 0; i < 6; ++i) {
                res->limits[i * 2] = soft_limits_[i][0];
                res->limits[i * 2 + 1] = soft_limits_[i][1];
            }
            res->success = true;
        });

    set_soft_limit_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SetSoftLimit>(
        "/xmate3/cobot/set_soft_limit",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetSoftLimit::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetSoftLimit::Response> res) {
            soft_limit_enabled_ = req->enable;
            if (req->limits.size() >= 12) {
                for (int i = 0; i < 6; ++i) {
                    soft_limits_[i][0] = req->limits[i * 2];
                    soft_limits_[i][1] = req->limits[i * 2 + 1];
                }
            }
            res->success = true;
        });

    // ==================== 非实时运动控制服务 ====================
    set_motion_control_mode_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SetMotionControlMode>(
        "/xmate3/cobot/set_motion_control_mode",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetMotionControlMode::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetMotionControlMode::Response> res) {
            this->motion_mode_ = req->mode;
            res->success = true;
        });

    move_reset_srv_ = node_->create_service<rokae_xmate3_ros2::srv::MoveReset>(
        "/xmate3/cobot/move_reset",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::MoveReset::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::MoveReset::Response> res) {
            (void)req;
            {
                std::lock_guard<std::mutex> lock(motion_queue_mutex_);
                std::queue<MotionCommand> empty;
                std::swap(motion_queue_, empty);
            }
            current_cmd_.reset();
            motion_running_ = false;
            res->success = true;
        });

    move_start_srv_ = node_->create_service<rokae_xmate3_ros2::srv::MoveStart>(
        "/xmate3/cobot/move_start",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::MoveStart::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::MoveStart::Response> res) {
            (void)req;
            if (!power_on_) {
                res->success = false;
                res->message = "Robot not powered on";
                return;
            }
            motion_running_ = true;
            res->success = true;
        });

    stop_srv_ = node_->create_service<rokae_xmate3_ros2::srv::Stop>(
        "/xmate3/cobot/stop",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::Stop::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::Stop::Response> res) {
            (void)req;
            motion_running_ = false;
            {
                std::lock_guard<std::mutex> lock(motion_queue_mutex_);
                std::queue<MotionCommand> empty;
                std::swap(motion_queue_, empty);
            }
            current_cmd_.reset();
            for (int i = 0; i < 6 && i < joint_num_; ++i) {
                hold_position_[i] = joints_[i]->Position(0);
            }
            res->success = true;
        });

    set_default_speed_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SetDefaultSpeed>(
        "/xmate3/cobot/set_default_speed",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetDefaultSpeed::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetDefaultSpeed::Response> res) {
            default_speed_ = req->speed;
            res->success = true;
        });

    set_default_zone_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SetDefaultZone>(
        "/xmate3/cobot/set_default_zone",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetDefaultZone::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetDefaultZone::Response> res) {
            default_zone_ = req->zone;
            res->success = true;
        });

    set_default_conf_opt_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SetDefaultConfOpt>(
        "/xmate3/cobot/set_default_conf_opt",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetDefaultConfOpt::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetDefaultConfOpt::Response> res) {
            default_conf_opt_forced_ = req->forced;
            res->success = true;
        });

    adjust_speed_online_srv_ = node_->create_service<rokae_xmate3_ros2::srv::AdjustSpeedOnline>(
        "/xmate3/cobot/adjust_speed_online",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::AdjustSpeedOnline::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::AdjustSpeedOnline::Response> res) {
            speed_scale_ = req->scale;
            res->success = true;
        });

    // ==================== 实时控制/高级兼容服务 ====================
    set_rt_control_mode_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SetRtControlMode>(
        "/xmate3/cobot/set_rt_control_mode",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetRtControlMode::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetRtControlMode::Response> res) {
            if (!power_on_) {
                res->success = false;
                res->error_code = 1002;
                res->error_msg = "robot power is off";
                return;
            }
            if (req->mode < 0 || req->mode > 4) {
                res->success = false;
                res->error_code = 1001;
                res->error_msg = "invalid realtime control mode";
                return;
            }
            rt_control_mode_ = req->mode;
            motion_mode_ = 1;
            res->success = true;
            res->error_code = 0;
            res->error_msg.clear();
        });

    get_rt_joint_data_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetRtJointData>(
        "/xmate3/cobot/get_rt_joint_data",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetRtJointData::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetRtJointData::Response> res) {
            (void)req;
            if (rt_control_mode_ < 0) {
                res->success = false;
                res->error_code = 2001;
                res->error_msg = "realtime control mode is not active";
                return;
            }
            for (int i = 0; i < 6 && i < joint_num_; ++i) {
                res->joint_position[i] = joints_[i]->Position(0);
                res->joint_velocity[i] = joints_[i]->GetVelocity(0);
                res->joint_torque[i] = joints_[i]->GetForce(0);
            }
            res->stamp = ToBuiltinTime(node_->get_clock()->now());
            res->success = true;
            res->error_code = 0;
            res->error_msg.clear();
        });

    send_custom_data_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SendCustomData>(
        "/xmate3/cobot/send_custom_data",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SendCustomData::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SendCustomData::Response> res) {
            custom_data_bank_[req->data_topic] = req->custom_data;
            if (req->data_topic.rfind("register/", 0) == 0) {
                register_bank_[req->data_topic.substr(9)] = req->custom_data;
            }
            res->success = true;
            res->error_code = 0;
            res->error_msg.clear();
            res->response_data = std::string("ACK:") + req->data_topic;
            res->send_time = ToBuiltinTime(node_->get_clock()->now());
        });

    register_data_callback_srv_ = node_->create_service<rokae_xmate3_ros2::srv::RegisterDataCallback>(
        "/xmate3/cobot/register_data_callback",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::RegisterDataCallback::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::RegisterDataCallback::Response> res) {
            if (req->data_topic.empty() || req->callback_id.empty()) {
                res->success = false;
                res->error_code = 12001;
                res->error_msg = "data_topic and callback_id must not be empty";
                return;
            }
            if (callback_registry_.count(req->callback_id) > 0) {
                res->success = false;
                res->error_code = 12004;
                res->error_msg = "callback_id already exists";
                return;
            }
            callback_registry_[req->callback_id] = req->data_topic;
            res->success = true;
            res->error_code = 0;
            res->error_msg.clear();
            res->register_time = ToBuiltinTime(node_->get_clock()->now());
        });

    read_register_srv_ = node_->create_service<rokae_xmate3_ros2::srv::ReadRegister>(
        "/xmate3/cobot/read_register",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::ReadRegister::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::ReadRegister::Response> res) {
            auto it = register_bank_.find(req->key);
            res->success = true;
            res->error_code = 0;
            res->error_msg.clear();
            res->value = (it != register_bank_.end()) ? it->second : std::string();
        });

    write_register_srv_ = node_->create_service<rokae_xmate3_ros2::srv::WriteRegister>(
        "/xmate3/cobot/write_register",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::WriteRegister::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::WriteRegister::Response> res) {
            register_bank_[req->key] = req->value;
            res->success = true;
            res->error_code = 0;
            res->error_msg.clear();
        });

    load_rl_project_srv_ = node_->create_service<rokae_xmate3_ros2::srv::LoadRLProject>(
        "/xmate3/cobot/load_rl_project",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::LoadRLProject::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::LoadRLProject::Response> res) {
            std::string project_name = req->project_path;
            const auto slash = project_name.find_last_of("/\\");
            if (slash != std::string::npos) {
                project_name = project_name.substr(slash + 1);
            }
            const auto dot = project_name.find_last_of('.');
            if (dot != std::string::npos) {
                project_name = project_name.substr(0, dot);
            }
            loaded_rl_project_path_ = req->project_path;
            loaded_rl_project_name_ = project_name.empty() ? std::string("default_rl_project") : project_name;
            rl_project_loaded_ = true;
            rl_project_running_ = false;
            rl_current_episode_ = 0;
            res->success = true;
            res->error_code = 0;
            res->error_msg.clear();
            res->project_name = loaded_rl_project_name_;
            res->load_time = 0.0;
        });

    start_rl_project_srv_ = node_->create_service<rokae_xmate3_ros2::srv::StartRLProject>(
        "/xmate3/cobot/start_rl_project",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::StartRLProject::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::StartRLProject::Response> res) {
            if (!rl_project_loaded_ || req->project_id != loaded_rl_project_name_) {
                res->success = false;
                res->error_code = 7001;
                res->error_msg = "RL project is not loaded";
                return;
            }
            if (rl_project_running_) {
                res->success = false;
                res->error_code = 7002;
                res->error_msg = "RL project is already running";
                return;
            }
            rl_project_running_ = true;
            rl_current_episode_ = 1;
            res->success = true;
            res->error_code = 0;
            res->error_msg.clear();
            res->current_episode = rl_current_episode_;
            res->start_time = ToBuiltinTime(node_->get_clock()->now());
        });

    stop_rl_project_srv_ = node_->create_service<rokae_xmate3_ros2::srv::StopRLProject>(
        "/xmate3/cobot/stop_rl_project",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::StopRLProject::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::StopRLProject::Response> res) {
            if (!rl_project_running_ || req->project_id != loaded_rl_project_name_) {
                res->success = false;
                res->error_code = 8001;
                res->error_msg = "RL project is not running";
                return;
            }
            rl_project_running_ = false;
            res->success = true;
            res->error_code = 0;
            res->error_msg.clear();
            res->finished_episode = std::max(rl_current_episode_, 1);
            res->stop_time = ToBuiltinTime(node_->get_clock()->now());
        });

    set_avoid_singularity_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SetAvoidSingularity>(
        "/xmate3/cobot/set_avoid_singularity",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetAvoidSingularity::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetAvoidSingularity::Response> res) {
            avoid_singularity_enabled_ = req->enable;
            res->success = true;
            res->message = avoid_singularity_enabled_ ? "enabled" : "disabled";
        });

    get_avoid_singularity_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetAvoidSingularity>(
        "/xmate3/cobot/get_avoid_singularity",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetAvoidSingularity::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetAvoidSingularity::Response> res) {
            (void)req;
            res->success = true;
            res->enabled = avoid_singularity_enabled_;
            res->message = avoid_singularity_enabled_ ? "enabled" : "disabled";
        });

    calc_joint_torque_srv_ = node_->create_service<rokae_xmate3_ros2::srv::CalcJointTorque>(
        "/xmate3/cobot/calc_joint_torque",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::CalcJointTorque::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::CalcJointTorque::Response> res) {
            std::vector<double> joints(req->joint_pos.begin(), req->joint_pos.end());
            if (joints.size() != 6) {
                res->success = false;
                res->error_code = 4002;
                res->error_msg = "joint_pos size is invalid";
                return;
            }
            Eigen::MatrixXd J = kinematics_->computeJacobian(joints);
            Eigen::Matrix<double, 6, 1> wrench;
            for (int i = 0; i < 6; ++i) {
                wrench(i) = req->external_force[i];
            }
            Eigen::Matrix<double, 6, 1> tau_ext = J.transpose() * wrench;
            for (int i = 0; i < 6; ++i) {
                res->gravity_torque[i] = std::sin(req->joint_pos[i]) * 1.5;
                res->coriolis_torque[i] = req->joint_vel[i] * 0.05;
                res->joint_torque[i] = res->gravity_torque[i] + res->coriolis_torque[i] + req->joint_acc[i] * 0.02 + tau_ext(i);
            }
            res->success = true;
            res->error_code = 0;
            res->error_msg.clear();
        });

    generate_s_trajectory_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GenerateSTrajectory>(
        "/xmate3/cobot/generate_s_trajectory",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GenerateSTrajectory::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GenerateSTrajectory::Response> res) {
            std::vector<double> start(req->start_joint_pos.begin(), req->start_joint_pos.end());
            std::vector<double> target(req->target_joint_pos.begin(), req->target_joint_pos.end());
            auto trajectory = TrajectoryPlanner::planJointMove(start, target, 32.0, 0.001);
            if (trajectory.empty()) {
                res->success = false;
                res->error_code = 3003;
                res->error_msg = "trajectory planning failed";
                return;
            }
            res->trajectory_points.reserve(trajectory.size());
            for (const auto& point : trajectory) {
                rokae_xmate3_ros2::msg::JointPos6 joint_msg;
                for (int i = 0; i < 6 && i < static_cast<int>(point.size()); ++i) {
                    joint_msg.pos[i] = point[i];
                }
                res->trajectory_points.push_back(joint_msg);
            }
            res->total_time = trajectory.size() > 1 ? (trajectory.size() - 1) * 0.001 : 0.0;
            res->stamp = ToBuiltinTime(node_->get_clock()->now());
            res->success = true;
            res->error_code = 0;
            res->error_msg.clear();
        });

    map_cartesian_to_joint_torque_srv_ = node_->create_service<rokae_xmate3_ros2::srv::MapCartesianToJointTorque>(
        "/xmate3/cobot/map_cartesian_to_joint_torque",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::MapCartesianToJointTorque::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::MapCartesianToJointTorque::Response> res) {
            std::vector<double> joints(req->joint_pos.begin(), req->joint_pos.end());
            if (joints.size() != 6) {
                res->success = false;
                res->error_code = 6002;
                res->error_msg = "joint_pos size is invalid";
                return;
            }
            Eigen::MatrixXd J = kinematics_->computeJacobian(joints);
            Eigen::Matrix<double, 6, 1> wrench;
            for (int i = 0; i < 6; ++i) {
                wrench(i) = req->cart_force[i];
            }
            Eigen::Matrix<double, 6, 1> tau = J.transpose() * wrench;
            for (int i = 0; i < 6; ++i) {
                res->joint_torque[i] = tau(i);
            }
            res->stamp = ToBuiltinTime(node_->get_clock()->now());
            res->success = true;
            res->error_code = 0;
            res->error_msg.clear();
        });

    get_end_torque_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetEndTorque>(
        "/xmate3/cobot/get_end_torque",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetEndTorque::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetEndTorque::Response> res) {
            (void)req;
            std::vector<double> joints(6, 0.0);
            Eigen::Matrix<double, 6, 1> tau = Eigen::Matrix<double, 6, 1>::Zero();
            for (int i = 0; i < 6 && i < joint_num_; ++i) {
                joints[i] = joints_[i]->Position(0);
                tau(i) = joints_[i]->GetForce(0);
            }
            Eigen::MatrixXd J = kinematics_->computeJacobian(joints);
            Eigen::Matrix<double, 6, 1> wrench = J.transpose().completeOrthogonalDecomposition().solve(tau);
            for (int i = 0; i < 6; ++i) {
                res->end_torque[i] = wrench(i);
            }
            res->success = true;
            res->message.clear();
        });

    // ==================== IO控制服务 ====================
    get_di_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetDI>(
        "/xmate3/io/get_di",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetDI::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetDI::Response> res) {
            auto key = std::make_pair(req->board, req->port);
            res->state = di_state_[key];
            res->success = true;
        });

    get_do_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetDO>(
        "/xmate3/io/get_do",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetDO::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetDO::Response> res) {
            auto key = std::make_pair(req->board, req->port);
            res->state = do_state_[key];
            res->success = true;
        });

    set_di_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SetDI>(
        "/xmate3/io/set_di",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetDI::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetDI::Response> res) {
            auto key = std::make_pair(req->board, req->port);
            di_state_[key] = req->state;
            res->success = true;
        });

    set_do_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SetDO>(
        "/xmate3/io/set_do",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetDO::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetDO::Response> res) {
            auto key = std::make_pair(req->board, req->port);
            do_state_[key] = req->state;
            res->success = true;
        });

    get_ai_srv_ = node_->create_service<rokae_xmate3_ros2::srv::GetAI>(
        "/xmate3/io/get_ai",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetAI::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetAI::Response> res) {
            auto key = std::make_pair(req->board, req->port);
            res->value = ai_state_[key];
            res->success = true;
        });

    set_ao_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SetAO>(
        "/xmate3/io/set_ao",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetAO::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetAO::Response> res) {
            auto key = std::make_pair(req->board, req->port);
            ao_state_[key] = req->value;
            res->success = true;
        });

    set_simulation_mode_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SetSimulationMode>(
        "/xmate3/io/set_simulation_mode",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetSimulationMode::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetSimulationMode::Response> res) {
            simulation_mode_ = req->state;
            res->success = true;
        });

    // 兼容旧命名空间 /xmate3/cobot/*
    compatibility_services_.push_back(node_->create_service<rokae_xmate3_ros2::srv::GetDI>(
        "/xmate3/cobot/get_di",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetDI::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetDI::Response> res) {
            auto key = std::make_pair(req->board, req->port);
            res->state = di_state_[key];
            res->success = true;
        }));
    compatibility_services_.push_back(node_->create_service<rokae_xmate3_ros2::srv::GetDO>(
        "/xmate3/cobot/get_do",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetDO::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetDO::Response> res) {
            auto key = std::make_pair(req->board, req->port);
            res->state = do_state_[key];
            res->success = true;
        }));
    compatibility_services_.push_back(node_->create_service<rokae_xmate3_ros2::srv::SetDI>(
        "/xmate3/cobot/set_di",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetDI::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetDI::Response> res) {
            auto key = std::make_pair(req->board, req->port);
            di_state_[key] = req->state;
            res->success = true;
        }));
    compatibility_services_.push_back(node_->create_service<rokae_xmate3_ros2::srv::SetDO>(
        "/xmate3/cobot/set_do",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetDO::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetDO::Response> res) {
            auto key = std::make_pair(req->board, req->port);
            do_state_[key] = req->state;
            res->success = true;
        }));
    compatibility_services_.push_back(node_->create_service<rokae_xmate3_ros2::srv::GetAI>(
        "/xmate3/cobot/get_ai",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::GetAI::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::GetAI::Response> res) {
            auto key = std::make_pair(req->board, req->port);
            res->value = ai_state_[key];
            res->success = true;
        }));
    compatibility_services_.push_back(node_->create_service<rokae_xmate3_ros2::srv::SetAO>(
        "/xmate3/cobot/set_ao",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetAO::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetAO::Response> res) {
            auto key = std::make_pair(req->board, req->port);
            ao_state_[key] = req->value;
            res->success = true;
        }));
    compatibility_services_.push_back(node_->create_service<rokae_xmate3_ros2::srv::SetSimulationMode>(
        "/xmate3/cobot/set_simulation_mode",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SetSimulationMode::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SetSimulationMode::Response> res) {
            simulation_mode_ = req->state;
            res->success = true;
        }));
    // ==================== 拖动与路径录制服务 ====================
    enable_drag_srv_ = node_->create_service<rokae_xmate3_ros2::srv::EnableDrag>(
        "/xmate3/cobot/enable_drag",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::EnableDrag::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::EnableDrag::Response> res) {
            (void)req;
            drag_mode_ = true;
            res->success = true;
        });

    disable_drag_srv_ = node_->create_service<rokae_xmate3_ros2::srv::DisableDrag>(
        "/xmate3/cobot/disable_drag",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::DisableDrag::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::DisableDrag::Response> res) {
            (void)req;
            drag_mode_ = false;
            for (int i = 0; i < 6 && i < joint_num_; ++i) {
                hold_position_[i] = joints_[i]->Position(0);
            }
            res->success = true;
        });

    start_record_path_srv_ = node_->create_service<rokae_xmate3_ros2::srv::StartRecordPath>(
        "/xmate3/cobot/start_record_path",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::StartRecordPath::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::StartRecordPath::Response> res) {
            (void)req;
            is_recording_path_ = true;
            recorded_path_.clear();
            res->success = true;
        });

    stop_record_path_srv_ = node_->create_service<rokae_xmate3_ros2::srv::StopRecordPath>(
        "/xmate3/cobot/stop_record_path",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::StopRecordPath::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::StopRecordPath::Response> res) {
            (void)req;
            is_recording_path_ = false;
            res->success = true;
        });

    cancel_record_path_srv_ = node_->create_service<rokae_xmate3_ros2::srv::CancelRecordPath>(
        "/xmate3/cobot/cancel_record_path",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::CancelRecordPath::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::CancelRecordPath::Response> res) {
            (void)req;
            is_recording_path_ = false;
            recorded_path_.clear();
            res->success = true;
        });

    save_record_path_srv_ = node_->create_service<rokae_xmate3_ros2::srv::SaveRecordPath>(
        "/xmate3/cobot/save_record_path",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::SaveRecordPath::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::SaveRecordPath::Response> res) {
            saved_paths_[req->name] = recorded_path_;
            res->success = true;
        });

    replay_path_srv_ = node_->create_service<rokae_xmate3_ros2::srv::ReplayPath>(
        "/xmate3/cobot/replay_path",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::ReplayPath::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::ReplayPath::Response> res) {
            auto it = saved_paths_.find(req->name);
            if (it == saved_paths_.end()) {
                res->success = false;
                res->message = "Path not found";
                return;
            }
            {
                std::lock_guard<std::mutex> lock(motion_queue_mutex_);
                for (const auto& joints : it->second) {
                    MotionCommand cmd;
                    cmd.type = MotionType::MOVE_ABSJ;
                    cmd.target_joints = joints;
                    cmd.speed = static_cast<int>(default_speed_ * req->rate);
                    cmd.zone = default_zone_;
                    motion_queue_.push(cmd);
                }
            }
            motion_running_ = true;
            res->success = true;
        });

    remove_path_srv_ = node_->create_service<rokae_xmate3_ros2::srv::RemovePath>(
        "/xmate3/cobot/remove_path",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::RemovePath::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::RemovePath::Response> res) {
            if (req->remove_all) {
                saved_paths_.clear();
            } else {
                saved_paths_.erase(req->name);
            }
            res->success = true;
        });

    query_path_lists_srv_ = node_->create_service<rokae_xmate3_ros2::srv::QueryPathLists>(
        "/xmate3/cobot/query_path_lists",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::QueryPathLists::Request> req,
               std::shared_ptr<rokae_xmate3_ros2::srv::QueryPathLists::Response> res) {
            (void)req;
            for (const auto& pair : saved_paths_) {
                res->path_names.push_back(pair.first);
            }
            res->success = true;
        });
}

void XCoreControllerPlugin::InitActionServers() {
    move_append_action_server_ = rclcpp_action::create_server<rokae_xmate3_ros2::action::MoveAppend>(
        node_,
        "/xmate3/cobot/move_append",
        [this](const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const rokae_xmate3_ros2::action::MoveAppend::Goal> goal) {
            (void)uuid;
            if (!power_on_) {
                return rclcpp_action::GoalResponse::REJECT;
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<rokae_xmate3_ros2::action::MoveAppend>> goal_handle) {
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<rokae_xmate3_ros2::action::MoveAppend>> goal_handle) {
            std::thread{[this, goal_handle]() { this->ExecuteMoveAppend(goal_handle); }}.detach();
        });
}

void XCoreControllerPlugin::ExecuteMoveAppend(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<rokae_xmate3_ros2::action::MoveAppend>> goal_handle) {

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Result>();

    std::vector<double> current_joints(6);
    for (int i = 0; i < 6 && i < joint_num_; ++i) {
        current_joints[i] = joints_[i]->Position(0);
    }

    size_t total_cmds = goal->absj_cmds.size() + goal->j_cmds.size() + goal->l_cmds.size() +
                        goal->c_cmds.size() + goal->cf_cmds.size() + goal->sp_cmds.size();
    std::string planning_error;

    {
        std::lock_guard<std::mutex> lock(motion_queue_mutex_);
        for (const auto& cmd : goal->absj_cmds) {
            MotionCommand motion_cmd;
            motion_cmd.type = MotionType::MOVE_ABSJ;
            motion_cmd.target_joints.assign(cmd.target.joints.begin(), cmd.target.joints.end());
            if (soft_limit_enabled_ && ViolatesSoftLimit(motion_cmd.target_joints, soft_limits_)) {
                planning_error = "MoveAbsJ target violates soft limit";
                break;
            }
            motion_cmd.speed = cmd.speed > 0 ? cmd.speed : default_speed_;
            motion_cmd.zone = cmd.zone;
            motion_cmd.trajectory = TrajectoryPlanner::planJointMove(
                current_joints, motion_cmd.target_joints, motion_cmd.speed, kTrajectorySampleDt);
            motion_cmd.current_point = 0;
            motion_cmd.trajectory_dt = kTrajectorySampleDt;
            motion_cmd.trajectory_elapsed = 0.0;
            motion_cmd.fine_tuning_steps = 0;
            motion_cmd.in_fine_tuning = false;
            motion_queue_.push(motion_cmd);
            current_joints = motion_cmd.target_joints;
        }

        for (const auto& cmd : goal->j_cmds) {
            if (!planning_error.empty()) {
                break;
            }
            MotionCommand motion_cmd;
            motion_cmd.type = MotionType::MOVE_J;
            motion_cmd.target_cartesian = ApplyOffset(
                MsgPoseToVector(cmd.target), cmd.offset_type, cmd.offset_pose);
            motion_cmd.speed = cmd.speed > 0 ? cmd.speed : default_speed_;
            motion_cmd.zone = cmd.zone;
            const auto candidate_solutions = kinematics_->inverseKinematicsMultiSolution(
                motion_cmd.target_cartesian, current_joints);
            const auto selected = SelectIkSolution(
                *kinematics_, candidate_solutions, motion_cmd.target_cartesian, current_joints,
                cmd.target.conf_data, default_conf_opt_forced_, true, soft_limit_enabled_, soft_limits_);
            if (!selected.success) {
                planning_error = "MoveJ planning failed: " + selected.message;
                break;
            }
            motion_cmd.target_joints = selected.joints;
            motion_cmd.trajectory = TrajectoryPlanner::planJointMove(
                current_joints, motion_cmd.target_joints, motion_cmd.speed, kTrajectorySampleDt);
            motion_cmd.current_point = 0;
            motion_cmd.trajectory_dt = kTrajectorySampleDt;
            motion_cmd.trajectory_elapsed = 0.0;
            motion_cmd.fine_tuning_steps = 0;
            motion_cmd.in_fine_tuning = false;
            motion_queue_.push(motion_cmd);
            current_joints = motion_cmd.target_joints;
        }

        for (const auto& cmd : goal->l_cmds) {
            if (!planning_error.empty()) {
                break;
            }
            MotionCommand motion_cmd;
            motion_cmd.type = MotionType::MOVE_L;
            motion_cmd.target_cartesian = ApplyOffset(
                MsgPoseToVector(cmd.target), cmd.offset_type, cmd.offset_pose);
            motion_cmd.speed = cmd.speed > 0 ? cmd.speed : default_speed_;
            motion_cmd.zone = cmd.zone;
            const auto current_pose = kinematics_->forwardKinematicsRPY(current_joints);
            const auto cart_trajectory = TrajectoryPlanner::planCartesianLine(
                current_pose, motion_cmd.target_cartesian, motion_cmd.speed, kTrajectorySampleDt);
            std::string error_message;
            std::vector<double> last_joints;
            if (!BuildJointTrajectoryFromCartesian(
                    *kinematics_, cart_trajectory, current_joints, cmd.target.conf_data,
                    default_conf_opt_forced_, true, soft_limit_enabled_, soft_limits_,
                    motion_cmd.trajectory, last_joints, error_message)) {
                planning_error = "MoveL planning failed: " + error_message;
                break;
            }
            motion_cmd.target_joints = last_joints;
            motion_cmd.current_point = 0;
            motion_cmd.trajectory_dt = kTrajectorySampleDt;
            motion_cmd.trajectory_elapsed = 0.0;
            motion_cmd.fine_tuning_steps = 0;
            motion_cmd.in_fine_tuning = false;
            motion_queue_.push(motion_cmd);
            current_joints = motion_cmd.target_joints;
        }

        for (const auto& cmd : goal->c_cmds) {
            if (!planning_error.empty()) {
                break;
            }
            MotionCommand motion_cmd;
            motion_cmd.type = MotionType::MOVE_C;
            motion_cmd.speed = cmd.speed > 0 ? cmd.speed : default_speed_;
            motion_cmd.zone = cmd.zone;
            const auto current_pose = kinematics_->forwardKinematicsRPY(current_joints);
            motion_cmd.target_cartesian = ApplyOffset(
                MsgPoseToVector(cmd.target), cmd.target_offset_type, cmd.target_offset_pose);
            motion_cmd.aux_cartesian = ApplyOffset(
                MsgPoseToVector(cmd.aux), cmd.aux_offset_type, cmd.aux_offset_pose);
            const auto cart_trajectory = TrajectoryPlanner::planCircularArc(
                current_pose, motion_cmd.aux_cartesian, motion_cmd.target_cartesian, motion_cmd.speed,
                kTrajectorySampleDt);
            std::string error_message;
            std::vector<double> last_joints;
            if (!BuildJointTrajectoryFromCartesian(
                    *kinematics_, cart_trajectory, current_joints, cmd.target.conf_data,
                    default_conf_opt_forced_, true, soft_limit_enabled_, soft_limits_,
                    motion_cmd.trajectory, last_joints, error_message)) {
                planning_error = "MoveC planning failed: " + error_message;
                break;
            }
            motion_cmd.target_joints = last_joints;
            motion_cmd.current_point = 0;
            motion_cmd.trajectory_dt = kTrajectorySampleDt;
            motion_cmd.trajectory_elapsed = 0.0;
            motion_cmd.fine_tuning_steps = 0;
            motion_cmd.in_fine_tuning = false;
            motion_queue_.push(motion_cmd);
            current_joints = motion_cmd.target_joints;
        }

        for (const auto& cmd : goal->cf_cmds) {
            if (!planning_error.empty()) {
                break;
            }
            MotionCommand motion_cmd;
            motion_cmd.type = MotionType::MOVE_CF;
            motion_cmd.speed = cmd.speed > 0 ? cmd.speed : default_speed_;
            motion_cmd.zone = cmd.zone;
            const auto current_pose = kinematics_->forwardKinematicsRPY(current_joints);
            motion_cmd.target_cartesian = ApplyOffset(
                MsgPoseToVector(cmd.target), cmd.target_offset_type, cmd.target_offset_pose);
            motion_cmd.aux_cartesian = ApplyOffset(
                MsgPoseToVector(cmd.aux), cmd.aux_offset_type, cmd.aux_offset_pose);
            const auto cart_trajectory = TrajectoryPlanner::planCircularContinuous(
                current_pose, motion_cmd.aux_cartesian, motion_cmd.target_cartesian,
                cmd.angle, motion_cmd.speed, kTrajectorySampleDt);
            std::string error_message;
            std::vector<double> last_joints;
            if (!BuildJointTrajectoryFromCartesian(
                    *kinematics_, cart_trajectory, current_joints, cmd.target.conf_data,
                    default_conf_opt_forced_, true, soft_limit_enabled_, soft_limits_,
                    motion_cmd.trajectory, last_joints, error_message)) {
                planning_error = "MoveCF planning failed: " + error_message;
                break;
            }
            motion_cmd.target_joints = last_joints;
            motion_cmd.current_point = 0;
            motion_cmd.trajectory_dt = kTrajectorySampleDt;
            motion_cmd.trajectory_elapsed = 0.0;
            motion_cmd.fine_tuning_steps = 0;
            motion_cmd.in_fine_tuning = false;
            motion_queue_.push(motion_cmd);
            current_joints = motion_cmd.target_joints;
        }

        for (const auto& cmd : goal->sp_cmds) {
            if (!planning_error.empty()) {
                break;
            }
            MotionCommand motion_cmd;
            motion_cmd.type = MotionType::MOVE_SP;
            motion_cmd.speed = cmd.speed > 0 ? cmd.speed : default_speed_;
            motion_cmd.zone = cmd.zone;
            const auto current_pose = kinematics_->forwardKinematicsRPY(current_joints);
            motion_cmd.target_cartesian = ApplyOffset(
                MsgPoseToVector(cmd.target), cmd.offset_type, cmd.offset_pose);
            const auto cart_trajectory = TrajectoryPlanner::planSpiralMove(
                current_pose, motion_cmd.target_cartesian, cmd.radius, cmd.radius_step,
                cmd.angle, cmd.direction, motion_cmd.speed, kTrajectorySampleDt);
            std::string error_message;
            std::vector<double> last_joints;
            if (!BuildJointTrajectoryFromCartesian(
                    *kinematics_, cart_trajectory, current_joints, cmd.target.conf_data,
                    default_conf_opt_forced_, true, soft_limit_enabled_, soft_limits_,
                    motion_cmd.trajectory, last_joints, error_message)) {
                planning_error = "MoveSP planning failed: " + error_message;
                break;
            }
            motion_cmd.target_joints = last_joints;
            motion_cmd.current_point = 0;
            motion_cmd.trajectory_dt = kTrajectorySampleDt;
            motion_cmd.trajectory_elapsed = 0.0;
            motion_cmd.fine_tuning_steps = 0;
            motion_cmd.in_fine_tuning = false;
            motion_queue_.push(motion_cmd);
            current_joints = motion_cmd.target_joints;
        }
    }

    if (!planning_error.empty()) {
        result->success = false;
        result->message = planning_error;
        goal_handle->abort(result);
        return;
    }

    motion_running_ = true;

    size_t last_queue_size;
    {
        std::lock_guard<std::mutex> lock(motion_queue_mutex_);
        last_queue_size = motion_queue_.size();
    }
    while (rclcpp::ok() && motion_running_) {
        bool queue_empty;
        size_t current_queue_size;
        {
            std::lock_guard<std::mutex> lock(motion_queue_mutex_);
            queue_empty = motion_queue_.empty();
            current_queue_size = motion_queue_.size();
        }
        if (queue_empty && !current_cmd_) {
            break;
        }

        if (current_queue_size != last_queue_size) {
            size_t completed_cmds = total_cmds - current_queue_size - (current_cmd_ ? 1 : 0);
            auto feedback = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Feedback>();
            feedback->progress = static_cast<double>(completed_cmds) / total_cmds;
            feedback->current_cmd_index = completed_cmds;
            feedback->current_state = "executing";
            goal_handle->publish_feedback(feedback);
            last_queue_size = current_queue_size;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Canceled";
        goal_handle->canceled(result);
    } else {
        result->success = true;
        result->message = "All commands executed";
        result->cmd_id = std::to_string(total_cmds);
        goal_handle->succeed(result);
    }

    motion_running_ = false;
}

void XCoreControllerPlugin::OnUpdate() {
    double sim_dt = kTrajectorySampleDt;
    if (model_ && model_->GetWorld()) {
        const double sim_time = model_->GetWorld()->SimTime().Double();
        if (last_sim_time_ >= 0.0) {
            const double candidate_dt = sim_time - last_sim_time_;
            if (candidate_dt > 1e-6 && candidate_dt < 0.1) {
                sim_dt = candidate_dt;
            }
        }
        last_sim_time_ = sim_time;
    }
    current_update_dt_ = sim_dt;

    // ========== 【最高优先级】强制设置初始姿态（仅执行一次） ==========
    if (!initial_pose_set_ && joint_num_ > 0) {
        gzmsg << "[xCore Controller] Setting initial joint pose..." << std::endl;
        for (int i = 0; i < joint_num_ && i < static_cast<int>(default_initial_pose_.size()); ++i) {
            // 第三个参数true：跳过物理约束，强制设置关节位置
            joints_[i]->SetPosition(0, default_initial_pose_[i], true);
            // 同步更新保持位置，确保抱闸时锁在初始姿态
            hold_position_[i] = default_initial_pose_[i];
            // 清空关节速度和力，避免残留值导致姿态跳变
            joints_[i]->SetVelocity(0, 0.0);
            joints_[i]->SetForce(0, 0.0);
        }
        // 标记初始化完成
        initial_pose_set_ = true;
        joint_positions_initialized_ = true;
        hold_position_initialized_ = true;
        gzmsg << "[xCore Controller] Initial pose set successfully!" << std::endl;
    }

    // 原有发布逻辑
    static auto last_pub_time = node_->now();
    auto now = node_->now();
    if ((now - last_pub_time).seconds() >= 0.001) {
        PublishJointStates();
        PublishOperationState();
        last_pub_time = now;
    }

    // 路径录制逻辑
    if (is_recording_path_) {
        std::vector<double> joints(6);
        for (int i = 0; i < 6 && i < joint_num_; ++i) {
            joints[i] = joints_[i]->Position(0);
        }
        recorded_path_.push_back(joints);
    }

    // 运动控制/抱闸逻辑（必须在初始姿态设置完成后执行）
    if (!drag_mode_ && initial_pose_set_) {
        ExecuteMotion();
    }

    rclcpp::spin_some(node_);
}

void XCoreControllerPlugin::ExecuteMotion() {
    // 1. 未上电状态：物理抱闸锁死（仅当初始姿态设置完成后执行）
    if (!power_on_ && initial_pose_set_) {
        if (!is_joint_locked_) {
            gzmsg << "[xCore Controller] Engaging joint brakes..." << std::endl;
            for (int i = 0; i < 6 && i < joint_num_; ++i) {
                // 抱闸时锁定的是已经设置好的初始姿态，而非零位
                double current_pos = joints_[i]->Position(0);
                hold_position_[i] = current_pos;

                // 限位锁死+速度清零双重保险
                joints_[i]->SetLowerLimit(0, current_pos);
                joints_[i]->SetUpperLimit(0, current_pos);
                joints_[i]->SetVelocity(0, 0.0);
                joints_[i]->SetForce(0, 0.0);
            }
            is_joint_locked_ = true;
            gzmsg << "[xCore Controller] Joint brakes locked at initial pose." << std::endl;
        }
        return;
    }

    // 2. 上电状态：解锁关节并执行控制
    if (is_joint_locked_ && power_on_) {
        gzmsg << "[xCore Controller] Releasing joint brakes..." << std::endl;
        for (int i = 0; i < 6 && i < joint_num_; ++i) {
            // 恢复原始限位
            joints_[i]->SetLowerLimit(0, original_joint_limits_[i].first);
            joints_[i]->SetUpperLimit(0, original_joint_limits_[i].second);
        }
        is_joint_locked_ = false;
        // 解锁瞬间更新保持位置，防止跳变
        for (int i = 0; i < 6 && i < joint_num_; ++i) {
            hold_position_[i] = joints_[i]->Position(0);
        }
        gzmsg << "[xCore Controller] Joint brakes released." << std::endl;
    }

    // 上电无指令时的位置保持初始化
    if (!hold_position_initialized_ && joint_num_ > 0) {
        for (int i = 0; i < 6 && i < joint_num_; ++i) {
            hold_position_[i] = joints_[i]->Position(0);
        }
        hold_position_initialized_ = true;
    }

    // ========== 改进版轨迹运动执行逻辑 ==========
    if (!current_cmd_) {
        std::lock_guard<std::mutex> lock(motion_queue_mutex_);
        if (!motion_queue_.empty()) {
            current_cmd_ = std::make_unique<MotionCommand>(motion_queue_.front());
            motion_queue_.pop();
            current_cmd_->current_point = 0;
            current_cmd_->trajectory_dt =
                current_cmd_->trajectory_dt > 1e-9 ? current_cmd_->trajectory_dt : kTrajectorySampleDt;
            current_cmd_->trajectory_elapsed = 0.0;
            current_cmd_->fine_tuning_steps = 0;
            current_cmd_->in_fine_tuning = false;
        }
    }

    if (current_cmd_) {
        // 确保有目标关节位置
        if (current_cmd_->target_joints.empty() && !current_cmd_->trajectory.empty()) {
            current_cmd_->target_joints = current_cmd_->trajectory.back();
        }

        if (!current_cmd_->trajectory.empty()) {
            const double effective_dt =
                current_update_dt_ * std::clamp(speed_scale_, 0.05, 2.0);
            current_cmd_->trajectory_elapsed += effective_dt;
            const auto sample = SampleJointTrajectory(
                current_cmd_->trajectory, current_cmd_->trajectory_elapsed, current_cmd_->trajectory_dt);
            current_cmd_->current_point = static_cast<size_t>(
                std::min(current_cmd_->trajectory.size() - 1,
                         static_cast<size_t>(current_cmd_->trajectory_elapsed / current_cmd_->trajectory_dt)));

            for (int i = 0; i < 6 && i < joint_num_; ++i) {
                const double target_pos = sample.position[i];
                const double target_vel = sample.velocity[i];
                joints_[i]->SetPosition(0, target_pos, true);
                joints_[i]->SetVelocity(0, target_vel);
                joints_[i]->SetForce(0, 0.0);
            }

            if (sample.finished) {
                for (int i = 0; i < 6 && i < joint_num_; ++i) {
                    hold_position_[i] = current_cmd_->target_joints[i];
                    joints_[i]->SetPosition(0, current_cmd_->target_joints[i], true);
                    joints_[i]->SetVelocity(0, 0.0);
                    joints_[i]->SetForce(0, 0.0);
                }
                current_cmd_.reset();
            }
        } else {
            for (int i = 0; i < 6 && i < joint_num_; ++i) {
                joints_[i]->SetPosition(0, current_cmd_->target_joints[i], true);
                joints_[i]->SetVelocity(0, 0.0);
                joints_[i]->SetForce(0, 0.0);
                hold_position_[i] = current_cmd_->target_joints[i];
            }
            current_cmd_.reset();
        }
    } else {
        std::lock_guard<std::mutex> lock(motion_queue_mutex_);
        if (motion_queue_.empty()) {
            motion_running_ = false;
        }
        // 上电无指令时的位置保持
        for (int i = 0; i < 6 && i < joint_num_; ++i) {
            joints_[i]->SetPosition(0, hold_position_[i], true);
            joints_[i]->SetVelocity(0, 0.0);
            joints_[i]->SetForce(0, 0.0);
        }
    }
}

void XCoreControllerPlugin::PublishJointStates() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "base_link";

    for (size_t i = 0; i < joint_names_.size() && i < joints_.size(); ++i) {
        msg.name.push_back(joint_names_[i]);
        msg.position.push_back(joints_[i]->Position(0));
        msg.velocity.push_back(joints_[i]->GetVelocity(0));
        msg.effort.push_back(joints_[i]->GetForce(0));
    }

    joint_state_pub_->publish(msg);
}

void XCoreControllerPlugin::PublishOperationState() {
    rokae_xmate3_ros2::msg::OperationState msg;
    msg.state = motion_running_ ? rokae_xmate3_ros2::msg::OperationState::MOVING : rokae_xmate3_ros2::msg::OperationState::IDLE;
    operation_state_pub_->publish(msg);
}

GZ_REGISTER_MODEL_PLUGIN(XCoreControllerPlugin)

} // namespace gazebo
