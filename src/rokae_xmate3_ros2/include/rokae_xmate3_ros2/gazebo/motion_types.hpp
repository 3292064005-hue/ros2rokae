/**
 * @file motion_types.hpp
 * @brief 运动指令类型定义
 */

#ifndef ROKAE_XMATE3_GAZEBO_MOTION_TYPES_HPP
#define ROKAE_XMATE3_GAZEBO_MOTION_TYPES_HPP

#include <vector>

namespace gazebo {

/**
 * @brief 运动指令类型枚举
 */
enum class MotionType {
    NONE,
    MOVE_ABSJ,      // 轴空间绝对运动
    MOVE_J,         // 轴空间相对运动
    MOVE_L,         // 直线运动
    MOVE_C,         // 圆弧运动
    MOVE_CF,        // 连续轨迹
    MOVE_SP         // 样条曲线
};

/**
 * @brief 运动指令结构体
 */
struct MotionCommand {
    MotionType type;
    std::vector<double> target_joints;      // 目标关节位置
    std::vector<double> target_cartesian;   // 目标笛卡尔位置 [x,y,z,rx,ry,rz]
    std::vector<double> aux_cartesian;      // 辅助点（圆弧运动用）
    int speed;                              // 速度百分比 1-100
    int zone;                               // 转弯区半径 mm
    int cmd_id;                             // 指令ID

    // 轨迹插补数据
    std::vector<std::vector<double>> trajectory;  // 插补后的轨迹点
    size_t current_point;                         // 当前执行到的轨迹点
    double trajectory_dt = 0.01;                 // 轨迹采样周期（秒）
    double trajectory_elapsed = 0.0;             // 已执行轨迹时间（秒）
    int fine_tuning_steps;                        // 精调阶段计数
    int settle_attempts = 0;                      // 精调阶段总尝试次数
    bool in_fine_tuning;                          // 是否在精调阶段
};

} // namespace gazebo

#endif // ROKAE_XMATE3_GAZEBO_MOTION_TYPES_HPP
