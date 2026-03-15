/**
 * @file 07_safety_collision.cpp
 * @brief 第4.3/4.8节 - 安全与碰撞检测示例
 *
 * 本示例演示：
 * - 碰撞检测启用/禁用
 * - 碰撞检测参数设置
 * - 软限位设置
 * - 拖动示教（协作机器人）
 */

#include <iostream>
#include <array>
#include <system_error>
#include "rokae_xmate3_ros2/robot.hpp"

using namespace std;

int main() {
    cout << "==========================================" << endl;
    cout << "  示例 7: 安全与碰撞检测" << endl;
    cout << "  (对应SDK手册第4.3/4.8节)" << endl;
    cout << "==========================================" << endl;

    error_code ec;

    // 1. 初始化连接
    cout << "\n[1] 初始化连接..." << endl;
    rokae::ros2::xMateRobot robot;
    robot.connectToRobot(ec);
    if (ec) {
        cerr << "连接失败: " << ec.message() << endl;
        return 1;
    }
    robot.setPowerState(true, ec);
    cout << "    已连接并上电" << endl;

    // 2. 碰撞检测 - 禁用
    cout << "\n[2] 禁用碰撞检测..." << endl;
    robot.disableCollisionDetection(ec);
    if (!ec) {
        cout << "    碰撞检测已禁用" << endl;
    }

    // 3. 碰撞检测 - 启用并设置参数
    cout << "\n[3] 启用碰撞检测并设置参数..." << endl;

    // 灵敏度设置: 6个关节的灵敏度 (0.0~1.0)
    array<double, 6> sensitivity = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8};

    // 停止级别
    rokae::StopLevel stop_behaviour = rokae::StopLevel::stop2;

    // 回退距离 (米)
    double fallback_distance = 0.01;  // 1cm

    robot.enableCollisionDetection(sensitivity, stop_behaviour, fallback_distance, ec);
    if (!ec) {
        cout << "    碰撞检测已启用" << endl;
        cout << "    灵敏度: [";
        for (size_t i = 0; i < 6; ++i) {
            cout << sensitivity[i];
            if (i < 5) cout << ", ";
        }
        cout << "]" << endl;
        cout << "    停止级别: stop2 (规划停止后不断电)" << endl;
        cout << "    回退距离: " << fallback_distance * 1000 << " mm" << endl;
    }

    // 4. 软限位 - 读取当前设置
    cout << "\n[4] 读取软限位设置..." << endl;
    array<array<double, 2>, 6> limits;
    bool limits_enabled = robot.getSoftLimit(limits, ec);
    if (!ec) {
        cout << "    软限位状态: " << (limits_enabled ? "启用" : "禁用") << endl;
        cout << "    各关节限位 (弧度):" << endl;
        for (size_t i = 0; i < 6; ++i) {
            cout << "      关节" << (i+1) << ": ["
                 << limits[i][0] << ", " << limits[i][1] << "]" << endl;
        }
    }

    // 5. 软限位 - 设置新的限位
    cout << "\n[5] 设置软限位 (示例)..." << endl;
    array<array<double, 2>, 6> new_limits;
    // 设置示例限位 (弧度)
    new_limits[0] = {-2.0, 2.0};  // J1
    new_limits[1] = {-2.0, 2.0};  // J2
    new_limits[2] = {-2.0, 2.0};  // J3
    new_limits[3] = {-2.0, 2.0};  // J4
    new_limits[4] = {-2.0, 2.0};  // J5
    new_limits[5] = {-2.0, 2.0};  // J6

    // 注意: 在仿真中设置软限位可能不会有实际效果, 这里演示接口用法
    cout << "    (演示接口用法, 暂不实际设置)" << endl;
    // robot.setSoftLimit(true, ec, new_limits);  // 启用并设置限位
    // robot.setSoftLimit(false, ec);  // 仅禁用限位

    // 6. 拖动示教 - 启用 (协作机器人专属)
    cout << "\n[6] 拖动示教 (协作机器人专属)..." << endl;
    cout << "    拖动模式选项:" << endl;
    cout << "      空间: jointSpace=轴空间, cartesianSpace=笛卡尔空间" << endl;
    cout << "      类型: translationOnly=仅平移, rotationOnly=仅旋转, freely=自由" << endl;

    // 示例: 笛卡尔空间自由拖动
    cout << "    (演示接口用法)" << endl;
    // robot.enableDrag(
    //     rokae::DragParameter::Space::cartesianSpace,
    //     rokae::DragParameter::Type::freely,
    //     ec
    // );
    // if (!ec) {
    //     cout << "    笛卡尔空间自由拖动已启用" << endl;
    // }

    // 7. 拖动示教 - 禁用
    cout << "\n[7] 禁用拖动示教..." << endl;
    // robot.disableDrag(ec);
    cout << "    (演示接口用法)" << endl;

    // 8. 清除伺服报警
    cout << "\n[8] 清除伺服报警..." << endl;
    robot.clearServoAlarm(ec);
    if (!ec) {
        cout << "    伺服报警已清除 (如果有)" << endl;
    }

    // 9. 安全参数说明
    cout << "\n[9] 安全功能说明..." << endl;
    cout << "    碰撞检测:" << endl;
    cout << "      - 灵敏度越高, 越容易触发碰撞检测" << endl;
    cout << "      - stop0: 快速停止后断电 (最安全)" << endl;
    cout << "      - stop1: 规划停止后断电" << endl;
    cout << "      - stop2: 规划停止后不断电" << endl;
    cout << "      - suppleStop: 柔顺停止 (仅协作机型)" << endl;
    cout << endl;
    cout << "    软限位:" << endl;
    cout << "      - 限制关节运动范围" << endl;
    cout << "      - 超出限位时机器人停止" << endl;
    cout << endl;
    cout << "    拖动示教:" << endl;
    cout << "      - 仅协作机型支持" << endl;
    cout << "      - 可用于路径录制" << endl;

    // 10. 清理
    cout << "\n[10] 清理..." << endl;
    robot.disableCollisionDetection(ec);
    robot.setPowerState(false, ec);
    robot.disconnectFromRobot(ec);

    cout << "\n==========================================" << endl;
    cout << "  示例 7 完成!" << endl;
    cout << "==========================================" << endl;

    return 0;
}
