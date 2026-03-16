/**
 * @file 05_motion_cartesian.cpp
 * @brief 第4.4节 - 笛卡尔空间运动示例
 *
 * 本示例演示：
 * - MoveJ: 轴空间运动（笛卡尔目标）
 * - MoveL: 直线运动
 * - MoveC: 圆弧运动
 * - 多个指令的队列执行
 */

#include <array>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cmath>
#include <system_error>
#include "rokae_xmate3_ros2/robot.hpp"

using namespace std;
using namespace chrono_literals;

void waitForIdle(rokae::ros2::xMateRobot& robot,
                 const array<double, 6>& initial_joints,
                 std::error_code& ec,
                 int timeout_sec = 60) {
    cout << "    等待运动完成..." << endl;
    auto start = chrono::steady_clock::now();
    bool seen_active_state = false;
    while (chrono::steady_clock::now() - start < chrono::seconds(timeout_sec)) {
        auto state = robot.operationState(ec);
        if (ec) {
            return;
        }
        const auto current_joints = robot.jointPos(ec);
        if (ec) {
            return;
        }
        bool joint_changed = false;
        for (size_t i = 0; i < initial_joints.size(); ++i) {
            if (fabs(current_joints[i] - initial_joints[i]) > 1e-3) {
                joint_changed = true;
                break;
            }
        }
        if (state != rokae::OperationState::idle && state != rokae::OperationState::unknown) {
            seen_active_state = true;
        }
        if ((seen_active_state && state == rokae::OperationState::idle) ||
            (!seen_active_state && joint_changed && state == rokae::OperationState::idle)) {
            cout << "    运动完成!" << endl;
            return;
        }
        this_thread::sleep_for(100ms);
    }
    ec = make_error_code(errc::timed_out);
    cout << "    等待超时!" << endl;
}

void printCartesianPos(rokae::ros2::xMateRobot& robot, std::error_code& ec) {
    auto pose = robot.cartPosture(rokae::CoordinateType::flangeInBase, ec);
    if (!ec) {
        cout << "    笛卡尔位姿: ["
             << fixed << setprecision(4) << pose.x << ", "
             << fixed << setprecision(4) << pose.y << ", "
             << fixed << setprecision(4) << pose.z << "] m | ["
             << fixed << setprecision(4) << pose.rx << ", "
             << fixed << setprecision(4) << pose.ry << ", "
             << fixed << setprecision(4) << pose.rz << "] rad" << endl;
    }
}

int main() {
    cout << "==========================================" << endl;
    cout << "  示例 5: 笛卡尔空间运动" << endl;
    cout << "  (对应SDK手册第4.4节)" << endl;
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
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    cout << "    初始化完成" << endl;

    // 2. 设置默认参数
    cout << "\n[2] 设置默认参数..." << endl;
    robot.setDefaultSpeed(30, ec);
    robot.setDefaultZone(5, ec);
    cout << "    速度: 30%, 转弯区: 5mm" << endl;

    // 3. 先移动到起始位置 (用MoveAbsJ)
    cout << "\n[3] 移动到起始位置..." << endl;
    robot.moveReset(ec);
    rokae::MoveAbsJCommand start_cmd;
    start_cmd.target.joints = {0.0, 0.5, 0.0, 0.0, 0.5, 0.0};
    start_cmd.speed = 30;
    start_cmd.zone = 0;
    robot.moveAbsJ(start_cmd, ec);
    const auto start_joints = robot.jointPos(ec);
    if (ec) {
        cerr << "读取起始关节失败: " << ec.message() << endl;
        return 1;
    }
    robot.moveStart(ec);
    if (ec) {
        cerr << "启动失败: " << ec.message() << endl;
        return 1;
    }
    waitForIdle(robot, start_joints, ec);
    if (ec) {
        cerr << "起始位运动失败: " << ec.message() << endl;
        return 1;
    }

    cout << "    起始位置:" << endl;
    printCartesianPos(robot, ec);

    // 4. MoveJ - 轴空间运动（笛卡尔目标）
    cout << "\n[4] MoveJ - 轴空间运动（笛卡尔目标）..." << endl;
    robot.moveReset(ec);

    rokae::MoveJCommand j_cmd;
    j_cmd.target.x = 0.4;
    j_cmd.target.y = 0.0;
    j_cmd.target.z = 0.5;
    j_cmd.target.rx = 3.14159;
    j_cmd.target.ry = 0.0;
    j_cmd.target.rz = 0.0;
    j_cmd.speed = 40;
    j_cmd.zone = 5;

    robot.moveJ(j_cmd, ec);
    if (!ec) {
        cout << "    MoveJ指令已添加" << endl;
    }

    const auto movej_start_joints = robot.jointPos(ec);
    if (ec) {
        cerr << "读取MoveJ起始关节失败: " << ec.message() << endl;
        return 1;
    }
    robot.moveStart(ec);
    if (ec) {
        cerr << "MoveJ启动失败: " << ec.message() << endl;
        return 1;
    }
    waitForIdle(robot, movej_start_joints, ec);
    if (ec) {
        cerr << "MoveJ执行失败: " << ec.message() << endl;
        return 1;
    }

    cout << "    MoveJ后位置:" << endl;
    printCartesianPos(robot, ec);

    // 5. MoveL - 直线运动
    cout << "\n[5] MoveL - 直线运动..." << endl;
    robot.moveReset(ec);

    rokae::MoveLCommand l_cmd1;
    l_cmd1.target.x = 0.3;
    l_cmd1.target.y = 0.2;
    l_cmd1.target.z = 0.5;
    l_cmd1.target.rx = 3.14159;
    l_cmd1.target.ry = 0.0;
    l_cmd1.target.rz = 0.0;
    l_cmd1.speed = 20;  // 直线运动速度稍慢
    l_cmd1.zone = 5;

    robot.moveL(l_cmd1, ec);
    cout << "    MoveL指令1已添加" << endl;

    const auto movel_start_joints = robot.jointPos(ec);
    if (ec) {
        cerr << "读取MoveL起始关节失败: " << ec.message() << endl;
        return 1;
    }
    robot.moveStart(ec);
    if (ec) {
        cerr << "MoveL启动失败: " << ec.message() << endl;
        return 1;
    }
    waitForIdle(robot, movel_start_joints, ec);
    if (ec) {
        cerr << "MoveL执行失败: " << ec.message() << endl;
        return 1;
    }

    cout << "    MoveL后位置:" << endl;
    printCartesianPos(robot, ec);

    // 6. 多条MoveL指令组成队列
    cout << "\n[6] 多条MoveL指令队列..." << endl;
    robot.moveReset(ec);

    // 位置1
    rokae::MoveLCommand l1;
    l1.target.x = 0.4;
    l1.target.y = 0.1;
    l1.target.z = 0.5;
    l1.target.rx = 3.14159;
    l1.target.ry = 0.0;
    l1.target.rz = 0.0;
    l1.speed = 20;
    l1.zone = 5;
    robot.moveL(l1, ec);

    // 位置2
    rokae::MoveLCommand l2;
    l2.target.x = 0.4;
    l2.target.y = -0.1;
    l2.target.z = 0.5;
    l2.target.rx = 3.14159;
    l2.target.ry = 0.0;
    l2.target.rz = 0.0;
    l2.speed = 20;
    l2.zone = 5;
    robot.moveL(l2, ec);

    // 位置3 - 回到中间
    rokae::MoveLCommand l3;
    l3.target.x = 0.4;
    l3.target.y = 0.0;
    l3.target.z = 0.4;
    l3.target.rx = 3.14159;
    l3.target.ry = 0.0;
    l3.target.rz = 0.0;
    l3.speed = 20;
    l3.zone = 0;  // 最后一点精确到位
    robot.moveL(l3, ec);

    cout << "    3条MoveL指令已添加到队列" << endl;

    const auto movel_queue_start_joints = robot.jointPos(ec);
    if (ec) {
        cerr << "读取MoveL队列起始关节失败: " << ec.message() << endl;
        return 1;
    }
    robot.moveStart(ec);
    if (ec) {
        cerr << "MoveL队列启动失败: " << ec.message() << endl;
        return 1;
    }
    waitForIdle(robot, movel_queue_start_joints, ec, 90);
    if (ec) {
        cerr << "MoveL队列执行失败: " << ec.message() << endl;
        return 1;
    }

    cout << "    队列执行完毕位置:" << endl;
    printCartesianPos(robot, ec);

    // 7. MoveC - 圆弧运动
    cout << "\n[7] MoveC - 圆弧运动 (可选)..." << endl;
    cout << "    (本示例演示接口用法, 实际执行取决于仿真支持)" << endl;

    robot.moveReset(ec);

    rokae::MoveCCommand c_cmd;
    // 起始点（当前位置）-> 辅助点 -> 目标点
    c_cmd.aux.x = 0.45;
    c_cmd.aux.y = 0.1;
    c_cmd.aux.z = 0.4;
    c_cmd.aux.rx = 3.14159;
    c_cmd.aux.ry = 0.0;
    c_cmd.aux.rz = 0.0;

    c_cmd.target.x = 0.45;
    c_cmd.target.y = 0.0;
    c_cmd.target.z = 0.45;
    c_cmd.target.rx = 3.14159;
    c_cmd.target.ry = 0.0;
    c_cmd.target.rz = 0.0;

    c_cmd.speed = 20;
    c_cmd.zone = 0;

    // robot.moveC(c_cmd, ec);  // 取消注释以实际执行
    // robot.moveStart(ec);
    // waitForIdle(robot, ec);

    cout << "    MoveC接口说明: aux为圆弧辅助点, target为目标点" << endl;

    // 8. 回到零位
    cout << "\n[8] 回到零位..." << endl;
    robot.moveReset(ec);

    rokae::MoveAbsJCommand home_cmd;
    home_cmd.target.joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    home_cmd.speed = 30;
    home_cmd.zone = 0;
    robot.moveAbsJ(home_cmd, ec);
    const auto home_start_joints = robot.jointPos(ec);
    if (ec) {
        cerr << "读取回零起始关节失败: " << ec.message() << endl;
        return 1;
    }
    robot.moveStart(ec);
    if (ec) {
        cerr << "回零启动失败: " << ec.message() << endl;
        return 1;
    }
    waitForIdle(robot, home_start_joints, ec);
    if (ec) {
        cerr << "回零执行失败: " << ec.message() << endl;
        return 1;
    }

    // 9. 清理
    cout << "\n[9] 清理..." << endl;
    robot.setPowerState(false, ec);
    robot.disconnectFromRobot(ec);

    cout << "\n==========================================" << endl;
    cout << "  示例 5 完成!" << endl;
    cout << "==========================================" << endl;

    return 0;
}
