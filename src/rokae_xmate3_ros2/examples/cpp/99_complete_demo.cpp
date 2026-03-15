/**
 * @file 99_complete_demo.cpp
 * @brief 综合示例 - 完整演示SDK主要功能
 *
 * 本示例演示：
 * - 完整的初始化流程
 * - 多种运动指令组合
 * - IO控制
 * - 状态监控
 */

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <system_error>
#include "rokae_xmate3_ros2/robot.hpp"

using namespace std;
using namespace chrono_literals;

void printSeparator(const string& title) {
    cout << "\n==========================================" << endl;
    cout << "  " << title << endl;
    cout << "==========================================" << endl;
}

void waitForIdle(rokae::ros2::xMateRobot& robot, std::error_code& ec, int timeout_sec = 60) {
    auto start = chrono::steady_clock::now();
    while (chrono::steady_clock::now() - start < chrono::seconds(timeout_sec)) {
        auto state = robot.operationState(ec);
        if (!ec && state == rokae::OperationState::idle) {
            return;
        }
        this_thread::sleep_for(100ms);
    }
}

void printJointState(rokae::ros2::xMateRobot& robot, std::error_code& ec) {
    auto joints = robot.jointPos(ec);
    auto pose = robot.cartPosture(rokae::CoordinateType::flangeInBase, ec);
    if (!ec) {
        cout << "    关节: [";
        for (size_t i = 0; i < 6; ++i) {
            cout << fixed << setprecision(3) << joints[i];
            if (i < 5) cout << ", ";
        }
        cout << "] rad" << endl;
        cout << "    笛卡尔: [" << fixed << setprecision(3)
             << pose.x << ", " << pose.y << ", " << pose.z << "] m" << endl;
    }
}

int main() {
    cout << R"(
╔═══════════════════════════════════════════════════════════════╗
║     ROKAE xMate3 ROS2 SDK - 综合演示程序                        ║
║     适用于Gazebo仿真环境                                          ║
╚═══════════════════════════════════════════════════════════════╝)" << endl;

    error_code ec;

    // ========== 阶段1: 初始化 ==========
    printSeparator("阶段1: 初始化连接");

    cout << "\n[1.1] 创建机器人对象..." << endl;
    rokae::ros2::xMateRobot robot;
    cout << "      SDK版本: " << rokae::ros2::xMateRobot::sdkVersion() << endl;

    cout << "\n[1.2] 连接到机器人..." << endl;
    robot.connectToRobot(ec);
    if (ec) {
        cerr << "      失败: " << ec.message() << endl;
        return 1;
    }
    cout << "      成功!" << endl;

    cout << "\n[1.3] 获取机器人信息..." << endl;
    auto info = robot.robotInfo(ec);
    if (!ec) {
        cout << "      机型: " << info.type << endl;
        cout << "      固件: " << info.version << endl;
    }

    cout << "\n[1.4] 上电..." << endl;
    robot.setPowerState(true, ec);
    if (ec) {
        cerr << "      上电失败!" << endl;
        return 1;
    }

    cout << "\n[1.5] 设置为自动模式..." << endl;
    robot.setOperateMode(rokae::OperateMode::automatic, ec);

    cout << "\n[1.6] 设置运动控制模式..." << endl;
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);

    cout << "\n[1.7] 设置默认运动参数..." << endl;
    robot.setDefaultSpeed(40, ec);
    robot.setDefaultZone(5, ec);

    cout << "\n[1.8] 初始状态:" << endl;
    printJointState(robot, ec);

    // ========== 阶段2: 基础运动 ==========
    printSeparator("阶段2: 基础运动演示");

    cout << "\n[2.1] MoveAbsJ - 移动到准备位置..." << endl;
    robot.moveReset(ec);
    {
        rokae::MoveAbsJCommand cmd;
        cmd.target.joints = {0.0, 0.4, 0.0, 0.0, 0.4, 0.0};
        cmd.speed = 30;
        cmd.zone = 0;
        robot.moveAbsJ(cmd, ec);
    }
    robot.moveStart(ec);
    if (ec) {
        cerr << "    moveStart failed: " << ec.message() << endl;
    }
    waitForIdle(robot, ec);
    printJointState(robot, ec);

    cout << "\n[2.2] MoveJ - 笛卡尔目标点动..." << endl;
    robot.moveReset(ec);
    {
        rokae::MoveJCommand cmd;
        cmd.target.x = 0.4;
        cmd.target.y = 0.0;
        cmd.target.z = 0.5;
        cmd.target.rx = 3.14159;
        cmd.target.ry = 0.0;
        cmd.target.rz = 0.0;
        cmd.speed = 40;
        cmd.zone = 5;
        robot.moveJ(cmd, ec);
    }
    robot.moveStart(ec);
    if (ec) {
        cerr << "    moveStart failed: " << ec.message() << endl;
    }
    waitForIdle(robot, ec);
    printJointState(robot, ec);

    // ========== 阶段3: 直线运动队列 ==========
    printSeparator("阶段3: 直线运动队列");

    cout << "\n[3.1] 添加多条MoveL指令..." << endl;
    robot.moveReset(ec);

    // 位置1
    {
        rokae::MoveLCommand cmd;
        cmd.target.x = 0.35;
        cmd.target.y = 0.15;
        cmd.target.z = 0.5;
        cmd.target.rx = 3.14159;
        cmd.target.ry = 0.0;
        cmd.target.rz = 0.0;
        cmd.speed = 25;
        cmd.zone = 5;
        robot.moveL(cmd, ec);
    }

    // 位置2
    {
        rokae::MoveLCommand cmd;
        cmd.target.x = 0.35;
        cmd.target.y = -0.15;
        cmd.target.z = 0.5;
        cmd.target.rx = 3.14159;
        cmd.target.ry = 0.0;
        cmd.target.rz = 0.0;
        cmd.speed = 25;
        cmd.zone = 5;
        robot.moveL(cmd, ec);
    }

    // 位置3
    {
        rokae::MoveLCommand cmd;
        cmd.target.x = 0.4;
        cmd.target.y = 0.0;
        cmd.target.z = 0.45;
        cmd.target.rx = 3.14159;
        cmd.target.ry = 0.0;
        cmd.target.rz = 0.0;
        cmd.speed = 25;
        cmd.zone = 0;
        robot.moveL(cmd, ec);
    }

    cout << "      3条指令已添加到队列" << endl;
    robot.moveStart(ec);
    if (ec) {
        cerr << "    moveStart failed: " << ec.message() << endl;
    }
    waitForIdle(robot, ec, 90);
    printJointState(robot, ec);

    // ========== 阶段4: IO控制 ==========
    printSeparator("阶段4: IO控制演示");

    cout << "\n[4.1] 设置DO[0] = ON..." << endl;
    robot.setDO(0, 0, true, ec);
    if (!ec) {
        cout << "      DO[0] = ON" << endl;
    }

    cout << "\n[4.2] 读取DO状态..." << endl;
    for (unsigned int i = 0; i < 4; ++i) {
        bool state = robot.getDO(0, i, ec);
        if (!ec) {
            cout << "      DO[" << i << "] = " << (state ? "ON" : "OFF") << endl;
        }
    }

    cout << "\n[4.3] 关闭所有DO..." << endl;
    for (unsigned int i = 0; i < 4; ++i) {
        robot.setDO(0, i, false, ec);
    }

    // ========== 阶段5: 运动学计算 ==========
    printSeparator("阶段5: 运动学计算");

    cout << "\n[5.1] 当前关节位置正运动学..." << endl;
    auto current_joints = robot.jointPos(ec);
    if (!ec) {
        rokae::JointPosition jp;
        jp.joints = {current_joints[0], current_joints[1], current_joints[2],
                      current_joints[3], current_joints[4], current_joints[5]};
        auto fk = robot.calcFk(jp, ec);
        if (!ec) {
            cout << "      FK: X=" << fixed << setprecision(4) << fk.x
                 << ", Y=" << fk.y << ", Z=" << fk.z << endl;
        }
    }

    cout << "\n[5.2] 逆运动学计算测试..." << endl;
    {
        rokae::CartesianPosition target;
        target.x = 0.35;
        target.y = 0.1;
        target.z = 0.45;
        target.rx = 3.14159;
        target.ry = 0.0;
        target.rz = 0.0;
        auto ik = robot.calcIk(target, ec);
        if (!ec && ik.joints.size() >= 6) {
            cout << "      IK结果: [";
            for (size_t i = 0; i < 6; ++i) {
                cout << fixed << setprecision(4) << ik.joints[i];
                if (i < 5) cout << ", ";
            }
            cout << "]" << endl;
        }
    }

    // ========== 阶段6: 返回零位 ==========
    printSeparator("阶段6: 返回零位");

    cout << "\n[6.1] MoveAbsJ - 返回零位..." << endl;
    robot.moveReset(ec);
    {
        rokae::MoveAbsJCommand cmd;
        cmd.target.joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        cmd.speed = 30;
        cmd.zone = 0;
        robot.moveAbsJ(cmd, ec);
    }
    robot.moveStart(ec);
    if (ec) {
        cerr << "    moveStart failed: " << ec.message() << endl;
    }
    waitForIdle(robot, ec);
    cout << "\n      已回到零位:" << endl;
    printJointState(robot, ec);

    // ========== 阶段7: 清理 ==========
    printSeparator("阶段7: 清理退出");

    cout << "\n[7.1] 下电..." << endl;
    robot.setPowerState(false, ec);

    cout << "\n[7.2] 断开连接..." << endl;
    robot.disconnectFromRobot(ec);

    cout << R"(

╔═══════════════════════════════════════════════════════════════╗
║                        演示完成!                                  ║
║     更多示例请参考 examples/cpp/ 目录下的其他文件               ║
╚═══════════════════════════════════════════════════════════════╝
)" << endl;

    return 0;
}
