/**
 * @file 04_motion_basic.cpp
 * @brief 第4.4节 - 基础运动控制示例
 *
 * 本示例演示：
 * - 设置运动控制模式
 * - 运动队列重置
 * - 设置默认速度和转弯区
 * - MoveAbsJ: 轴空间绝对运动
 * - 启动/停止运动
 */

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <system_error>
#include "rokae_xmate3_ros2/robot.hpp"

using namespace std;
using namespace chrono_literals;

void waitForIdle(rokae::ros2::xMateRobot& robot, std::error_code& ec, int timeout_sec = 30) {
    cout << "    等待运动完成..." << endl;
    auto start = chrono::steady_clock::now();
    while (chrono::steady_clock::now() - start < chrono::seconds(timeout_sec)) {
        auto state = robot.operationState(ec);
        if (!ec && state == rokae::OperationState::idle) {
            cout << "    运动完成!" << endl;
            return;
        }
        this_thread::sleep_for(100ms);
    }
    cout << "    等待超时!" << endl;
}

void printJointPos(rokae::ros2::xMateRobot& robot, std::error_code& ec) {
    auto joints = robot.jointPos(ec);
    if (!ec) {
        cout << "    关节位置: [";
        for (size_t i = 0; i < 6; ++i) {
            cout << fixed << setprecision(4) << joints[i];
            if (i < 5) cout << ", ";
        }
        cout << "]" << endl;
    }
}

int main() {
    cout << "==========================================" << endl;
    cout << "  示例 4: 基础运动控制" << endl;
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
    cout << "    已连接、上电、自动模式" << endl;

    // 2. 设置运动控制模式为非实时指令模式
    cout << "\n[2] 设置运动控制模式..." << endl;
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    if (!ec) {
        cout << "    已设置为非实时指令模式 (NrtCommand)" << endl;
    }

    // 3. 重置运动队列
    cout << "\n[3] 重置运动队列..." << endl;
    robot.moveReset(ec);
    cout << "    运动队列已清空" << endl;

    // 4. 设置默认运动参数
    cout << "\n[4] 设置默认运动参数..." << endl;
    robot.setDefaultSpeed(50, ec);   // 50% 速度
    robot.setDefaultZone(5, ec);     // 5mm 转弯区
    robot.setDefaultConfOpt(false, ec);  // 不强制轴配置
    cout << "    速度: 50%, 转弯区: 5mm" << endl;

    // 5. 获取初始位置
    cout << "\n[5] 初始位置..." << endl;
    printJointPos(robot, ec);

    // 6. MoveAbsJ - 轴空间绝对运动
    cout << "\n[6] MoveAbsJ - 轴空间绝对运动..." << endl;
    rokae::MoveAbsJCommand absj_cmd;
    absj_cmd.target.joints = {0.3, 0.2, -0.1, 0.0, 0.3, 0.0};
    absj_cmd.speed = 30;   // 30% 速度
    absj_cmd.zone = 0;      // 精确到位 (fine point)

    robot.moveAbsJ(absj_cmd, ec);
    if (!ec) {
        cout << "    MoveAbsJ指令已添加到队列" << endl;
    }

    // 7. 启动运动
    cout << "\n[7] 启动运动执行..." << endl;
    robot.moveStart(ec);
    if (!ec) {
        cout << "    运动已启动" << endl;
        waitForIdle(robot, ec);
    }

    // 8. 显示新位置
    cout << "\n[8] 运动后位置..." << endl;
    printJointPos(robot, ec);

    // 9. 第二次MoveAbsJ - 移动到另一个位置
    cout << "\n[9] 第二次MoveAbsJ..." << endl;
    robot.moveReset(ec);

    rokae::MoveAbsJCommand absj_cmd2;
    absj_cmd2.target.joints = {-0.2, 0.4, 0.1, 0.2, -0.3, 0.1};
    absj_cmd2.speed = 40;
    absj_cmd2.zone = 5;

    robot.moveAbsJ(absj_cmd2, ec);
    robot.moveStart(ec);
    waitForIdle(robot, ec);

    cout << "    第二个位置:" << endl;
    printJointPos(robot, ec);

    // 10. 回到零位
    cout << "\n[10] 回到零位..." << endl;
    robot.moveReset(ec);

    rokae::MoveAbsJCommand home_cmd;
    home_cmd.target.joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    home_cmd.speed = 30;
    home_cmd.zone = 0;

    robot.moveAbsJ(home_cmd, ec);
    robot.moveStart(ec);
    waitForIdle(robot, ec);

    cout << "    已回到零位:" << endl;
    printJointPos(robot, ec);

    // 11. 测试在线速度调整
    cout << "\n[11] 测试在线速度调整 (可选)..." << endl;
    cout << "    (本示例跳过实际调整)" << endl;
    // robot.adjustSpeedOnline(0.5, ec);  // 降速到50%

    // 12. 清理
    cout << "\n[12] 清理..." << endl;
    robot.setPowerState(false, ec);
    robot.disconnectFromRobot(ec);

    cout << "\n==========================================" << endl;
    cout << "  示例 4 完成!" << endl;
    cout << "==========================================" << endl;

    return 0;
}
