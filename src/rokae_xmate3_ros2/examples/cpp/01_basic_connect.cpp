/**
 * @file 01_basic_connect.cpp
 * @brief 第4.3节 - 机器人基本连接与信息查询示例
 *
 * 本示例演示：
 * - 创建机器人对象
 * - 连接/断开机器人
 * - 获取机器人信息
 * - 电源控制
 * - 操作模式设置
 */

#include <iostream>
#include <system_error>
#include "rokae_xmate3_ros2/robot.hpp"

using namespace std;

int main() {
    cout << "==========================================" << endl;
    cout << "  示例 1: 机器人基本连接与信息查询" << endl;
    cout << "  (对应SDK手册第4.3节)" << endl;
    cout << "==========================================" << endl;

    error_code ec;

    // 1. 创建机器人对象
    cout << "\n[1] 创建机器人对象..." << endl;
    rokae::ros2::xMateRobot robot;
    cout << "    SDK版本: " << rokae::ros2::xMateRobot::sdkVersion() << endl;

    // 2. 连接到机器人
    cout << "\n[2] 连接机器人..." << endl;
    robot.connectToRobot(ec);
    if (ec) {
        cerr << "    连接失败: " << ec.message() << endl;
        return 1;
    }
    cout << "    连接成功!" << endl;

    // 3. 获取机器人基本信息
    cout << "\n[3] 获取机器人信息..." << endl;
    auto info = robot.robotInfo(ec);
    if (!ec) {
        cout << "    机器人ID: " << info.id << endl;
        cout << "    机型: " << info.type << endl;
        cout << "    固件版本: " << info.version << endl;
        cout << "    关节数: " << info.joint_num << endl;
    }

    // 4. 获取电源状态
    cout << "\n[4] 获取电源状态..." << endl;
    auto power_state = robot.powerState(ec);
    if (!ec) {
        cout << "    当前电源状态: ";
        switch (power_state) {
            case rokae::PowerState::off: cout << "下电"; break;
            case rokae::PowerState::on: cout << "上电"; break;
            case rokae::PowerState::estop: cout << "急停"; break;
            case rokae::PowerState::gstop: cout << "安全门"; break;
            default: cout << "未知"; break;
        }
        cout << endl;
    }

    // 5. 上电
    cout << "\n[5] 机器人上电..." << endl;
    robot.setPowerState(true, ec);
    if (ec) {
        cerr << "    上电失败: " << ec.message() << endl;
    } else {
        cout << "    上电成功!" << endl;
    }

    // 6. 获取操作模式
    cout << "\n[6] 获取操作模式..." << endl;
    auto op_mode = robot.operateMode(ec);
    if (!ec) {
        cout << "    当前操作模式: ";
        switch (op_mode) {
            case rokae::OperateMode::manual: cout << "手动"; break;
            case rokae::OperateMode::automatic: cout << "自动"; break;
            default: cout << "未知"; break;
        }
        cout << endl;
    }

    // 7. 设置操作模式为自动
    cout << "\n[7] 设置操作模式为自动..." << endl;
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    if (!ec) {
        cout << "    操作模式已设置为自动" << endl;
    }

    // 8. 获取操作状态
    cout << "\n[8] 获取操作状态..." << endl;
    auto op_state = robot.operationState(ec);
    if (!ec) {
        cout << "    当前操作状态: ";
        switch (op_state) {
            case rokae::OperationState::idle: cout << "空闲"; break;
            case rokae::OperationState::moving: cout << "运动中"; break;
            case rokae::OperationState::jog: cout << "点动"; break;
            case rokae::OperationState::drag: cout << "拖动"; break;
            default: cout << "未知"; break;
        }
        cout << endl;
    }

    // 9. 查询控制器日志
    cout << "\n[9] 查询最近3条控制器日志..." << endl;
    auto logs = robot.queryControllerLog(3, ec);
    if (!ec && !logs.empty()) {
        cout << "    找到 " << logs.size() << " 条日志:" << endl;
        for (size_t i = 0; i < logs.size(); ++i) {
            cout << "      [" << i << "] " << logs[i].content << endl;
        }
    }

    // 10. 清理 - 下电并断开连接
    cout << "\n[10] 清理..." << endl;
    robot.setPowerState(false, ec);
    robot.disconnectFromRobot(ec);
    cout << "    已下电并断开连接" << endl;

    cout << "\n==========================================" << endl;
    cout << "  示例 1 完成!" << endl;
    cout << "==========================================" << endl;

    return 0;
}
