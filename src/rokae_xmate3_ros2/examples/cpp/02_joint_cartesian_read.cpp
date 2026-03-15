/**
 * @file 02_joint_cartesian_read.cpp
 * @brief 第4.3节 - 关节位置与笛卡尔位姿读取示例
 *
 * 本示例演示：
 * - 读取关节位置
 * - 读取关节速度
 * - 读取关节力矩
 * - 读取笛卡尔位姿
 * - 设置工具坐标系
 */

#include <iostream>
#include <iomanip>
#include <system_error>
#include "rokae_xmate3_ros2/robot.hpp"

using namespace std;

void printJointData(const string& name, const array<double, 6>& data) {
    cout << "    " << name << ": [";
    for (size_t i = 0; i < 6; ++i) {
        cout << fixed << setprecision(4) << data[i];
        if (i < 5) cout << ", ";
    }
    cout << "]" << endl;
}

void printCartesianPose(const rokae::CartesianPosition& pose) {
    cout << "    位置 (m): [" << fixed << setprecision(4)
         << pose.x << ", " << pose.y << ", " << pose.z << "]" << endl;
    cout << "    姿态 (rad): [" << fixed << setprecision(4)
         << pose.rx << ", " << pose.ry << ", " << pose.rz << "]" << endl;
}

int main() {
    cout << "==========================================" << endl;
    cout << "  示例 2: 关节与笛卡尔位姿读取" << endl;
    cout << "  (对应SDK手册第4.3节)" << endl;
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

    // 2. 读取关节位置
    cout << "\n[2] 读取关节位置 (弧度)..." << endl;
    auto joint_pos = robot.jointPos(ec);
    if (!ec) {
        printJointData("关节位置", joint_pos);
    }

    // 3. 读取关节速度
    cout << "\n[3] 读取关节速度 (弧度/秒)..." << endl;
    auto joint_vel = robot.jointVel(ec);
    if (!ec) {
        printJointData("关节速度", joint_vel);
    }

    // 4. 读取关节力矩
    cout << "\n[4] 读取关节力矩 (牛·米)..." << endl;
    auto joint_torque = robot.jointTorque(ec);
    if (!ec) {
        printJointData("关节力矩", joint_torque);
    }

    // 5. 读取法兰在基坐标系的笛卡尔位姿
    cout << "\n[5] 读取法兰笛卡尔位姿 (基坐标系)..." << endl;
    auto flange_pose = robot.cartPosture(rokae::CoordinateType::flangeInBase, ec);
    if (!ec) {
        printCartesianPose(flange_pose);
    }

    // 6. 使用posture()接口读取
    cout << "\n[6] 使用posture()接口读取位姿..." << endl;
    auto posture_data = robot.posture(rokae::CoordinateType::flangeInBase, ec);
    if (!ec) {
        cout << "    [X, Y, Z, Rx, Ry, Rz] = [";
        for (size_t i = 0; i < 6; ++i) {
            cout << fixed << setprecision(4) << posture_data[i];
            if (i < 5) cout << ", ";
        }
        cout << "]" << endl;
    }

    // 7. 读取基坐标系
    cout << "\n[7] 读取基坐标系..." << endl;
    auto base_frame = robot.baseFrame(ec);
    if (!ec) {
        cout << "    基坐标系: [";
        for (size_t i = 0; i < 6; ++i) {
            cout << fixed << setprecision(4) << base_frame[i];
            if (i < 5) cout << ", ";
        }
        cout << "]" << endl;
    }

    // 8. 获取当前工具工件组
    cout << "\n[8] 获取当前工具工件组..." << endl;
    auto toolset = robot.toolset(ec);
    if (!ec) {
        cout << "    工具名: " << toolset.tool_name << endl;
        cout << "    工件名: " << toolset.wobj_name << endl;
    }

    // 9. 设置新的工具工件组（按名称）
    cout << "\n[9] 设置工具工件组 (示例: 名称设置)..." << endl;
    cout << "    (本示例跳过实际设置, 请根据实际情况配置)" << endl;
    // robot.setToolset("tool0", "wobj0", ec);

    // 10. 连续读取5次数据
    cout << "\n[10] 连续读取5次关节位置..." << endl;
    for (int i = 0; i < 5; ++i) {
        auto pos = robot.jointPos(ec);
        if (!ec) {
            cout << "    [" << i+1 << "] J1=" << fixed << setprecision(4) << pos[0]
                 << " rad" << endl;
        }
    }

    // 11. 清理
    cout << "\n[11] 清理..." << endl;
    robot.setPowerState(false, ec);
    robot.disconnectFromRobot(ec);

    cout << "\n==========================================" << endl;
    cout << "  示例 2 完成!" << endl;
    cout << "==========================================" << endl;

    return 0;
}
