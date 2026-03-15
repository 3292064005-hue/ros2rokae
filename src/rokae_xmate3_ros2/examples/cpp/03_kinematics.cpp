/**
 * @file 03_kinematics.cpp
 * @brief 第4.3节 - 运动学计算示例
 *
 * 本示例演示：
 * - 正运动学计算 (FK)
 * - 逆运动学计算 (IK)
 */

#include <iostream>
#include <iomanip>
#include <array>
#include <system_error>
#include "rokae_xmate3_ros2/robot.hpp"

using namespace std;

void printJoints(const array<double, 6>& joints) {
    cout << "    [";
    for (size_t i = 0; i < 6; ++i) {
        cout << fixed << setprecision(4) << joints[i];
        if (i < 5) cout << ", ";
    }
    cout << "] (rad)" << endl;
}

void printCartesian(const rokae::CartesianPosition& pose) {
    cout << "    位置: [" << fixed << setprecision(4)
         << pose.x << ", " << pose.y << ", " << pose.z << "] m" << endl;
    cout << "    姿态: [" << fixed << setprecision(4)
         << pose.rx << ", " << pose.ry << ", " << pose.rz << "] rad" << endl;
}

int main() {
    cout << "==========================================" << endl;
    cout << "  示例 3: 运动学计算 (FK/IK)" << endl;
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
    cout << "    已连接" << endl;

    // 2. 获取当前关节位置
    cout << "\n[2] 获取当前关节位置..." << endl;
    auto current_joints = robot.jointPos(ec);
    if (!ec) {
        cout << "    当前关节: ";
        printJoints(current_joints);
    }

    // 3. 正运动学计算 (FK) - 使用当前关节
    cout << "\n[3] 正运动学计算 (FK)..." << endl;
    rokae::JointPosition joint_input;
    joint_input.joints = {current_joints[0], current_joints[1], current_joints[2],
                          current_joints[3], current_joints[4], current_joints[5]};

    auto fk_result = robot.calcFk(joint_input, ec);
    if (!ec) {
        cout << "    FK计算结果:" << endl;
        printCartesian(fk_result);
    }

    // 4. 使用指定关节进行FK计算
    cout << "\n[4] 使用指定关节进行FK计算..." << endl;
    rokae::JointPosition test_joints;
    test_joints.joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 零位
    cout << "    输入关节 (零位): ";
    printJoints({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    auto fk_zero = robot.calcFk(test_joints, ec);
    if (!ec) {
        cout << "    FK结果 (零位):" << endl;
        printCartesian(fk_zero);
    }

    // 5. 另一个测试位置
    cout << "\n[5] 另一个测试位置的FK计算..." << endl;
    rokae::JointPosition test_joints2;
    test_joints2.joints = {0.5, 0.3, -0.2, 0.1, 0.4, 0.0};
    cout << "    输入关节: ";
    printJoints({0.5, 0.3, -0.2, 0.1, 0.4, 0.0});

    auto fk_test = robot.calcFk(test_joints2, ec);
    if (!ec) {
        cout << "    FK结果:" << endl;
        printCartesian(fk_test);
    }

    // 6. 逆运动学计算 (IK)
    cout << "\n[6] 逆运动学计算 (IK)..." << endl;
    rokae::CartesianPosition target_pose;
    target_pose.x = 0.4;
    target_pose.y = 0.0;
    target_pose.z = 0.5;
    target_pose.rx = 3.14159;  // PI
    target_pose.ry = 0.0;
    target_pose.rz = 0.0;

    cout << "    目标笛卡尔位姿:" << endl;
    printCartesian(target_pose);

    auto ik_result = robot.calcIk(target_pose, ec);
    if (!ec) {
        cout << "    IK计算结果 (关节角度):" << endl;
        cout << "    [";
        for (size_t i = 0; i < ik_result.joints.size() && i < 6; ++i) {
            cout << fixed << setprecision(4) << ik_result.joints[i];
            if (i < 5) cout << ", ";
        }
        cout << "] (rad)" << endl;
    } else {
        cout << "    IK计算失败: " << ec.message() << endl;
    }

    // 7. 使用FK验证IK结果
    cout << "\n[7] FK/IK 一致性验证..." << endl;
    if (!ec && ik_result.joints.size() >= 6) {
        rokae::JointPosition ik_joints;
        ik_joints.joints = ik_result.joints;
        auto verify_fk = robot.calcFk(ik_joints, ec);
        if (!ec) {
            cout << "    验证: IK结果的FK计算:" << endl;
            printCartesian(verify_fk);
            cout << "    误差 (X): " << fixed << setprecision(6)
                 << abs(verify_fk.x - target_pose.x) << " m" << endl;
        }
    }

    // 8. 另一个IK目标
    cout << "\n[8] 另一个IK目标..." << endl;
    rokae::CartesianPosition target_pose2;
    target_pose2.x = 0.3;
    target_pose2.y = 0.2;
    target_pose2.z = 0.4;
    target_pose2.rx = 3.14159;
    target_pose2.ry = 0.0;
    target_pose2.rz = 0.7854;  // PI/4

    cout << "    目标位姿 (带旋转):" << endl;
    printCartesian(target_pose2);

    auto ik_result2 = robot.calcIk(target_pose2, ec);
    if (!ec) {
        cout << "    IK结果:" << endl;
        cout << "    [";
        for (size_t i = 0; i < ik_result2.joints.size() && i < 6; ++i) {
            cout << fixed << setprecision(4) << ik_result2.joints[i];
            if (i < 5) cout << ", ";
        }
        cout << "] (rad)" << endl;
    }

    // 9. 清理
    cout << "\n[9] 清理..." << endl;
    robot.disconnectFromRobot(ec);

    cout << "\n==========================================" << endl;
    cout << "  示例 3 完成!" << endl;
    cout << "==========================================" << endl;

    return 0;
}
