/**
 * @file 04_motion_basic.cpp
 * @brief 第4.4节 - 基础运动控制示例
 *
 * 这个版本不再演示大跨步的单点跳转，而是采用：
 * - 先进入安全准备位
 * - 再执行带 blending 的多段关节运动
 * - 最后回到安全停靠位
 *
 * 这样更接近真实示教/生产里的基础运动用法，也更容易观察轨迹执行质量。
 */

#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "rokae_xmate3_ros2/robot.hpp"

using namespace std;
using namespace chrono_literals;

namespace {

using JointTarget = array<double, 6>;

void waitForIdle(rokae::ros2::xMateRobot &robot,
                 const array<double, 6> &initial_joints,
                 std::error_code &ec,
                 int timeout_sec = 60) {
    cout << "    等待运动完成..." << endl;
    const auto start = chrono::steady_clock::now();
    bool seen_active_state = false;
    while (chrono::steady_clock::now() - start < chrono::seconds(timeout_sec)) {
        const auto state = robot.operationState(ec);
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

void printJointPos(rokae::ros2::xMateRobot &robot, std::error_code &ec) {
    const auto joints = robot.jointPos(ec);
    if (ec) {
        return;
    }
    cout << "    关节位置: [";
    for (size_t i = 0; i < joints.size(); ++i) {
        cout << fixed << setprecision(4) << joints[i];
        if (i + 1 < joints.size()) {
            cout << ", ";
        }
    }
    cout << "]" << endl;
}

void appendMoveAbsJ(rokae::ros2::xMateRobot &robot,
                    const JointTarget &target,
                    int speed,
                    int zone,
                    std::error_code &ec) {
    rokae::MoveAbsJCommand cmd;
    cmd.target.joints.assign(target.begin(), target.end());
    cmd.speed = speed;
    cmd.zone = zone;
    robot.moveAbsJ(cmd, ec);
}

bool runSequence(rokae::ros2::xMateRobot &robot,
                 const vector<JointTarget> &targets,
                 int speed,
                 int blend_zone,
                 std::error_code &ec,
                 const string &label) {
    cout << "    执行序列: " << label << endl;
    robot.moveReset(ec);
    if (ec) {
        return false;
    }

    for (size_t i = 0; i < targets.size(); ++i) {
        const int zone = (i + 1 == targets.size()) ? 0 : blend_zone;
        appendMoveAbsJ(robot, targets[i], speed, zone, ec);
        if (ec) {
            return false;
        }
    }

    const auto joints_before_start = robot.jointPos(ec);
    if (ec) {
        return false;
    }
    robot.moveStart(ec);
    if (ec) {
        return false;
    }
    waitForIdle(robot, joints_before_start, ec, 90);
    printJointPos(robot, ec);
    return !ec;
}

}  // namespace

int main() {
    cout << "==========================================" << endl;
    cout << "  示例 4: 基础运动控制" << endl;
    cout << "  (优化版 - 更平滑的关节路径执行)" << endl;
    cout << "==========================================" << endl;

    std::error_code ec;

    const JointTarget ready_pose = {0.0, 0.45, 0.08, 0.0, 0.72, 0.0};
    const JointTarget left_pose = {0.20, 0.32, 0.18, 0.12, 0.88, 0.06};
    const JointTarget right_pose = {-0.22, 0.56, 0.12, -0.08, 0.74, -0.06};
    const JointTarget center_pose = {0.0, 0.50, 0.05, 0.0, 0.82, 0.0};
    const JointTarget park_pose = {0.0, 0.35, 0.0, 0.0, 0.65, 0.0};

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
    if (ec) {
        cerr << "初始化失败: " << ec.message() << endl;
        return 1;
    }
    cout << "    已连接、上电、自动模式、非实时指令模式" << endl;

    cout << "\n[2] 设置默认运动参数..." << endl;
    robot.setDefaultSpeed(20, ec);
    robot.setDefaultZone(8, ec);
    robot.setDefaultConfOpt(false, ec);
    if (ec) {
        cerr << "设置默认参数失败: " << ec.message() << endl;
        return 1;
    }
    cout << "    默认速度: 20%, 默认转弯区: 8mm" << endl;

    cout << "\n[3] 当前关节位置..." << endl;
    printJointPos(robot, ec);

    cout << "\n[4] 进入安全准备位..." << endl;
    if (!runSequence(robot, {ready_pose}, 18, 0, ec, "ready pose")) {
        cerr << "准备位运动失败: " << ec.message() << endl;
        return 1;
    }

    cout << "\n[5] 执行带 blending 的基础关节路径..." << endl;
    if (!runSequence(robot, {left_pose, right_pose, center_pose}, 16, 12, ec,
                     "left -> right -> center")) {
        cerr << "基础路径执行失败: " << ec.message() << endl;
        return 1;
    }

    cout << "\n[6] 精确回到安全停靠位..." << endl;
    if (!runSequence(robot, {park_pose}, 14, 0, ec, "park pose")) {
        cerr << "停靠位运动失败: " << ec.message() << endl;
        return 1;
    }

    cout << "\n[7] 清理..." << endl;
    robot.setPowerState(false, ec);
    robot.disconnectFromRobot(ec);

    cout << "\n==========================================" << endl;
    cout << "  示例 4 完成!" << endl;
    cout << "==========================================" << endl;

    return 0;
}
