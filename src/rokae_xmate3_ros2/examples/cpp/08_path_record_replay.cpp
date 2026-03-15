/**
 * @file 08_path_record_replay.cpp
 * @brief 第4.8节 - 路径录制与回放示例
 *
 * 本示例演示：
 * - 开始录制路径
 * - 停止录制路径
 * - 保存录制的路径
 * - 回放路径
 * - 查询路径列表
 * - 删除路径
 */

#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <system_error>
#include "rokae_xmate3_ros2/robot.hpp"

using namespace std;
using namespace chrono_literals;

int main() {
    cout << "==========================================" << endl;
    cout << "  示例 8: 路径录制与回放" << endl;
    cout << "  (对应SDK手册第4.8节)" << endl;
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

    // 2. 查询已保存的路径列表
    cout << "\n[2] 查询已保存的路径列表..." << endl;
    auto paths = robot.queryPathLists(ec);
    if (!ec) {
        cout << "    找到 " << paths.size() << " 个路径:" << endl;
        for (size_t i = 0; i < paths.size(); ++i) {
            cout << "      [" << i << "] " << paths[i] << endl;
        }
    }

    // 3. 开始录制路径
    cout << "\n[3] 开始录制路径..." << endl;
    cout << "    录制时长: 10 秒" << endl;

    auto record_duration = 10s;
    robot.startRecordPath(chrono::duration_cast<chrono::seconds>(record_duration), ec);
    if (!ec) {
        cout << "    路径录制已开始" << endl;

        // 等待录制（这里可以进行拖动示教或其他运动）
        cout << "    录制中..." << endl;
        cout << "    (在真实机器人上, 此时可以拖动机器人)" << endl;

        // 模拟等待
        for (int i = 0; i < 5; ++i) {
            cout << "      ... " << (i + 1) * 2 << "s" << endl;
            this_thread::sleep_for(2s);
        }
    }

    // 4. 停止录制路径
    cout << "\n[4] 停止录制路径..." << endl;
    robot.stopRecordPath(ec);
    if (!ec) {
        cout << "    路径录制已停止" << endl;
    }

    // 5. 保存录制的路径
    cout << "\n[5] 保存录制的路径..." << endl;
    string path_name = "my_demo_path";
    robot.saveRecordPath(path_name, ec);
    if (!ec) {
        cout << "    路径已保存为: " << path_name << endl;
    }

    // 6. 再次查询路径列表
    cout << "\n[6] 再次查询路径列表..." << endl;
    paths = robot.queryPathLists(ec);
    if (!ec) {
        cout << "    当前路径列表:" << endl;
        for (size_t i = 0; i < paths.size(); ++i) {
            cout << "      [" << i << "] " << paths[i] << endl;
        }
    }

    // 7. 回放路径
    cout << "\n[7] 回放路径..." << endl;
    double replay_rate = 1.0;  // 1.0倍速
    cout << "    回放路径: " << path_name << endl;
    cout << "    回放速度: " << replay_rate << "x" << endl;

    robot.replayPath(path_name, replay_rate, ec);
    if (!ec) {
        cout << "    路径回放已开始" << endl;
        // 这里可以等待回放完成
    }

    // 8. 取消录制（如果需要）
    cout << "\n[8] 取消录制接口说明..." << endl;
    cout << "    如果在录制过程中想要取消, 使用:" << endl;
    cout << "      robot.cancelRecordPath(ec);" << endl;

    // 9. 删除路径
    cout << "\n[9] 删除路径 (演示)..." << endl;
    cout << "    接口用法:" << endl;
    cout << "      // 删除单个路径" << endl;
    cout << "      robot.removePath(path_name, ec);" << endl;
    cout << "      // 删除所有路径" << endl;
    cout << "      robot.removePath(\"\", ec, true);" << endl;
    cout << "    (本示例保留路径, 不执行删除)" << endl;

    // 10. 路径录制与回放说明
    cout << "\n[10] 路径录制与回放说明..." << endl;
    cout << "    录制流程:" << endl;
    cout << "      1. startRecordPath(duration) - 开始录制" << endl;
    cout << "      2. 拖动机器人或执行运动" << endl;
    cout << "      3. stopRecordPath() - 停止录制" << endl;
    cout << "      4. saveRecordPath(name) - 保存路径" << endl;
    cout << endl;
    cout << "    回放流程:" << endl;
    cout << "      1. queryPathLists() - 查询可用路径" << endl;
    cout << "      2. replayPath(name, rate) - 回放路径" << endl;
    cout << endl;
    cout << "    注意事项:" << endl;
    cout << "      - 路径录制通常与拖动示教配合使用" << endl;
    cout << "      - 回放速度可调整 (0.5x ~ 2.0x)" << endl;
    cout << "      - 未保存的路径会在停止录制后丢失" << endl;

    // 11. 清理
    cout << "\n[11] 清理..." << endl;
    robot.setPowerState(false, ec);
    robot.disconnectFromRobot(ec);

    cout << "\n==========================================" << endl;
    cout << "  示例 8 完成!" << endl;
    cout << "==========================================" << endl;

    return 0;
}
