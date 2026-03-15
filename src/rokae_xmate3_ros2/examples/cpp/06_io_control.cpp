/**
 * @file 06_io_control.cpp
 * @brief 第4.6节 - IO控制示例
 *
 * 本示例演示：
 * - 数字输入 (DI) 读取
 * - 数字输出 (DO) 读取/设置
 * - 模拟输入 (AI) 读取
 * - 模拟输出 (AO) 读取/设置
 * - 仿真模式设置
 */

#include <iostream>
#include <iomanip>
#include <system_error>
#include "rokae_xmate3_ros2/robot.hpp"

using namespace std;

int main() {
    cout << "==========================================" << endl;
    cout << "  示例 6: IO控制" << endl;
    cout << "  (对应SDK手册第4.6节)" << endl;
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

    // 2. 设置仿真模式
    cout << "\n[2] 设置仿真模式..." << endl;
    robot.setSimulationMode(true, ec);
    if (!ec) {
        cout << "    仿真模式已启用" << endl;
    }

    // 3. 读取数字输入 (DI)
    cout << "\n[3] 读取数字输入 (DI)..." << endl;
    cout << "    控制板0, 端口0-7:" << endl;
    for (unsigned int port = 0; port < 8; ++port) {
        bool di_state = robot.getDI(0, port, ec);
        if (!ec) {
            cout << "      DI[" << port << "] = " << (di_state ? "ON" : "OFF") << endl;
        }
    }

    // 4. 读取数字输出 (DO)
    cout << "\n[4] 读取数字输出 (DO)..." << endl;
    cout << "    控制板0, 端口0-7:" << endl;
    for (unsigned int port = 0; port < 8; ++port) {
        bool do_state = robot.getDO(0, port, ec);
        if (!ec) {
            cout << "      DO[" << port << "] = " << (do_state ? "ON" : "OFF") << endl;
        }
    }

    // 5. 设置数字输出 (DO)
    cout << "\n[5] 设置数字输出 (DO)..." << endl;
    unsigned int test_port = 0;

    // 设置为ON
    cout << "    设置 DO[" << test_port << "] = ON..." << endl;
    robot.setDO(0, test_port, true, ec);
    if (!ec) {
        bool state = robot.getDO(0, test_port, ec);
        cout << "      读取: DO[" << test_port << "] = " << (state ? "ON" : "OFF") << endl;
    }

    // 设置为OFF
    cout << "    设置 DO[" << test_port << "] = OFF..." << endl;
    robot.setDO(0, test_port, false, ec);
    if (!ec) {
        bool state = robot.getDO(0, test_port, ec);
        cout << "      读取: DO[" << test_port << "] = " << (state ? "ON" : "OFF") << endl;
    }

    // 6. 读取模拟输入 (AI)
    cout << "\n[6] 读取模拟输入 (AI)..." << endl;
    cout << "    控制板0, 端口0-3:" << endl;
    for (unsigned int port = 0; port < 4; ++port) {
        double ai_value = robot.getAI(0, port, ec);
        if (!ec) {
            cout << "      AI[" << port << "] = " << fixed << setprecision(4) << ai_value << " V" << endl;
        }
    }

    // 7. 设置模拟输出 (AO)
    cout << "\n[7] 设置模拟输出 (AO)..." << endl;
    unsigned int ao_port = 0;
    double target_value = 2.5;  // 2.5V

    cout << "    设置 AO[" << ao_port << "] = " << target_value << " V..." << endl;
    robot.setAO(0, ao_port, target_value, ec);
    if (!ec) {
        cout << "      设置完成" << endl;
    }

    // 8. 多个DO批量操作演示
    cout << "\n[8] 多个DO操作演示..." << endl;
    cout << "    依次设置 DO[0]~DO[3] 为 ON:" << endl;
    for (unsigned int port = 0; port < 4; ++port) {
        robot.setDO(0, port, true, ec);
        if (!ec) {
            cout << "      DO[" << port << "] = ON" << endl;
        }
    }

    // 读取全部状态
    cout << "    当前DO状态:" << endl;
    for (unsigned int port = 0; port < 8; ++port) {
        bool state = robot.getDO(0, port, ec);
        if (!ec) {
            cout << "      DO[" << port << "] = " << (state ? "ON" : "OFF") << endl;
        }
    }

    // 关闭所有DO
    cout << "    关闭所有DO..." << endl;
    for (unsigned int port = 0; port < 8; ++port) {
        robot.setDO(0, port, false, ec);
    }
    cout << "      完成" << endl;

    // 9. 关于仿真模式的说明
    cout << "\n[9] 仿真模式说明..." << endl;
    cout << "    在仿真环境中:" << endl;
    cout << "      - DI读取返回默认值或上次设置的值" << endl;
    cout << "      - DO设置会被记录" << endl;
    cout << "      - AI/AO类似处理" << endl;
    cout << "      - 如需物理IO, 请连接真实机器人" << endl;

    // 10. 清理
    cout << "\n[10] 清理..." << endl;
    robot.setSimulationMode(false, ec);
    robot.disconnectFromRobot(ec);

    cout << "\n==========================================" << endl;
    cout << "  示例 6 完成!" << endl;
    cout << "==========================================" << endl;

    return 0;
}
