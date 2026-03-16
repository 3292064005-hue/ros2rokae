#include "rokae_xmate3_ros2/robot.hpp"
#include <iostream>
#include <iomanip>

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    rokae::ros2::xMateRobot robot;
    std::error_code ec;

    robot.connectToRobot(ec);
    if (ec) {
        std::cerr << "connectToRobot failed: " << ec.message() << std::endl;
        return 1;
    }

    robot.setPowerState(true, ec);
    robot.setOperateMode(rokae::OperateMode::automatic, ec);
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);

    // 1) Jog / 奇异规避 / 末端力矩
    robot.setAvoidSingularity(true, ec);
    std::cout << "Avoid singularity: " << (robot.getAvoidSingularity(ec) ? "ON" : "OFF") << std::endl;
    robot.startJog(rokae::JogOpt::Space::jointSpace, 0, true, 0.05, ec);
    auto wrench = robot.getEndTorque(ec);
    std::cout << "Estimated end wrench: ";
    for (double v : wrench) std::cout << std::fixed << std::setprecision(4) << v << ' ';
    std::cout << std::endl;

    // 2) 实时控制/状态
    robot.setRtControlMode(rokae::RtControllerMode::jointPosition, ec);
    std::array<double, 6> q{}, dq{}, tau{};
    if (robot.getRtJointData(q, dq, tau, ec)) {
        std::cout << "RT joint position: ";
        for (double v : q) std::cout << std::fixed << std::setprecision(4) << v << ' ';
        std::cout << std::endl;
    }

    // 3) 寄存器 / 自定义数据
    robot.writeRegister("demo.counter", "42", ec);
    std::cout << "Register demo.counter = " << robot.readRegister("demo.counter", ec) << std::endl;
    std::cout << "Custom ACK = " << robot.sendCustomData("demo/topic", "{\"hello\":\"world\"}", ec) << std::endl;
    std::cout << "Callback register = " << (robot.registerDataCallback("joint_state", "demo_cb", ec) ? "OK" : "FAIL") << std::endl;

    // 4) 动力学/轨迹能力
    std::array<double, 6> joint_torque{}, gravity_torque{}, coriolis_torque{};
    robot.calcJointTorque(q, dq, {0,0,0,0,0,0}, {0,0,0,0,0,0}, joint_torque, gravity_torque, coriolis_torque, ec);

    std::vector<std::array<double, 6>> trajectory_points;
    double total_time = 0.0;
    std::array<double, 6> target = q;
    target[0] += 0.1;
    if (robot.generateSTrajectory(q, target, trajectory_points, total_time, ec)) {
        std::cout << "Generated S-trajectory points: " << trajectory_points.size()
                  << ", total_time = " << total_time << " s" << std::endl;
    }

    std::array<double, 6> mapped_torque{};
    robot.mapCartesianToJointTorque({0, 0, 10, 0, 0, 0}, q, mapped_torque, ec);

    // 5) RL 工程模拟接口
    std::string project_name;
    if (robot.loadRLProject("/tmp/demo_project.rl", project_name, ec)) {
        int episode = 0;
        robot.startRLProject(project_name, episode, ec);
        std::cout << "RL project started: " << project_name << ", episode = " << episode << std::endl;
        int finished = 0;
        robot.stopRLProject(project_name, finished, ec);
        std::cout << "RL project stopped, finished_episode = " << finished << std::endl;
    }

    robot.disconnectFromRobot(ec);
    return 0;
}
