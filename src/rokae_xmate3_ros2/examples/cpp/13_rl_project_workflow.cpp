/**
 * @file 13_rl_project_workflow.cpp
 * @brief 外部 rl_project 的 xMate3 非交互整理版
 */

#include <thread>
#include <vector>

#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
using namespace example;

int main() {
  printHeader("示例 13: RL 工程工作流", "rl_project 对齐版");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }

  printSection("1 切换到 RL 任务模式");
  robot.setMotionControlMode(MotionControlMode::NrtRLTask, ec);
  if (reportError("setMotionControlMode(NrtRLTask)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.setOperateMode(OperateMode::automatic, ec);
  if (reportError("setOperateMode(automatic)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.setPowerState(true, ec);
  if (reportError("setPowerState(on)", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("2 查询项目、工具和工件");
  const auto projects = robot.projectsInfo(ec);
  if (reportError("projectsInfo", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "projects count: " << projects.size() << std::endl;
  for (const auto &project : projects) {
    os << "project: " << project.name << " tasks=";
    for (const auto &task : project.taskList) {
      os << task << ' ';
    }
    os << std::endl;
  }

  const auto tools = robot.toolsInfo(ec);
  if (reportError("toolsInfo", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "tools count: " << tools.size() << std::endl;
  for (const auto &tool : tools) {
    os << "tool: " << tool.name << " mass=" << tool.load.mass << std::endl;
  }

  const auto wobjs = robot.wobjsInfo(ec);
  if (reportError("wobjsInfo", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "wobjs count: " << wobjs.size() << std::endl;
  for (const auto &wobj : wobjs) {
    os << "wobj: " << wobj.name << " held=" << std::boolalpha << wobj.robotHeld << std::noboolalpha << std::endl;
  }

  printSection("3 设置运行参数");
  robot.setProjectRunningOpt(0.5, false, ec);
  if (reportError("setProjectRunningOpt", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "project running option updated" << std::endl;

  printSection("4 可选加载和运行首个工程");
  if (!projects.empty()) {
    const auto &project = projects.front();
    robot.loadProject(project.name, project.taskList, ec);
    if (reportError("loadProject", ec)) {
      cleanupRobot(robot);
      return 1;
    }
    robot.ppToMain(ec);
    if (reportError("ppToMain", ec)) {
      cleanupRobot(robot);
      return 1;
    }
    if (!project.taskList.empty()) {
      robot.runProject(ec);
      if (reportError("runProject", ec)) {
        cleanupRobot(robot);
        return 1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      robot.pauseProject(ec);
      if (reportError("pauseProject", ec)) {
        cleanupRobot(robot);
        return 1;
      }
      os << "project started then paused" << std::endl;
    } else {
      os << "project has no tasks, skip runProject" << std::endl;
    }
  } else {
    os << "no RL projects available in Gazebo shim" << std::endl;
  }

  printSection("5 恢复非实时命令模式并断开连接");
  robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
  robot.setOperateMode(OperateMode::automatic, ec);
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
