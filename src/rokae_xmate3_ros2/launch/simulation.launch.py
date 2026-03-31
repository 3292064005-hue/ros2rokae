import os
import sys

import launch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from _simulation_support import (
    build_controller_spawners,
    build_environment_actions,
    build_gazebo_launch,
    build_robot_description,
    build_robot_state_publisher,
    build_rviz_node,
    build_spawn_entity_action,
    build_spawn_exit_handler,
    declare_arguments,
    resolve_package_lib_dir,
    resolve_package_share,
    ros2_control_enabled_expression,
)


def generate_launch_description():
    """
    xMate3 纯 Gazebo 仿真启动文件。

    功能:
    - 启动 Gazebo 仿真环境
    - 加载 xMate3 机器人模型 (使用 xcore_controller_gazebo_plugin)
    - 启动 RViz 可视化
    """
    pkg_share = resolve_package_share()
    pkg_lib_dir = resolve_package_lib_dir(pkg_share)
    declared_arguments = declare_arguments(pkg_share)
    ros2_control_enabled = ros2_control_enabled_expression()

    robot_description = build_robot_description(pkg_share)
    robot_state_publisher_node = build_robot_state_publisher(robot_description)
    env_actions = build_environment_actions(pkg_share, pkg_lib_dir)
    gazebo_launch = build_gazebo_launch("world")
    spawn_entity_node = build_spawn_entity_action()
    joint_state_broadcaster_spawner, joint_trajectory_controller_spawner = build_controller_spawners(
        ros2_control_enabled
    )
    on_spawn_exit = build_spawn_exit_handler(
        spawn_entity_node,
        ros2_control_enabled,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    )
    rviz_node = build_rviz_node(pkg_share)

    return launch.LaunchDescription(
        declared_arguments
        + [
            launch.actions.LogInfo(msg="正在启动 xMate3 纯 Gazebo 仿真环境..."),
            *env_actions,
            robot_state_publisher_node,
            gazebo_launch,
            spawn_entity_node,
            on_spawn_exit,
            rviz_node,
        ]
    )
