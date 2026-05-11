import os
import sys

import launch
import launch_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from _simulation_support import (
    build_controller_spawners,
    build_gazebo_description_publisher_action,
    build_environment_actions,
    build_gazebo_launch,
    build_robot_description,
    build_robot_state_publisher,
    build_rviz_node,
    build_spawn_entity_action,
    build_spawn_exit_handler,
    build_jtc_readiness_probe,
    build_runtime_host_group,
    declare_arguments,
    resolve_package_lib_dir,
    resolve_package_share,
    ros2_control_enabled_expression,
)


def generate_launch_description():
    """
    xMateER3 Gazebo/JTC public 仿真启动文件。

    功能:
    - 默认使用 public_xmate_er3_jtc profile 启动 Gazebo 仿真环境
    - 加载 xMateER3 机器人模型和 xcore_controller_gazebo_plugin
    - 启动 ros2_control / joint_trajectory_controller 闭环
    - 按 service_exposure_profile 裁剪 RT、拖动和路径类实验服务
    """
    pkg_share = resolve_package_share()
    pkg_lib_dir = resolve_package_lib_dir(pkg_share)
    declared_arguments = declare_arguments(pkg_share)
    ros2_control_enabled = ros2_control_enabled_expression()

    robot_description = build_robot_description(
        pkg_share,
        "package://rokae_xmate3_ros2/models/rokae_xmate3_ros2/meshes/",
    )
    robot_state_publisher_node = build_robot_state_publisher(robot_description)
    env_actions = build_environment_actions(pkg_share, pkg_lib_dir)
    gazebo_launch = build_gazebo_launch("world")
    gazebo_description_publisher = build_gazebo_description_publisher_action(pkg_share)
    spawn_entity_node = build_spawn_entity_action("/robot_description_gazebo")
    joint_state_broadcaster_spawner, joint_trajectory_controller_spawner = build_controller_spawners(
        ros2_control_enabled
    )
    jtc_readiness_probe = build_jtc_readiness_probe(pkg_share, ros2_control_enabled)
    on_spawn_exit = build_spawn_exit_handler(
        spawn_entity_node,
        ros2_control_enabled,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        jtc_readiness_probe,
    )
    rviz_node = build_rviz_node(pkg_share)

    return launch.LaunchDescription(
        declared_arguments
        + build_runtime_host_group(
            pkg_share,
            robot_state_publisher_node,
            env_actions,
            gazebo_launch,
            gazebo_description_publisher,
            spawn_entity_node,
            on_spawn_exit,
            rviz_node,
        )
    )
