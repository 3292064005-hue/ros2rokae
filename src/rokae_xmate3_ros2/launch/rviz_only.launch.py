#!/usr/bin/env python3
"""
@file rviz_only.launch.py
@brief 仅启动RViz2用于可视化
@details 用于查看机器人模型而不启动Gazebo
"""

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command


def generate_launch_description():
    pkg_share = get_package_share_directory("rokae_xmate3_ros2")
    urdf_path = os.path.join(pkg_share, "urdf", "xMate3.xacro")
    rviz_config = os.path.join(pkg_share, "config", "xMate3.rviz")

    arg_model = DeclareLaunchArgument(
        name="model",
        default_value=str(urdf_path),
        description="URDF/xacro模型文件路径"
    )

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    node_robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}]
    )

    node_joint_state_pub_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )

    return launch.LaunchDescription([
        arg_model,
        launch.actions.LogInfo(msg="启动 xMate3 RViz 可视化..."),
        node_robot_state_pub,
        node_joint_state_pub_gui,
        node_rviz,
    ])
