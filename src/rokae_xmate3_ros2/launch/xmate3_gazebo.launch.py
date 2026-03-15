#!/usr/bin/env python3
"""
@file xmate3_gazebo.launch.py
@brief 兼容旧版launch文件名 - 重定向到 simulation.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """兼容旧版文件名"""
    pkg_share = get_package_share_directory("rokae_xmate3_ros2")

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "simulation.launch.py")
        )
    )
