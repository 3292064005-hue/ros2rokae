import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from _simulation_support import resolve_canonical_model, resolve_package_share


def generate_launch_description():
    pkg_share = resolve_package_share()
    simulation_launch = os.path.join(pkg_share, "launch", "simulation.launch.py")
    default_model = resolve_canonical_model(pkg_share)
    default_world = os.path.join(pkg_share, "worlds", "empty.world")

    forwarded_arguments = {
        "model": LaunchConfiguration("model"),
        "world": LaunchConfiguration("world"),
        "gui": LaunchConfiguration("gui"),
        "rviz": LaunchConfiguration("rviz"),
        "verbose": LaunchConfiguration("verbose"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "enable_ros2_control": LaunchConfiguration("enable_ros2_control"),
        "enable_xcore_plugin": LaunchConfiguration("enable_xcore_plugin"),
        "backend_mode": LaunchConfiguration("backend_mode"),
        "allow_noncanonical_model": LaunchConfiguration("allow_noncanonical_model"),
    }

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=default_model),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("verbose", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("enable_ros2_control", default_value="true"),
        DeclareLaunchArgument("enable_xcore_plugin", default_value="true"),
        DeclareLaunchArgument("backend_mode", default_value="hybrid"),
        DeclareLaunchArgument("allow_noncanonical_model", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch),
            launch_arguments=forwarded_arguments.items(),
        ),
    ])
