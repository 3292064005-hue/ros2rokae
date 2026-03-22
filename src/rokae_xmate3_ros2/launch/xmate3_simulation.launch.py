import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _resolve_package_share():
    env_share = os.environ.get("ROKAE_XMATE3_ROS2_SHARE_DIR", "")
    if env_share and os.path.isdir(env_share):
        return env_share
    try:
        return get_package_share_directory("rokae_xmate3_ros2")
    except PackageNotFoundError:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    pkg_share = _resolve_package_share()
    simulation_launch = os.path.join(pkg_share, "launch", "simulation.launch.py")
    default_model = os.path.join(pkg_share, "urdf", "xMate3.xacro")
    default_world = os.path.join(pkg_share, "worlds", "empty.world")

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch),
            launch_arguments={
                "model": LaunchConfiguration("model"),
                "world": LaunchConfiguration("world"),
                "gui": LaunchConfiguration("gui"),
                "rviz": LaunchConfiguration("rviz"),
                "verbose": LaunchConfiguration("verbose"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "enable_ros2_control": LaunchConfiguration("enable_ros2_control"),
                "enable_xcore_plugin": LaunchConfiguration("enable_xcore_plugin"),
                "backend_mode": LaunchConfiguration("backend_mode"),
            }.items(),
        ),
    ])
