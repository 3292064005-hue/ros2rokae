import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from _simulation_support import resolve_canonical_artifact, resolve_canonical_model


def generate_launch_description():
    pkg_share = get_package_share_directory("rokae_xmate3_ros2")
    renderer = os.path.join(pkg_share, "tools", "render_robot_description.py")
    urdf_file = resolve_canonical_model(pkg_share)
    canonical_artifact = resolve_canonical_artifact(pkg_share)
    rviz_config = os.path.join(pkg_share, "config", "xMate3.rviz")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=urdf_file,
        description="URDF/xacro 模型文件路径（默认 canonical URDF；非 canonical 输入需显式打开 allow_noncanonical_model）",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="是否启动 RViz",
    )
    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="RViz only 模式默认不使用仿真时间",
    )
    allow_noncanonical_arg = DeclareLaunchArgument(
        "allow_noncanonical_model",
        default_value="false",
        description="是否允许显式使用非 canonical 模型输入（开发者兼容旁路）",
    )

    robot_description_content = Command([
        sys.executable,
        " ",
        renderer,
        " --model ",
        LaunchConfiguration("model"),
        " --package-share ",
        pkg_share,
        " --mesh-root model://rokae_xmate3_ros2/meshes/",
        " --enable-ros2-control true",
        " --enable-xcore-plugin true",
        " --backend-mode hybrid",
        " --canonical-model ",
        canonical_artifact,
        " --allow-noncanonical-model ",
        LaunchConfiguration("allow_noncanonical_model"),
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[("/joint_states", "/xmate3/joint_states")],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"robot_description": robot_description},
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        sim_time_arg,
        allow_noncanonical_arg,
        rsp,
        rviz,
    ])
