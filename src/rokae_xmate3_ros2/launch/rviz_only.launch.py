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

from _simulation_support import (
    resolve_canonical_artifact, resolve_canonical_metadata, resolve_canonical_model,
    resolved_backend_mode_expression, resolved_enable_ros2_control_expression,
    resolved_enable_xcore_plugin_expression, resolved_service_profile_expression,
    resolved_compatibility_alias_policy_expression
)


def generate_launch_description():
    pkg_share = get_package_share_directory("rokae_xmate3_ros2")
    renderer = os.path.join(pkg_share, "tools", "render_robot_description.py")
    urdf_file = resolve_canonical_model(pkg_share)
    canonical_artifact = resolve_canonical_artifact(pkg_share) or urdf_file
    canonical_metadata = resolve_canonical_metadata(pkg_share)
    rviz_config = os.path.join(pkg_share, "config", "xMateER3.rviz")

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
    launch_profile_arg = DeclareLaunchArgument(
        "launch_profile",
        default_value="public_xmate_er3_jtc",
        description="能力矩阵 profile；空缺的后端/暴露参数将跟随该 profile",
    )
    backend_mode_arg = DeclareLaunchArgument(
        "backend_mode",
        default_value="",
        description="robot_description 生成使用的后端模式",
    )
    service_profile_arg = DeclareLaunchArgument(
        "service_exposure_profile",
        default_value="",
        description="robot_description 生成使用的服务暴露 profile",
    )
    compatibility_alias_policy_arg = DeclareLaunchArgument(
        "compatibility_alias_policy",
        default_value="",
        description="robot_description / runtime 兼容别名发布策略；留空时跟随 launch_profile",
    )
    enable_xcore_plugin_arg = DeclareLaunchArgument(
        "enable_xcore_plugin",
        default_value="",
        description="robot_description 生成时是否启用 xCore Gazebo plugin；留空时跟随 launch_profile",
    )
    enable_ros2_control_arg = DeclareLaunchArgument(
        "enable_ros2_control",
        default_value="",
        description="robot_description 生成时是否启用 ros2_control；留空时跟随 launch_profile",
    )

    robot_description_content = Command([
        sys.executable,
        " ",
        renderer,
        " --model ",
        LaunchConfiguration("model"),
        " --package-share ",
        pkg_share,
        " --mesh-root package://rokae_xmate3_ros2/models/rokae_xmate3_ros2/meshes/",
        " --enable-ros2-control ",
        resolved_enable_ros2_control_expression(),
        " --enable-xcore-plugin ",
        resolved_enable_xcore_plugin_expression(),
        " --backend-mode ",
        resolved_backend_mode_expression(),
        " --service-exposure-profile ",
        resolved_service_profile_expression(),
        " --compatibility-alias-policy ",
        resolved_compatibility_alias_policy_expression(),
        " --canonical-model ",
        canonical_artifact,
        " --canonical-metadata ",
        canonical_metadata,
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
        remappings=[("/joint_states", "/xmate_er3/joint_states")],
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
        launch_profile_arg,
        backend_mode_arg,
        service_profile_arg,
        compatibility_alias_policy_arg,
        enable_xcore_plugin_arg,
        enable_ros2_control_arg,
        rsp,
        rviz,
    ])
