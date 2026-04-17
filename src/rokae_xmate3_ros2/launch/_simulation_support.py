import os
import sys

import launch
import launch_ros

from _launch_profile import default_launch_profile_name, profile_names, resolve_launch_profile
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def resolve_package_share():
    env_share = os.environ.get("ROKAE_XMATE3_ROS2_SHARE_DIR", "")
    if env_share and os.path.isdir(env_share):
        return env_share
    try:
        return get_package_share_directory("rokae_xmate3_ros2")
    except PackageNotFoundError:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def resolve_package_lib_dir(pkg_share):
    env_lib = os.environ.get("ROKAE_XMATE3_ROS2_LIB_DIR", "")
    if env_lib and os.path.isdir(env_lib):
        return env_lib
    pkg_prefix = os.path.dirname(os.path.dirname(pkg_share))
    return os.path.join(pkg_prefix, "lib")


def resolve_canonical_artifact(pkg_share):
    canonical = os.path.join(pkg_share, "generated", "urdf", "xMate3.urdf")
    if os.path.isfile(canonical):
        return canonical
    return ""


def resolve_canonical_model(pkg_share):
    canonical = resolve_canonical_artifact(pkg_share)
    if canonical:
        return canonical
    return os.path.join(pkg_share, "urdf", "xMate3.xacro")


def resolve_canonical_metadata(pkg_share):
    canonical = os.path.join(pkg_share, "generated", "urdf", "xMate3.description.json")
    if os.path.isfile(canonical):
        return canonical
    return ""


def declare_arguments(pkg_share):
    urdf_file = resolve_canonical_model(pkg_share)
    world_file = os.path.join(pkg_share, "worlds", "empty.world")
    profiles = " | ".join(profile_names())
    return [
        launch.actions.DeclareLaunchArgument("model", default_value=str(urdf_file), description="URDF/xacro 模型文件路径"),
        launch.actions.DeclareLaunchArgument("world", default_value=str(world_file), description="Gazebo world 文件路径"),
        launch.actions.DeclareLaunchArgument("gui", default_value="true", description="是否启动 Gazebo GUI"),
        launch.actions.DeclareLaunchArgument("rviz", default_value="true", description="是否启动 RViz"),
        launch.actions.DeclareLaunchArgument("verbose", default_value="true", description="Gazebo 详细输出"),
        launch.actions.DeclareLaunchArgument("use_sim_time", default_value="true", description="使用仿真时间"),
        launch.actions.DeclareLaunchArgument("launch_profile", default_value=default_launch_profile_name(), description=f"能力矩阵 profile: {profiles}"),
        launch.actions.DeclareLaunchArgument("runtime_host", default_value="", description="运行宿主: gazebo_plugin | daemonized_runtime；留空时跟随 launch_profile"),
        launch.actions.DeclareLaunchArgument("runtime_profile", default_value="", description="运行时 profile；留空时跟随 launch_profile"),
        launch.actions.DeclareLaunchArgument("enable_ros2_control", default_value="", description="是否启用 ros2_control / joint_trajectory_controller；留空时跟随 launch_profile"),
        launch.actions.DeclareLaunchArgument("enable_xcore_plugin", default_value="", description="是否加载 xCore Gazebo plugin；留空时跟随 launch_profile"),
        launch.actions.DeclareLaunchArgument("backend_mode", default_value="", description="后端装配模式: effort | jtc | hybrid；留空时跟随 launch_profile"),
        launch.actions.DeclareLaunchArgument("service_exposure_profile", default_value="", description="服务暴露面: public_xmate6_only | internal_full；留空时跟随 launch_profile"),
        launch.actions.DeclareLaunchArgument("allow_noncanonical_model", default_value="false", description="是否允许显式使用非 canonical 模型输入（开发者兼容旁路）"),
    ]




def validate_launch_profile_action():
    def _validate(context, *_args, **_kwargs):
        selected = LaunchConfiguration("launch_profile").perform(context)
        if not selected:
            return []
        if selected not in profile_names():
            allowed = ", ".join(profile_names())
            raise RuntimeError(f"unknown launch_profile '{selected}'; allowed: {allowed}")
        return []

    return OpaqueFunction(function=_validate)


def build_runtime_host_group(pkg_share, robot_state_publisher_node, env_actions, gazebo_launch, spawn_entity_node, on_spawn_exit, rviz_node):
    daemon_runtime_node = Node(
        package="rokae_xmate3_ros2",
        executable="rokae_sim_runtime",
        output="screen",
        parameters=[{
            "service_exposure_profile": resolved_service_profile_expression(),
            "runtime_profile": resolved_runtime_profile_expression(),
        }],
        condition=launch.conditions.IfCondition(
            launch.substitutions.PythonExpression(["'", resolved_runtime_host_expression(), "' == 'daemonized_runtime'"]))
    )
    gazebo_group = launch.actions.GroupAction(
        actions=[gazebo_launch, spawn_entity_node, on_spawn_exit],
        condition=launch.conditions.IfCondition(
            launch.substitutions.PythonExpression(["'", resolved_runtime_host_expression(), "' == 'gazebo_plugin'"]))
    )
    return [
        validate_launch_profile_action(),
        launch.actions.LogInfo(msg=["启动 canonical xMate6 launch_profile=", launch.substitutions.LaunchConfiguration("launch_profile")]),
        *env_actions,
        robot_state_publisher_node,
        gazebo_group,
        daemon_runtime_node,
        rviz_node,
    ]

def profile_field_substitution(field_name):
    profile_map = {
        name: getattr(resolve_launch_profile(name), field_name)
        for name in profile_names()
    }
    default_value = getattr(resolve_launch_profile(default_launch_profile_name()), field_name)
    mapping_literals = ', '.join([f"\"{name}\":\"{value}\"" for name, value in profile_map.items()])
    return PythonExpression([
        "({", mapping_literals, "}.get('", LaunchConfiguration('launch_profile'), "', '", default_value, "')) if '",
        LaunchConfiguration(field_name), "' == '' else '", LaunchConfiguration(field_name), "'"
    ])


def resolved_runtime_host_expression():
    return profile_field_substitution('runtime_host')


def resolved_runtime_profile_expression():
    return profile_field_substitution('runtime_profile')


def resolved_backend_mode_expression():
    return profile_field_substitution('backend_mode')


def resolved_service_profile_expression():
    return profile_field_substitution('service_exposure_profile')


def resolved_enable_ros2_control_expression():
    return profile_field_substitution('enable_ros2_control')


def resolved_enable_xcore_plugin_expression():
    return profile_field_substitution('enable_xcore_plugin')


def ros2_control_enabled_expression():
    return PythonExpression([
        "'", resolved_enable_ros2_control_expression(), "' == 'true' and '", resolved_backend_mode_expression(), "' != 'effort'"
    ])


def build_robot_description(pkg_share):
    renderer = os.path.join(pkg_share, "tools", "render_robot_description.py")
    content = launch.substitutions.Command([
        sys.executable,
        " ",
        renderer,
        " --model ",
        LaunchConfiguration("model"),
        " --package-share ",
        pkg_share,
        " --mesh-root model://rokae_xmate3_ros2/meshes/",
        " --enable-ros2-control ",
        resolved_enable_ros2_control_expression(),
        " --enable-xcore-plugin ",
        resolved_enable_xcore_plugin_expression(),
        " --backend-mode ",
        resolved_backend_mode_expression(),
        " --service-exposure-profile ",
        resolved_service_profile_expression(),
        " --canonical-model ",
        resolve_canonical_model(pkg_share),
        " --canonical-metadata ",
        resolve_canonical_metadata(pkg_share),
        " --allow-noncanonical-model ",
        LaunchConfiguration("allow_noncanonical_model"),
    ])
    return launch_ros.parameter_descriptions.ParameterValue(content, value_type=str)


def build_robot_state_publisher(robot_description):
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[("/joint_states", "/xmate3/joint_states")],
    )


def _gazebo_install_prefix():
    env_prefix = os.environ.get("ROKAE_GAZEBO_PREFIX", "")
    if env_prefix and os.path.isdir(env_prefix):
        return env_prefix
    return ""


def build_environment_actions(pkg_share, pkg_lib_dir):
    existing_gazebo_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    existing_gazebo_resource_path = os.environ.get("GAZEBO_RESOURCE_PATH", "")
    existing_gazebo_plugin_path = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    gazebo_worlds_dir = os.path.join(pkg_share, "worlds")
    gazebo_models_dir = os.path.join(pkg_share, "models")
    canonical_urdf = os.path.join(pkg_share, "generated", "urdf", "xMate3.urdf")
    canonical_metadata = os.path.join(pkg_share, "generated", "urdf", "xMate3.description.json")

    gazebo_model_path_entries = []
    if existing_gazebo_model_path:
        gazebo_model_path_entries.append(existing_gazebo_model_path)
    if os.path.isdir(gazebo_models_dir):
        gazebo_model_path_entries.append(gazebo_models_dir)
    if os.path.isdir(gazebo_worlds_dir):
        gazebo_model_path_entries.append(gazebo_worlds_dir)

    gazebo_resource_root = _gazebo_install_prefix()
    gazebo_resource_path_entries = []
    if existing_gazebo_resource_path:
        gazebo_resource_path_entries.append(existing_gazebo_resource_path)
    if gazebo_resource_root:
        gazebo_resource_path_entries.append(gazebo_resource_root)
    gazebo_resource_path_entries.append(pkg_share)

    gazebo_plugin_path_entries = []
    if existing_gazebo_plugin_path:
        gazebo_plugin_path_entries.append(existing_gazebo_plugin_path)
    if os.path.isdir(pkg_lib_dir):
        gazebo_plugin_path_entries.append(pkg_lib_dir)

    actions = [
        launch.actions.SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.pathsep.join(gazebo_model_path_entries)),
        launch.actions.SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", os.pathsep.join(gazebo_resource_path_entries)),
        launch.actions.SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", os.pathsep.join(gazebo_plugin_path_entries)),
        launch.actions.SetEnvironmentVariable("ROKAE_SERVICE_EXPOSURE_PROFILE", resolved_service_profile_expression()),
    ]
    if os.path.isfile(canonical_urdf):
        actions.append(launch.actions.SetEnvironmentVariable("ROKAE_XMATE3_CANONICAL_URDF", canonical_urdf))
    if os.path.isfile(canonical_metadata):
        actions.append(launch.actions.SetEnvironmentVariable("ROKAE_XMATE3_CANONICAL_URDF_METADATA", canonical_metadata))
    return actions


def build_gazebo_launch(world):
    try:
        gazebo_share = get_package_share_directory("gazebo_ros")
    except PackageNotFoundError as exc:
        raise RuntimeError("gazebo_ros package is required for simulation launch") from exc
    gazebo_launch_dir = os.path.join(gazebo_share, "launch")
    return launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch_dir, "gazebo.launch.py")),
        launch_arguments={
            "world": LaunchConfiguration(world),
            "verbose": LaunchConfiguration("verbose"),
            "gui": LaunchConfiguration("gui"),
            "pause": "false",
            "server_required": "true",
        }.items(),
    )


def build_spawn_entity_action():
    try:
        gazebo_ros_share = get_package_share_directory("gazebo_ros")
    except PackageNotFoundError as exc:
        raise RuntimeError("gazebo_ros package is required to locate spawn_entity.py") from exc
    gazebo_ros_prefix = os.path.dirname(os.path.dirname(gazebo_ros_share))
    spawn_entity_script = os.path.join(gazebo_ros_prefix, "lib", "gazebo_ros", "spawn_entity.py")
    if not os.path.isfile(spawn_entity_script):
        raise RuntimeError(f"spawn_entity.py not found at expected path: {spawn_entity_script}")
    python_bin = os.environ.get("ROKAE_PYTHON_EXECUTABLE", sys.executable)
    if not python_bin or not os.path.exists(python_bin):
        raise RuntimeError("A valid Python interpreter is required for spawn_entity.py")
    return launch.actions.ExecuteProcess(
        cmd=[
            python_bin,
            spawn_entity_script,
            "-topic", "/robot_description",
            "-entity", "xmate",
        ],
        output="screen",
        additional_env={"PATH": os.environ.get("PATH", ""), "PYTHONHOME": ""},
    )


def build_controller_spawners(enabled_condition):
    common_parameters = [{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "120"],
        output="screen",
        parameters=common_parameters,
        condition=launch.conditions.IfCondition(enabled_condition),
    )
    trajectory = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "120"],
        output="screen",
        parameters=common_parameters,
        condition=launch.conditions.IfCondition(enabled_condition),
    )
    return broadcaster, trajectory


def build_spawn_exit_handler(spawn_entity_node, ros2_control_enabled, joint_state_broadcaster_spawner, joint_trajectory_controller_spawner):
    def log_info(msg):
        return launch.actions.LogInfo(msg=msg)

    return launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                launch.actions.TimerAction(
                    period=3.0,
                    actions=[
                        log_info("正在启动 ros2_control controllers: joint_state_broadcaster, joint_trajectory_controller"),
                        joint_state_broadcaster_spawner,
                        joint_trajectory_controller_spawner,
                    ],
                    condition=launch.conditions.IfCondition(ros2_control_enabled),
                ),
                log_info("=" * 60),
                log_info("spawn_entity.py 已退出，请检查上方输出确认机器人是否成功生成。"),
                log_info("若看到 'Successfully spawned entity [xmate]'，则说明机器人已正确加载。"),
                log_info("使用 xcore_controller_gazebo_plugin 提供 xCore SDK 仿真。"),
                log_info(["当前 launch_profile=", LaunchConfiguration("launch_profile")]),
                log_info(["当前 runtime_host=", resolved_runtime_host_expression(), " runtime_profile=", resolved_runtime_profile_expression()]),
                log_info(["当前 backend_mode=", resolved_backend_mode_expression(), " enable_xcore_plugin=", resolved_enable_xcore_plugin_expression(), " enable_ros2_control=", resolved_enable_ros2_control_expression()]),
                log_info(["当前 service_exposure_profile=", resolved_service_profile_expression()]),
                log_info(["模型来源=", LaunchConfiguration("model")]),
                log_info(["allow_noncanonical_model=", LaunchConfiguration("allow_noncanonical_model")]),
                log_info(["RT 能力级别: experimental  diagnostics backend=", resolved_backend_mode_expression(), " runtime_profile=", resolved_runtime_profile_expression()]),
                log_info("可用服务和话题:"),
                log_info("  - /xmate3/cobot/*  (SDK服务)"),
                log_info("  - /xmate3/joint_states  (关节状态)"),
                log_info("  - /xmate3/internal/get_runtime_diagnostics  (运行时诊断服务)"),
                log_info("  - /xmate3/internal/runtime_status  (只读运行时诊断话题)"),
                log_info("  - /xmate3/internal/validate_motion  (仅 service_exposure_profile=internal_full 时暴露)"),
                log_info("兼容别名: /xmate3/cobot/get_joint_torque, /xmate3/cobot/get_end_torque"),
                log_info("运行示例程序:"),
                log_info("  ros2 run rokae_xmate3_ros2 example_04_motion_basic"),
                log_info("=" * 60),
            ],
        )
    )


def build_rviz_node(pkg_share):
    rviz_config = os.path.join(pkg_share, "config", "xMate3.rviz")
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=launch.conditions.IfCondition(LaunchConfiguration("rviz")),
    )
