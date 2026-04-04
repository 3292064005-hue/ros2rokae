import os
import sys

import launch
import launch_ros
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
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


def declare_arguments(pkg_share):
    urdf_file = resolve_canonical_model(pkg_share)
    world_file = os.path.join(pkg_share, "worlds", "empty.world")
    return [
        launch.actions.DeclareLaunchArgument("model", default_value=str(urdf_file), description="URDF/xacro 模型文件路径"),
        launch.actions.DeclareLaunchArgument("world", default_value=str(world_file), description="Gazebo world 文件路径"),
        launch.actions.DeclareLaunchArgument("gui", default_value="true", description="是否启动 Gazebo GUI"),
        launch.actions.DeclareLaunchArgument("rviz", default_value="true", description="是否启动 RViz"),
        launch.actions.DeclareLaunchArgument("verbose", default_value="true", description="Gazebo 详细输出"),
        launch.actions.DeclareLaunchArgument("use_sim_time", default_value="true", description="使用仿真时间"),
        launch.actions.DeclareLaunchArgument("enable_ros2_control", default_value="true", description="是否启用 ros2_control / joint_trajectory_controller"),
        launch.actions.DeclareLaunchArgument("enable_xcore_plugin", default_value="true", description="是否加载 xCore Gazebo plugin"),
        launch.actions.DeclareLaunchArgument("backend_mode", default_value="hybrid", description="后端装配模式: effort | jtc | hybrid"),
        launch.actions.DeclareLaunchArgument("allow_noncanonical_model", default_value="false", description="是否允许显式使用非 canonical 模型输入（开发者兼容旁路）"),
    ]


def ros2_control_enabled_expression():
    return PythonExpression([
        "'", LaunchConfiguration("enable_ros2_control"), "' == 'true' and '", LaunchConfiguration("backend_mode"), "' != 'effort'"
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
        LaunchConfiguration("enable_ros2_control"),
        " --enable-xcore-plugin ",
        LaunchConfiguration("enable_xcore_plugin"),
        " --backend-mode ",
        LaunchConfiguration("backend_mode"),
        " --canonical-model ",
        resolve_canonical_artifact(pkg_share),
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
                log_info(["当前 backend_mode=", LaunchConfiguration("backend_mode"), " enable_xcore_plugin=", LaunchConfiguration("enable_xcore_plugin"), " enable_ros2_control=", LaunchConfiguration("enable_ros2_control")]),
                log_info(["模型来源=", LaunchConfiguration("model")]),
                log_info(["allow_noncanonical_model=", LaunchConfiguration("allow_noncanonical_model")]),
                log_info(["RT 能力级别: experimental  diagnostics backend=", LaunchConfiguration("backend_mode")]),
                log_info("可用服务和话题:"),
                log_info("  - /xmate3/cobot/*  (SDK服务)"),
                log_info("  - /xmate3/joint_states  (关节状态)"),
                log_info("  - /xmate3/internal/validate_motion  (运动预验证)"),
                log_info("  - /xmate3/internal/get_runtime_diagnostics  (运行时诊断服务)"),
                log_info("  - /xmate3/internal/runtime_status  (只读运行时诊断话题)"),
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
