import os
import sys

import launch
import launch_ros

from _launch_profile import (
    backend_mode_requirements,
    capability_matrix,
    default_launch_profile_name,
    profile_names,
    resolve_launch_profile,
    service_exposure_profiles,
)
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
    canonical = os.path.join(pkg_share, "generated", "urdf", "xMateER3.urdf")
    if os.path.isfile(canonical):
        return canonical
    return ""


def resolve_canonical_model(pkg_share):
    canonical = resolve_canonical_artifact(pkg_share)
    if canonical:
        return canonical
    return os.path.join(pkg_share, "urdf", "xMateER3.xacro")


def resolve_canonical_metadata(pkg_share):
    return os.path.join(pkg_share, "generated", "urdf", "xMateER3.description.json")


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
        launch.actions.DeclareLaunchArgument("backend_mode", default_value="", description="后端装配模式: jtc | hybrid | effort | headless_sim；留空时跟随 launch_profile"),
        launch.actions.DeclareLaunchArgument("service_exposure_profile", default_value="", description="服务暴露面: public_xmate_er3_only | public_xmate_er3_experimental | internal_full；留空时跟随 launch_profile"),
        launch.actions.DeclareLaunchArgument("compatibility_alias_policy", default_value="", description="兼容别名发布策略: canonical_plus_compat | canonical_only | legacy_only；留空时跟随 launch_profile"),
        launch.actions.DeclareLaunchArgument("allow_noncanonical_model", default_value="false", description="是否允许显式使用非 canonical 模型输入（开发者兼容旁路）"),
        launch.actions.DeclareLaunchArgument("require_runtime_readiness", default_value="true", description="默认 Gazebo/JTC public profile 是否强制等待 controller/action server 就绪"),
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


def _resolved_profile_value(context, field_name: str) -> str:
    explicit_value = LaunchConfiguration(field_name).perform(context)
    if explicit_value:
        return explicit_value
    selected = LaunchConfiguration("launch_profile").perform(context) or default_launch_profile_name()
    return getattr(resolve_launch_profile(selected), field_name)


def _capabilities_for_resolved_profile(runtime_host: str,
                                       backend_mode: str,
                                       enable_ros2_control: str,
                                       enable_xcore_plugin: str) -> set[str]:
    capabilities = {"state_read"}
    if runtime_host == "gazebo_plugin" and enable_xcore_plugin == "true":
        capabilities.add("gazebo_physics")
    if backend_mode in {"jtc", "hybrid"} and enable_ros2_control == "true":
        capabilities.add("trajectory_execution")
    if backend_mode in {"hybrid", "effort", "headless_sim"}:
        capabilities.add("effort_execution")
    if backend_mode in {"hybrid", "effort", "headless_sim"}:
        capabilities.add("rt_direct_command")
    return capabilities


def validate_runtime_capability_action():
    def _validate(context, *_args, **_kwargs):
        launch_profile = LaunchConfiguration("launch_profile").perform(context) or default_launch_profile_name()
        profile = resolve_launch_profile(launch_profile)
        runtime_host = _resolved_profile_value(context, "runtime_host")
        backend_mode = _resolved_profile_value(context, "backend_mode")
        runtime_profile = _resolved_profile_value(context, "runtime_profile")
        service_exposure_profile = _resolved_profile_value(context, "service_exposure_profile")
        enable_ros2_control = _resolved_profile_value(context, "enable_ros2_control")
        enable_xcore_plugin = _resolved_profile_value(context, "enable_xcore_plugin")

        matrix = capability_matrix()
        backend_specs = matrix.get("backend_modes", {}) if isinstance(matrix, dict) else {}
        backend_spec = backend_specs.get(backend_mode, {}) if isinstance(backend_specs, dict) else {}
        if not isinstance(backend_spec, dict) or not backend_spec:
            allowed = ", ".join(sorted(str(key) for key in backend_specs.keys())) if isinstance(backend_specs, dict) else "jtc, hybrid, effort, headless_sim"
            raise RuntimeError(f"backend_mode '{backend_mode}' is not declared in xmate_er3_capability_matrix.json; allowed: {allowed}")

        expected_host = str(backend_spec.get("runtime_host", runtime_host))
        if runtime_host != expected_host:
            raise RuntimeError(
                f"backend_mode={backend_mode} requires runtime_host:={expected_host} per xmate_er3_capability_matrix.json"
            )
        if service_exposure_profile not in service_exposure_profiles():
            allowed = ", ".join(service_exposure_profiles())
            raise RuntimeError(f"unknown service_exposure_profile '{service_exposure_profile}'; allowed: {allowed}")
        if runtime_host == "gazebo_plugin" and enable_xcore_plugin != "true":
            raise RuntimeError("gazebo_plugin runtime_host requires enable_xcore_plugin:=true")
        if backend_mode in {"jtc", "hybrid"} and enable_ros2_control != "true":
            raise RuntimeError(f"{backend_mode} backend requires enable_ros2_control:=true")
        if backend_mode == "headless_sim" and runtime_host != "daemonized_runtime":
            raise RuntimeError("headless_sim backend requires runtime_host:=daemonized_runtime")
        if runtime_profile.startswith("rt") and service_exposure_profile == "public_xmate_er3_only":
            raise RuntimeError("RT runtime_profile requires service_exposure_profile:=public_xmate_er3_experimental or internal_full")

        required = set(profile.required_capabilities) | set(backend_mode_requirements(backend_mode))
        available = _capabilities_for_resolved_profile(runtime_host, backend_mode, enable_ros2_control, enable_xcore_plugin)
        missing = sorted(required - available)
        if missing:
            raise RuntimeError(
                "resolved launch configuration is missing required capability matrix entries: " +
                ", ".join(missing) +
                f"; launch_profile={launch_profile} backend_mode={backend_mode} runtime_host={runtime_host}"
            )
        return []

    return OpaqueFunction(function=_validate)


def build_runtime_host_group(pkg_share,
                             robot_state_publisher_node,
                             env_actions,
                             gazebo_launch,
                             gazebo_description_publisher,
                             spawn_entity_node,
                             on_spawn_exit,
                             rviz_node):
    daemon_runtime_node = Node(
        package="rokae_xmate3_ros2",
        executable="rokae_sim_runtime",
        output="screen",
        parameters=[{
            "service_exposure_profile": resolved_service_profile_expression(),
            "runtime_profile": resolved_runtime_profile_expression(),
            "compatibility_alias_policy": resolved_compatibility_alias_policy_expression(),
        }],
        condition=launch.conditions.IfCondition(
            launch.substitutions.PythonExpression(["'", resolved_runtime_host_expression(), "' == 'daemonized_runtime'"]))
    )
    gazebo_group = launch.actions.GroupAction(
        actions=[gazebo_launch, gazebo_description_publisher, spawn_entity_node, on_spawn_exit],
        condition=launch.conditions.IfCondition(
            launch.substitutions.PythonExpression(["'", resolved_runtime_host_expression(), "' == 'gazebo_plugin'"]))
    )
    return [
        validate_launch_profile_action(),
        validate_runtime_capability_action(),
        launch.actions.LogInfo(msg=["启动 canonical xMateER3 launch_profile=", launch.substitutions.LaunchConfiguration("launch_profile")]),
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


def resolved_compatibility_alias_policy_expression():
    return profile_field_substitution('compatibility_alias_policy')


def ros2_control_enabled_expression():
    return PythonExpression([
        "'", resolved_enable_ros2_control_expression(), "' == 'true' and '", resolved_backend_mode_expression(), "' != 'effort'"
    ])


def build_robot_description(pkg_share, mesh_root="package://rokae_xmate3_ros2/models/rokae_xmate3_ros2/meshes/"):
    renderer = os.path.join(pkg_share, "tools", "render_robot_description.py")
    content = launch.substitutions.Command([
        sys.executable,
        " ",
        renderer,
        " --model ",
        LaunchConfiguration("model"),
        " --package-share ",
        pkg_share,
        " --mesh-root ",
        mesh_root,
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
        remappings=[("/joint_states", "/xmate_er3/joint_states")],
    )


def build_gazebo_description_publisher_action(pkg_share, topic_name="/robot_description_gazebo"):
    publisher_script = os.path.join(pkg_share, "tools", "publish_description_topic.py")
    python_bin = os.environ.get("ROKAE_PYTHON_EXECUTABLE", sys.executable)
    if not python_bin or not os.path.exists(python_bin):
        raise RuntimeError("A valid Python interpreter is required for publish_description_topic.py")
    return launch.actions.ExecuteProcess(
        cmd=[
            python_bin,
            publisher_script,
            "--topic", topic_name,
            "--model", LaunchConfiguration("model"),
            "--package-share", pkg_share,
            "--mesh-root", "model://rokae_xmate3_ros2/meshes/",
            "--enable-ros2-control", resolved_enable_ros2_control_expression(),
            "--enable-xcore-plugin", resolved_enable_xcore_plugin_expression(),
            "--backend-mode", resolved_backend_mode_expression(),
            "--service-exposure-profile", resolved_service_profile_expression(),
            "--compatibility-alias-policy", resolved_compatibility_alias_policy_expression(),
            "--canonical-model", resolve_canonical_model(pkg_share),
            "--canonical-metadata", resolve_canonical_metadata(pkg_share),
            "--allow-noncanonical-model", LaunchConfiguration("allow_noncanonical_model"),
            "--use-sim-time", LaunchConfiguration("use_sim_time"),
            "--lifetime-sec", "30",
        ],
        output="screen",
        additional_env={"PATH": os.environ.get("PATH", ""), "PYTHONHOME": ""},
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
    canonical_urdf = os.path.join(pkg_share, "generated", "urdf", "xMateER3.urdf")
    canonical_metadata = os.path.join(pkg_share, "generated", "urdf", "xMateER3.description.json")

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
    try:
        gazebo_ros_share = get_package_share_directory("gazebo_ros")
        gazebo_ros_prefix = os.path.dirname(os.path.dirname(gazebo_ros_share))
        gazebo_ros_lib_dir = os.path.join(gazebo_ros_prefix, "lib")
        if os.path.isdir(gazebo_ros_lib_dir):
            gazebo_plugin_path_entries.append(gazebo_ros_lib_dir)
    except PackageNotFoundError:
        pass
    if os.path.isdir(pkg_lib_dir):
        gazebo_plugin_path_entries.append(pkg_lib_dir)

    actions = [
        launch.actions.SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.pathsep.join(gazebo_model_path_entries)),
        launch.actions.SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", os.pathsep.join(gazebo_resource_path_entries)),
        launch.actions.SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", os.pathsep.join(gazebo_plugin_path_entries)),
        launch.actions.SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        launch.actions.SetEnvironmentVariable("ROKAE_SERVICE_EXPOSURE_PROFILE", resolved_service_profile_expression()),
        launch.actions.SetEnvironmentVariable("ROKAE_COMPATIBILITY_ALIAS_POLICY", resolved_compatibility_alias_policy_expression()),
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


def build_spawn_entity_action(topic_name="/robot_description"):
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
            "-topic", topic_name,
            "-entity", "xmate",
        ],
        output="screen",
        additional_env={"PATH": os.environ.get("PATH", ""), "PYTHONHOME": ""},
    )


def build_jtc_readiness_probe(pkg_share, enabled_condition):
    probe_script = os.path.join(pkg_share, "tools", "check_gazebo_jtc_readiness.py")
    return launch.actions.ExecuteProcess(
        cmd=[
            sys.executable,
            probe_script,
            "--controller-manager", "/controller_manager",
            "--controller", "joint_trajectory_controller",
            "--action", "/joint_trajectory_controller/follow_joint_trajectory",
            "--timeout-sec", "120",
        ],
        output="screen",
        condition=launch.conditions.IfCondition(PythonExpression([
            "'", enabled_condition, "' == 'true' and '", LaunchConfiguration("require_runtime_readiness"), "' == 'true'"
        ])),
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


def build_spawn_exit_handler(spawn_entity_node, ros2_control_enabled, joint_state_broadcaster_spawner, joint_trajectory_controller_spawner, jtc_readiness_probe):
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
                        jtc_readiness_probe,
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
                log_info(["当前 service_exposure_profile=", resolved_service_profile_expression(), " compatibility_alias_policy=", resolved_compatibility_alias_policy_expression()]),
                log_info(["模型来源=", LaunchConfiguration("model")]),
                log_info(["allow_noncanonical_model=", LaunchConfiguration("allow_noncanonical_model")]),
                log_info(["RT 能力级别: experimental  diagnostics backend=", resolved_backend_mode_expression(), " runtime_profile=", resolved_runtime_profile_expression()]),
                log_info(["服务/话题命名由 compatibility_alias_policy 控制，当前策略=", resolved_compatibility_alias_policy_expression()]),
                log_info("  - canonical 允许时发布: /xmate_er3/cobot/*, /xmate_er3/joint_states, /xmate_er3/cobot/runtime_status"),
                log_info("  - compatibility 显式允许时发布: /xmate3/cobot/*, /xmate3/joint_states, /xmate3/internal/runtime_status"),
                log_info("  - RT/拖动/路径服务仅在 service_exposure_profile=public_xmate_er3_experimental 或 internal_full 时发布"),
                log_info("  - internal service 仅在 service_exposure_profile=internal_full 时发布: /xmate3/internal/validate_motion"),
                log_info("兼容别名默认关闭；需要旧 /xmate3 名称时设置 compatibility_alias_policy:=canonical_plus_compat"),
                log_info("运行示例程序:"),
                log_info("  ros2 run rokae_xmate3_ros2 example_04_motion_basic"),
                log_info("=" * 60),
            ],
        )
    )


def build_rviz_node(pkg_share):
    rviz_config = os.path.join(pkg_share, "config", "xMateER3.rviz")
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=launch.conditions.IfCondition(LaunchConfiguration("rviz")),
    )
