import os
import launch
import launch_ros
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _resolve_package_share():
    env_share = os.environ.get("ROKAE_XMATE3_ROS2_SHARE_DIR", "")
    if env_share and os.path.isdir(env_share):
        return env_share
    try:
        return get_package_share_directory("rokae_xmate3_ros2")
    except PackageNotFoundError:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _resolve_package_lib_dir(pkg_share):
    env_lib = os.environ.get("ROKAE_XMATE3_ROS2_LIB_DIR", "")
    if env_lib and os.path.isdir(env_lib):
        return env_lib
    pkg_prefix = os.path.dirname(os.path.dirname(pkg_share))
    return os.path.join(pkg_prefix, "lib")


def generate_launch_description():
    """
    xMate3 纯 Gazebo 仿真启动文件。

    功能:
    - 启动 Gazebo 仿真环境
    - 加载 xMate3 机器人模型 (使用 xcore_controller_gazebo_plugin)
    - 启动 RViz 可视化

    使用方式:
    - 默认启动: ros2 launch rokae_xmate3_ros2 simulation.launch.py
    - 无GUI: ros2 launch rokae_xmate3_ros2 simulation.launch.py gui:=false
    - 不启动RViz: ros2 launch rokae_xmate3_ros2 simulation.launch.py rviz:=false
    """
    pkg_share = _resolve_package_share()
    pkg_lib_dir = _resolve_package_lib_dir(pkg_share)

    existing_gazebo_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    existing_gazebo_resource_path = os.environ.get("GAZEBO_RESOURCE_PATH", "")
    existing_gazebo_plugin_path = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    gazebo_worlds_dir = os.path.join(pkg_share, "worlds")
    gazebo_models_dir = os.path.join(pkg_share, "models")

    urdf_file = os.path.join(pkg_share, "urdf", "xMate3.xacro")
    world_file = os.path.join(pkg_share, "worlds", "empty.world")
    rviz_config = os.path.join(pkg_share, "config", "xMate3.rviz")

    declared_arguments = [
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=str(urdf_file),
            description='URDF/xacro 模型文件路径'
        ),
        launch.actions.DeclareLaunchArgument(
            name='world',
            default_value=str(world_file),
            description='Gazebo world 文件路径'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='true',
            description='是否启动 Gazebo GUI'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='是否启动 RViz'
        ),
        launch.actions.DeclareLaunchArgument(
            name='verbose',
            default_value='true',
            description='Gazebo 详细输出'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='使用仿真时间'
        ),
        launch.actions.DeclareLaunchArgument(
            name='enable_ros2_control',
            default_value='false',
            description='是否启用 ros2_control / joint_trajectory_controller'
        ),
    ]

    robot_description_content = launch.substitutions.Command(
        [
            'xacro ',
            launch.substitutions.LaunchConfiguration('model'),
            ' mesh_root:=model://rokae_xmate3_ros2/meshes/',
            ' package_share:=',
            pkg_share,
            ' enable_ros2_control:=',
            launch.substitutions.LaunchConfiguration('enable_ros2_control'),
        ]
    )
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        robot_description_content, value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/joint_states', '/xmate3/joint_states')
        ]
    )

    gazebo_model_path_entries = []
    if existing_gazebo_model_path:
        gazebo_model_path_entries.append(existing_gazebo_model_path)
    if os.path.isdir(gazebo_models_dir):
        gazebo_model_path_entries.append(gazebo_models_dir)
    if os.path.isdir(gazebo_worlds_dir):
        gazebo_model_path_entries.append(gazebo_worlds_dir)
    set_gazebo_model_path = launch.actions.SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.pathsep.join(gazebo_model_path_entries)
    )

    gazebo_resource_root = "/usr/share/gazebo-11"
    gazebo_resource_path_entries = []
    if existing_gazebo_resource_path:
        gazebo_resource_path_entries.append(existing_gazebo_resource_path)
    gazebo_resource_path_entries.extend([gazebo_resource_root, pkg_share])
    set_gazebo_resource_path = launch.actions.SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=os.pathsep.join(gazebo_resource_path_entries)
    )

    gazebo_plugin_path_entries = []
    if existing_gazebo_plugin_path:
        gazebo_plugin_path_entries.append(existing_gazebo_plugin_path)
    if os.path.isdir(pkg_lib_dir):
        gazebo_plugin_path_entries.append(pkg_lib_dir)
    set_gazebo_plugin_path = launch.actions.SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=os.pathsep.join(gazebo_plugin_path_entries)
    )

    gazebo_launch_dir = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch')
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': launch.substitutions.LaunchConfiguration('world'),
            'verbose': launch.substitutions.LaunchConfiguration('verbose'),
            'gui': launch.substitutions.LaunchConfiguration('gui'),
            'pause': 'false',
        }.items()
    )

    spawn_entity_node = launch.actions.ExecuteProcess(
        cmd=[
            '/usr/bin/python3',
            '/opt/ros/humble/lib/gazebo_ros/spawn_entity.py',
            '-topic', '/robot_description',
            '-entity', 'xmate'
        ],
        output='screen',
        additional_env={
            'PATH': '/usr/bin:/bin:' + os.environ.get('PATH', ''),
            'PYTHONHOME': ''
        }
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
        parameters=[{'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')}],
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration('enable_ros2_control'))
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
        parameters=[{'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')}],
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration('enable_ros2_control'))
    )

    def log_info(msg):
        return launch.actions.LogInfo(msg=msg)

    on_spawn_exit = launch.actions.RegisterEventHandler(
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
                    condition=launch.conditions.IfCondition(
                        launch.substitutions.LaunchConfiguration('enable_ros2_control'))
                ),
                log_info("=" * 60),
                log_info("spawn_entity.py 已退出，请检查上方输出确认机器人是否成功生成。"),
                log_info("若看到 'Successfully spawned entity [xmate]'，则说明机器人已正确加载。"),
                log_info("使用 xcore_controller_gazebo_plugin 提供纯 xCore SDK 仿真。"),
                log_info("默认使用 xCore Gazebo runtime；如需 joint_trajectory_controller，请传入 enable_ros2_control:=true。"),
                log_info("可用服务和话题:"),
                log_info("  - /xmate3/cobot/*  (SDK服务)"),
                log_info("  - /xmate3/joint_states  (关节状态)"),
                log_info("运行示例程序:"),
                log_info("  ros2 run rokae_xmate3_ros2 example_04_motion_basic"),
                log_info("=" * 60),
            ]
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')}],
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration('rviz'))
    )

    return launch.LaunchDescription(
        declared_arguments + [
            log_info("正在启动 xMate3 纯 Gazebo 仿真环境..."),
            set_gazebo_model_path,
            set_gazebo_resource_path,
            set_gazebo_plugin_path,
            robot_state_publisher_node,
            launch_gazebo,
            spawn_entity_node,
            on_spawn_exit,
            rviz_node,
        ]
    )
