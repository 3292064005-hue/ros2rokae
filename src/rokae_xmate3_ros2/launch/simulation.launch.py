import launch
import launch_ros
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def load_yaml(file_path):
    if not os.path.exists(file_path):
        return {}
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    return data if isinstance(data, dict) else {}


def generate_launch_description():
    """
    xMate3 仿真启动文件

    功能:
    - 启动 Gazebo 仿真环境
    - 加载 xMate3 机器人模型 (使用 xcore_controller_gazebo_plugin)
    - 可选启动 MoveIt2 (moveit:=true)
    - 启动 RViz 可视化

    使用方式:
    - 默认启动: ros2 launch rokae_xmate3_ros2 simulation.launch.py
    - 带MoveIt2: ros2 launch rokae_xmate3_ros2 simulation.launch.py moveit:=true
    - 无GUI: ros2 launch rokae_xmate3_ros2 simulation.launch.py gui:=false
    - 不启动RViz: ros2 launch rokae_xmate3_ros2 simulation.launch.py rviz:=false
    """
    pkg_share = get_package_share_directory("rokae_xmate3_ros2")
    moveit_config_dir = os.path.join(pkg_share, "config", "moveit")
    existing_gazebo_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    existing_gazebo_resource_path = os.environ.get("GAZEBO_RESOURCE_PATH", "")
    gazebo_model_root = os.path.dirname(pkg_share)


    # 文件路径
    urdf_file = os.path.join(pkg_share, "urdf", "xMate3.xacro")
    world_file = os.path.join(pkg_share, "worlds", "empty.world")
    srdf_file = os.path.join(moveit_config_dir, "xMate3.srdf")

    # RViz配置
    rviz_config = os.path.join(pkg_share, "config", "xMate3.rviz")
    rviz_config_moveit = os.path.join(pkg_share, "config", "xMate3_moveit.rviz")
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(pkg_share, "launch", "xMate3.rviz")

    # 声明启动参数
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
            name='moveit',
            default_value='false',
            description='是否启动 MoveIt2 运动规划'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='使用仿真时间'
        ),
    ]

    # ========== 机器人描述 (使用 xacro 处理) ==========
    robot_description_content = launch.substitutions.Command(
        ['xacro ', launch.substitutions.LaunchConfiguration('model')]
    )
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        robot_description_content, value_type=str
    )

    # ========== 加载 SRDF ==========
    srdf_content = ""
    if os.path.exists(srdf_file):
        with open(srdf_file, 'r') as f:
            srdf_content = f.read()

    # ========== robot_state_publisher ==========
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

    # ========== 启动 Gazebo ==========
    set_gazebo_model_path = launch.actions.SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=(existing_gazebo_model_path + os.pathsep + gazebo_model_root) if existing_gazebo_model_path else gazebo_model_root
    )
    gazebo_resource_root = "/usr/share/gazebo-11"

    set_gazebo_resource_path = launch.actions.SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=gazebo_resource_root + os.pathsep + pkg_share
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

    # ========== 生成实体 (spawn robot) ==========
    # 显式固定到系统 Python，避免当前 shell 里的 conda/miniconda 覆盖 ROS Humble 的 python ABI。
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

    # ========== 启动提示信息 ==========
    def log_info(msg):
        return launch.actions.LogInfo(msg=msg)

    # 定义事件处理器 - 在spawn退出后提醒用户检查结果，避免失败时误报“启动完成”
    on_spawn_exit = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                log_info("=" * 60),
                log_info("spawn_entity.py 已退出，请检查上方输出确认机器人是否成功生成。"),
                log_info("若看到 'Successfully spawned entity [xmate]'，则说明机器人已正确加载。"),
                log_info("使用 xcore_controller_gazebo_plugin 提供 SDK 仿真。"),
                log_info("可用服务和话题:"),
                log_info("  - /xmate3/cobot/*  (SDK服务)"),
                log_info("  - /xmate3/joint_states  (关节状态)"),
                log_info("运行示例程序:"),
                log_info("  ros2 run rokae_xmate3_ros2 example_04_motion_basic"),
                log_info("=" * 60),
            ]
        )
    )

    # ========== 创建两个 Group: 一个带 MoveIt2，一个不带 ==========
    # 不带 MoveIt2 的节点
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

    # 不带 MoveIt2 的组
    group_no_moveit = launch.actions.GroupAction(
        condition=launch.conditions.UnlessCondition(
            launch.substitutions.LaunchConfiguration('moveit')
        ),
        actions=[
            log_info("正在启动 xMate3 仿真环境..."),
            set_gazebo_model_path,
            set_gazebo_resource_path,
            robot_state_publisher_node,
            launch_gazebo,
            spawn_entity_node,
            on_spawn_exit,
            rviz_node,
        ]
    )

    # 带 MoveIt2 的组
    if os.path.exists(srdf_file):
        # MoveIt2 配置文件
        ompl_planning = load_yaml(os.path.join(moveit_config_dir, "ompl_planning.yaml"))
        kinematics = load_yaml(os.path.join(moveit_config_dir, "kinematics.yaml"))
        joint_limits = load_yaml(os.path.join(moveit_config_dir, "joint_limits.yaml"))
        moveit_controllers = load_yaml(os.path.join(moveit_config_dir, "moveit_controllers.yaml"))

        move_group_node = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': srdf_content},
                {'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')},
                {'publish_robot_description_semantic': True},
                {'publish_robot_description': True},
                {'allow_trajectory_execution': True},
                {'publish_monitored_planning_scene': True},
                {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
                {'capabilities':
                    'move_group/MoveGroupCartesianPathService '
                    'move_group/MoveGroupExecuteService '
                    'move_group/MoveGroupKinematicsService '
                    'move_group/MoveGroupMoveAction '
                    'move_group/MoveGroupPlanService '
                    'move_group/MoveGroupQueryPlannersService '
                    'move_group/MoveGroupStateValidationService '},
                {'disable_capabilities': ''},
                {'max_velocity_scaling_factor': 0.3},
                {'max_acceleration_scaling_factor': 0.3},
                {'planning_plugin': 'ompl_interface/OMPLPlanner'},
                ompl_planning,
                kinematics,
                joint_limits,
                moveit_controllers,
            ],
            remappings=[
                ('/joint_states', '/xmate3/joint_states')
            ]
        )

        rviz_node_moveit = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_moveit],
            output='screen',
            parameters=[
                {'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')},
                {'robot_description': robot_description},
                {'robot_description_semantic': srdf_content},
                kinematics,
                joint_limits,
                ompl_planning,
            ],
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('rviz'))
        )

        on_spawn_exit_moveit = launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[
                    log_info("=" * 60),
                    log_info("xMate3 仿真环境 + MoveIt2 启动完成！"),
                    log_info("=" * 60),
                    log_info("使用 xcore_controller_gazebo_plugin + MoveIt2"),
                    log_info("=" * 60),
                    log_info("可用功能:"),
                    log_info("  - /xmate3/cobot/*  (SDK服务)"),
                    log_info("  - /xmate3/joint_states  (关节状态)"),
                    log_info("  - /move_group  (MoveIt2运动规划)"),
                    log_info("=" * 60),
                    log_info("运行示例程序:"),
                    log_info("  ros2 run rokae_xmate3_ros2 example_99_complete_demo"),
                    log_info("=" * 60),
                ]
            )
        )

        group_with_moveit = launch.actions.GroupAction(
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('moveit')
            ),
            actions=[
                log_info("正在启动 xMate3 仿真环境 + MoveIt2..."),
                set_gazebo_model_path,
                set_gazebo_resource_path,
                robot_state_publisher_node,
                launch_gazebo,
                spawn_entity_node,
                move_group_node,
                on_spawn_exit_moveit,
                rviz_node_moveit,
            ]
        )
    else:
        # 如果没有 SRDF 文件，带 MoveIt2 的组只是简单提示
        group_with_moveit = launch.actions.GroupAction(
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('moveit')
            ),
            actions=[
                log_info("MoveIt2 配置文件未找到，启动原版仿真..."),
                log_info("正在启动 xMate3 仿真环境..."),
                robot_state_publisher_node,
                launch_gazebo,
                spawn_entity_node,
                on_spawn_exit,
                rviz_node,
            ]
        )

    return launch.LaunchDescription(
        declared_arguments + [
            group_no_moveit,
            group_with_moveit,
        ]
    )
