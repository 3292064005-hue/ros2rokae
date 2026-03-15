import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    xMate3 仿真启动文件

    功能:
    - 启动 Gazebo 仿真环境
    - 加载 xMate3 机器人模型 (使用 xcore_controller_gazebo_plugin)
    - 启动 RViz 可视化

    使用方式:
    - 默认启动: ros2 launch rokae_xmate3_ros2 simulation.launch.py
    - 无GUI: ros2 launch rokae_xmate3_ros2 simulation.launch.py gui:=false
    - 不启动RViz: ros2 launch rokae_xmate3_ros2 simulation.launch.py rviz:=false
    """
    pkg_share = get_package_share_directory("rokae_xmate3_ros2")

    # 文件路径
    urdf_file = os.path.join(pkg_share, "urdf", "xMate3.xacro")
    world_file = os.path.join(pkg_share, "worlds", "empty.world")

    # RViz配置兼容新旧位置
    rviz_config = os.path.join(pkg_share, "config", "xMate3.rviz")
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
    ]

    # 机器人描述 (使用 xacro 处理)
    robot_description_content = launch.substitutions.Command(
        ['xacro ', launch.substitutions.LaunchConfiguration('model')]
    )
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        robot_description_content, value_type=str
    )

    # robot_state_publisher 节点 - 从 /xmate3/joint_states 读取
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ],
        remappings=[
            ('/joint_states', '/xmate3/joint_states')
        ]
    )

    # 启动 Gazebo
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
        }.items()
    )

    # 生成实体 (spawn robot)
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', '/robot_description', '-entity', 'xmate']
    )

    # RViz2 节点 (条件启动)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=launch.conditions.IfCondition(
            launch.substitutions.LaunchConfiguration('rviz'))
    )

    # 启动提示信息
    def log_info(msg):
        return launch.actions.LogInfo(msg=msg)

    # 定义事件处理器 - 在spawn完成后显示提示
    on_spawn_exit = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                log_info("=" * 60),
                log_info("xMate3 仿真环境启动完成！"),
                log_info("=" * 60),
                log_info("使用 xcore_controller_gazebo_plugin 提供SDK仿真"),
                log_info("=" * 60),
                log_info("可用服务和话题:"),
                log_info("  - /xmate3/cobot/*  (SDK服务)"),
                log_info("  - /xmate3/joint_states  (关节状态)"),
                log_info("=" * 60),
                log_info("运行示例程序:"),
                log_info("  ros2 run rokae_xmate3_ros2 example_04_motion_basic"),
                log_info("=" * 60),
            ]
        )
    )

    return launch.LaunchDescription(
        declared_arguments +
        [
            log_info("正在启动 xMate3 仿真环境..."),
            robot_state_publisher_node,
            launch_gazebo,
            spawn_entity_node,
            on_spawn_exit,
            rviz_node,
        ]
    )
