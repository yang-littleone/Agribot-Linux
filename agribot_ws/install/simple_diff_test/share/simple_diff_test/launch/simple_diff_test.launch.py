import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 查找包路径
    pkg_share = FindPackageShare('simple_diff_test')

    # URDF文件路径
    urdf_file = 'urdf/simple_diff_robot.urdf.xacro'
    urdf_path = PathJoinSubstitution([pkg_share, urdf_file])

    # 声明启动参数
    declare_urdf_path = DeclareLaunchArgument(
        'urdf',
        default_value=urdf_path,
        description='Path to robot URDF file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz if true'
    )

    # 处理URDF
    robot_description = Command(['xacro ', LaunchConfiguration('urdf')])

    # Robot State Publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # Gazebo启动
    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('gazebo_ros'),
            'launch',
            'gazebo.launch.py'
        ]),
        launch_arguments={
            'verbose': 'true',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Spawn机器人实体
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_diff_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # 延迟spawn以确保Gazebo完全启动
    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn_entity_node]
    )

    # 加载joint_state_broadcaster
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # 加载diff_drive_controller
    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['simple_diff_controller'],
        output='screen'
    )

    # RViz节点（可选）
    rviz_config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'diaplay_robot_model.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        declare_urdf_path,
        declare_use_sim_time,
        declare_rviz,
        robot_state_publisher_node,
        gazebo_launch,
        delayed_spawn,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        rviz_node
    ])
