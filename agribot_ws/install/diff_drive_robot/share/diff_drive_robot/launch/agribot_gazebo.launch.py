import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Get package path
    pkg_share = get_package_share_directory('diff_drive_robot')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    x_pos = LaunchConfiguration('x', default='0.0')
    y_pos = LaunchConfiguration('y', default='0.0')
    z_pos = LaunchConfiguration('z', default='0.1')
    yaw = LaunchConfiguration('yaw', default='0.0')
    robot_name = LaunchConfiguration('robot_name', default='agribot_diff_drive')

    # URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'agribot_diff_drive.urdf.xacro')

    # Process xacro to get URDF
    robot_description = Command(['xacro ', urdf_file])

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Gazebo launch
    gazebo_launch_dir = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch'
    )

    default_gazebo_world_path = os.path.join(
        get_package_share_directory('agribot_simulation'),
        'world',
        'twoworld.world'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')
        ),
        launch_arguments={
            'gui': gui,
            'world': default_gazebo_world_path,
            'verbose': 'true',
        }.items()
    )

    # Spawn robot in Gazebo - delayed to ensure Gazebo is ready
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name,
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos,
            '-Y', yaw
        ],
        output='screen'
    )

    # Load controllers after spawn
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Delayed actions
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[
            spawn_entity_node,
            TimerAction(
                period=2.0,
                actions=[load_joint_state_broadcaster]
            ),
            TimerAction(
                period=3.0,
                actions=[load_diff_drive_controller]
            ),
        ]
    )

    # Create launch description
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Flag to enable Gazebo GUI'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Initial x position in Gazebo'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Initial y position in Gazebo'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.1',
            description='Initial z position in Gazebo'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Initial yaw angle in Gazebo'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='agribot_diff_drive',
            description='Name of the robot entity in Gazebo'
        ),
        robot_state_publisher_node,
        gazebo_launch,
        delayed_spawn,
    ])

    return ld
