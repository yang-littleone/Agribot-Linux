import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Get package path
    pkg_share = get_package_share_directory('diff_drive_robot')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')

    # URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

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

    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'diff_drive_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.1',
            '-Y', '0'
        ],
        output='screen'
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
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,
    ])

    return ld
