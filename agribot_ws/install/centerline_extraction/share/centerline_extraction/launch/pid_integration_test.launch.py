import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, EqualsSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_diff_drive = get_package_share_directory('diff_drive_robot')
    pkg_centerline = get_package_share_directory('centerline_extraction')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    rviz = LaunchConfiguration('rviz', default='true')

    urdf_file = os.path.join(pkg_diff_drive, 'urdf', 'robot.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])

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

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'gui': gui,
            'verbose': 'true',
        }.items()
    )

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

    path_publisher_node = Node(
        package='centerline_extraction',
        executable='simple_test',
        name='path_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    pid_controller_node = Node(
        package='centerline_extraction',
        executable='cornfield_navigation_node',
        name='pid_controller',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'target_distance': 0.4},
            {'max_linear_speed': 0.4},
            {'min_linear_speed': 0.1},
            {'max_angular_speed': 1.0},
            {'lateral_kp': 20.0},
            {'lateral_ki': 1.0},
            {'lateral_kd': 10.0},
            {'heading_kp': 20.0},
            {'heading_ki': 1.0},
            {'heading_kd': 10.0},
            {'debug_mode': True}
        ],
        output='screen'
    )

    rviz_config_path = os.path.join(pkg_centerline, 'config', 'pid_test.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen'
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Enable Gazebo GUI'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Enable RViz'
        ),
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[path_publisher_node, pid_controller_node],
            )
        ),
        
        rviz_node,
    ])

    return ld