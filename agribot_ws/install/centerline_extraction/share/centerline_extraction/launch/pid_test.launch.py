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
    # Get package paths
    pkg_diff_drive = get_package_share_directory('diff_drive_robot')
    pkg_centerline = get_package_share_directory('centerline_extraction')
    pkg_agribot_sim = get_package_share_directory('agribot_simulation')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    rviz = LaunchConfiguration('rviz', default='true')
    open_loop = LaunchConfiguration('open_loop', default='true')

    # 1. Robot state publisher
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

    # 2. Gazebo launch
    gazebo_launch_dir = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch'
    )

    default_gazebo_world_path = os.path.join(pkg_agribot_sim, 'world', 'empty.world')

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

    # 3. Spawn robot in Gazebo
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

    # 4. PID Controller node
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
            {'debug_mode': False}
        ],
        output='screen'
    )

    # 5. PID Test node (publishes test path)
    pid_test_node = Node(
        package='centerline_extraction',
        executable='pid_controller_test',
        name='pid_controller_test',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'use_open_loop': open_loop}
        ],
        output='screen'
    )

    # 6. RViz2
    rviz_config_path = os.path.join(pkg_centerline, 'config', 'pid_test.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=LaunchConfigurationEquals(rviz, 'true'),
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
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Flag to enable RViz'
        ),
        DeclareLaunchArgument(
            'open_loop',
            default_value='true',
            description='Use open-loop movement for testing (true=open loop, false=PID control)'
        ),
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,
        
        # Start PID controller and test node after robot is spawned
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[pid_controller_node, pid_test_node],
            )
        ),
        
        rviz_node,
    ])

    return ld


class LaunchConfigurationEquals(IfCondition):
    def __init__(self, lhs: LaunchConfiguration, rhs: str):
        super().__init__(EqualsSubstitution(lhs, rhs))