import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
import os
from launch.actions import TimerAction

def generate_launch_description():
    urdf_package_share_path = get_package_share_directory('agribot_simulation')
    default_gazebo_world_path = os.path.join(urdf_package_share_path, 'world', 'empty.world')
    
    default_urdf_path = os.path.join(urdf_package_share_path, 'urdf', 'test_robot/test_robot.urdf.xacro')
    
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_urdf_path), description='URDF的绝对路径'
    )
    
    substitutions_command_result = launch.substitutions.Command(command=['xacro ', launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = ParameterValue(substitutions_command_result, value_type=str)
    
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value, 'use_sim_time': True}]
    )
    
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments=[('world', default_gazebo_world_path)]
    )
    
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'test_robot',
                   '-x', '0', '-y', '0', '-z', '0.1']
    )
    
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        TimerAction(
            period=2.0,
            actions=[action_spawn_entity]
        ),
    ])