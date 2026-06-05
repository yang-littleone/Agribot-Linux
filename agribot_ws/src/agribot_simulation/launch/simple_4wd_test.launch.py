import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch_ros.parameter_descriptions import ParameterValue
import os
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # 获取包路径
    package_share_path = get_package_share_directory('agribot_simulation')
    
    # URDF路径
    default_urdf_path = os.path.join(package_share_path, 'urdf', 'simple_4wd_test.urdf')
    
    # Gazebo世界路径（使用空世界）
    default_gazebo_world_path = os.path.join(package_share_path, 'world', 'empty.world')
    
    # 声明URDF路径参数
    action_declare_arg_model = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_urdf_path),
        description='URDF的绝对路径'
    )
    
    # 使用xacro处理URDF文件
    robot_description_value = ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )
    
    # Robot State Publisher
    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_value,
            'use_sim_time': True
        }]
    )
    
    # 启动Gazebo
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments=[('world', default_gazebo_world_path)]
    )
    
    # Spawn机器人到Gazebo
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', 
                   '-entity', 'simple_4wd_test',
                   '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )
    
    # 加载控制器
    action_load_joint_state_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['simple_4wd_joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    action_load_diff_drive_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['simple_4wd_diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    return launch.LaunchDescription([
        action_declare_arg_model,
        node_robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,
        # 事件动作，当加载机器人实体结束后执行
        launch.actions.RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_state_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=action_load_joint_state_controller,
                on_exit=[action_load_diff_drive_controller],
            )
        ),
    ])
