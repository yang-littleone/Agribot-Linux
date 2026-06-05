import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch_ros.parameter_descriptions import ParameterValue
import os
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction

def generate_launch_description():
    # 获取包路径
    package_share_path = get_package_share_directory('agribot_simulation')
    
    # URDF路径
    default_urdf_path = os.path.join(package_share_path, 'urdf', 'simple_4wd_robot.urdf')
    
    # Gazebo世界路径
    default_gazebo_world_path = os.path.join(package_share_path, 'world', 'corn_leaf_world.world')
    
    # 声明URDF路径参数
    action_declare_arg_model = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_urdf_path),
        description='URDF的绝对路径'
    )
    
    # 读取URDF文件
    robot_description_content = ParameterValue(
        open(default_urdf_path, 'r').read(),
        value_type=str
    )
    
    # Robot State Publisher
    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
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
    
    # Spawn机器人实体
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_4wd_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0'
        ]
    )
    
    # 生成控制器管理器spawner
    controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'simple_4wd_joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30'
        ]
    )
    
    diff_drive_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'simple_4wd_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30'
        ]
    )
    
    # 延迟启动控制器，等待Gazebo完全加载
    delayed_controller_spawner = TimerAction(
        period=5.0,
        actions=[controller_spawner]
    )
    
    delayed_diff_drive_spawner = TimerAction(
        period=7.0,
        actions=[diff_drive_spawner]
    )
    
    return launch.LaunchDescription([
        action_declare_arg_model,
        node_robot_state_publisher,
        action_launch_gazebo,
        TimerAction(
            period=3.0,
            actions=[spawn_entity_node]
        ),
        delayed_controller_spawner,
        delayed_diff_drive_spawner,
    ])
