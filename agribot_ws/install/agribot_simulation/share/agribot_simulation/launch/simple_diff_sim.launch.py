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
    default_urdf_path = os.path.join(package_share_path, 'urdf', 'simple_diff_robot.urdf')
    
    # Gazebo世界路径
    default_gazebo_world_path = os.path.join(package_share_path, 'world', 'corn_leaf_world.world')
    
    # 声明URDF路径参数
    action_declare_arg_model = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_urdf_path),
        description='URDF的绝对路径'
    )
    
    # 读取URDF文件
    robot_description_value = ParameterValue(
        launch.substitutions.Command(['cat ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )
    
    # 机器人状态发布节点
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}]
    )
    
    # 启动Gazebo
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments=[('world', default_gazebo_world_path)]
    )
    
    # 生成机器人实体到Gazebo
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'simple_diff_robot',
                   '-x', '0', '-y', '0', '-z', '0.15',
                   '-R', '0', '-P', '0', '-Y', '0']
    )
    
    # 加载关节状态控制器
    action_load_joint_state_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['simple_joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    
    # 加载差速驱动控制器
    action_load_diff_drive_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['simple_diff_drive_controller', '--controller-manager', '/controller_manager'],
    )
    
    return launch.LaunchDescription([
        action_declare_arg_model,
        action_robot_state_publisher,
        action_launch_gazebo,
        TimerAction(
            period=2.0,
            actions=[action_spawn_entity]
        ),
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
