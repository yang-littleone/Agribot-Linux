import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory #通过功能包的名字找到 share 目录
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch_ros.parameter_descriptions import ParameterValue
import os
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    # 获取默认的urdf路径
    urdf_package_share_path = get_package_share_directory('agribot_simulation')
    default_gazebo_world_path = os.path.join(urdf_package_share_path, 'world','cornlinens_angular2.world')
    # 获取默认的rviz配置文件路径
    # default_rviz_config_path = os.path.join(urdf_package_share_path, 'config','xkaibot_model.rviz')
    default_urdf_path = os.path.join(urdf_package_share_path, 'urdf','agribot/agribot.urdf.xacro')
    # 获取默认的rviz配置文件路径    # 声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_urdf_path),description='URDF的绝对路径'
    )
    """ 通过文件路径，获取内容，并转化为参数值对象，以供传入 robot_state_publisher """
    # 获取文件内容
    substitutions_command_result = launch.substitutions.Command(command=['xacro ',launch.substitutions.LaunchConfiguration('model')])
    # 将内容转换为参数值对象
    robot_description_value = ParameterValue(substitutions_command_result,value_type=str)

    # 状态发布节点
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]
    )
    
    """
        ros2 launch gazebo_ros gazebo.launch.py world:=xxx.world
        使用功能包提供的launch文件以指定 .world 启动 gazebo 仿真环境
    """
    # 在launch中启动别的launch文件
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'),'/launch', '/gazebo.launch.py']),
        launch_arguments=[('world',default_gazebo_world_path), ('verbose', 'true')]
    )
    
    """ 
        这个节点的作用是将之前通过robot_state_publisher发布的机器人描述信息（URDF模型）作为实体
        加载到Gazebo仿真环境中，使机器人能够在仿真世界中进行物理交互。
    """
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'agribot', 
                   '-x', '0', '-y', '0', '-z', '0.1',  # 设置初始位置，z为0.1确保机器人在地面以上
                   '-R', '0', '-P', '0', '-Y', '0']    # 设置初始方向
    )
    
    # package_name = 'ros2_livox_simulation'
    # pkg_share = os.pathsep + os.path.join(get_package_prefix(package_name), 'share')
    # if 'GAZEBO_MODEL_PATH' in os.environ:
    #     os.environ['GAZEBO_MODEL_PATH'] += pkg_share
    # else:
    #     os.environ['GAZEBO_MODEL_PATH'] = "/usr/share/gazebo-11/models" + pkg_share
        
    # 使用controller_manager的spawner节点加载控制器，它会自动等待controller_manager服务就绪
    action_load_joint_state_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['agribot_joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    action_load_diff_drive_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['agribot_diff_drive_controller', '--controller-manager', '/controller_manager'],
    )

    action_load_effort_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['agribot_effort_controller', '--controller-manager', '/controller_manager'],
    )
     
    
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
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