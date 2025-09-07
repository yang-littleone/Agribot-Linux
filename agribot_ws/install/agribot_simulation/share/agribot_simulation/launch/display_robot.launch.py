import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory #通过功能包的名字找到 share 目录
import os

def generate_launch_description():
    # 获取默认的urdf路径
    urdf_package_path = get_package_share_directory('agribot_simulation')
    default_urdf_path = os.path.join(urdf_package_path, 'urdf/agribot','agribot.urdf.xacro')
    # 获取默认的rviz配置文件路径
    default_rviz_config_path = os.path.join(urdf_package_path, 'config','diaplay_robot_model.rviz')
    # 声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_urdf_path),description='URDF的绝对路径'
    )
    """ 通过文件路径，获取内容，并转化为参数值对象，以供传入 robot_state_publisher """
    # 获取文件内容
    substitutions_command_result = launch.substitutions.Command(command=['xacro ',launch.substitutions.LaunchConfiguration('model')])
    # 将内容转换为参数值对象
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result,value_type=str)

    # 状态发布节点
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]
    )

    # 关节状态发布节点
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    # RViz 节点
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        # 使用保存的配置文件
        arguments=['-d', default_rviz_config_path],
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node,
    ])
