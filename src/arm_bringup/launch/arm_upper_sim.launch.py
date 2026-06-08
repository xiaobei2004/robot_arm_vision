from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # 机械臂描述
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('arm_description'), 'urdf', 'arm.urdf.xacro'
        ])
    ])

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('arm_controller_config'),
        'config',
        'arm_controllers.yaml'
    ])

    # 节点列表
    nodes = []

    # robot_state_publisher
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    ))

    # controller_manager
    nodes.append(Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_content},
                    robot_controllers],
        output='screen',
    ))

    # 加载并启动 joint_trajectory_controller
    # 注意：spawner 不带 .py
    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '-c', '/controller_manager'],
        output='screen',
    ))

    # 加载并启动 joint_state_broadcaster
    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    ))

    # rviz
    rviz_config = os.path.join(
        FindPackageShare('arm_description').find('arm_description'),
        'rviz',
        'arm_view.rviz'
    )
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen'
    ))

    return LaunchDescription(nodes)
