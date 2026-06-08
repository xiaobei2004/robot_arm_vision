import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 控制器配置文件路径
    controller_config = os.path.join(
        get_package_share_directory('arm_controller_config'),
        'config',
        'arm_controllers.yaml'
    )

    # ros2_control_node（生命周期节点）
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen',
    )

    # spawner 节点：启动 joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # spawner 节点：启动 joint_trajectory_controller
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '-c', '/controller_manager',
        ],
        output='screen',
    )

    # 按顺序启动：先启动 controller_manager，再 spawn 控制器
    # 使用 event handler 确保顺序
    delayed_spawner_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delayed_spawner_trajectory = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    return LaunchDescription([
        controller_manager_node,
        delayed_spawner_broadcaster,
        delayed_spawner_trajectory,
    ])
