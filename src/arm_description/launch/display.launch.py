import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ------------------------------------------------------------------
    # 1. 声明参数
    # ------------------------------------------------------------------
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )

    # ------------------------------------------------------------------
    # 2. 解析 URDF
    # ------------------------------------------------------------------
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('arm_description'),
            'urdf',
            'arm.urdf.xacro'
        ])
    ])

    robot_description = ParameterValue(
        robot_description_content,
        value_type=str
    )

    # ------------------------------------------------------------------
    # 3. 控制器配置文件路径
    # ------------------------------------------------------------------
    controller_config = os.path.join(
        get_package_share_directory('arm_controller_config'),
        'config',
        'arm_controllers.yaml'
    )

    # ------------------------------------------------------------------
    # 4. robot_state_publisher
    # ------------------------------------------------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
        }],
    )

    # ------------------------------------------------------------------
    # 5. controller_manager
    # ------------------------------------------------------------------
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_config,
        ],
        output='screen',
    )

    # ------------------------------------------------------------------
    # 6. spawn joint_state_broadcaster
    # ------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', controller_config,
        ],
        output='screen',
    )

    # ------------------------------------------------------------------
    # 7. spawn joint_trajectory_controller
    # ------------------------------------------------------------------
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '-c', '/controller_manager',
            '--param-file', controller_config,
        ],
        output='screen',
    )

    # ------------------------------------------------------------------
    # 8. RViz2（可选）
    # ------------------------------------------------------------------
    rviz_config = os.path.join(
        get_package_share_directory('arm_description'),
        'rviz',
        'arm_view.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    # ------------------------------------------------------------------
    # 9. 返回所有节点
    # ------------------------------------------------------------------
    return LaunchDescription([
        use_rviz_arg,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        rviz_node,
    ])
