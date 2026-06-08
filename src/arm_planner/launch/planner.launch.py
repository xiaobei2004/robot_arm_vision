from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_planner',
            executable='planner_node',
            name='planner',
            parameters=['$(find arm_planner)/config/planner_params.yaml'],
            output='screen',
        ),
    ])
