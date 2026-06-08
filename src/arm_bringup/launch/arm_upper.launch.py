from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 启动 display.launch.py（包含所有核心组件）
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            FindPackageShare('arm_description').find('arm_description'),
            'launch', 'display.launch.py'
        )])
    )
    
    # 启动规划节点
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            FindPackageShare('arm_planner').find('arm_planner'),
            'launch', 'planner.launch.py'
        )])
    )
    
    # 可选：启动 vision
    # vision_launch = ...
    
    return LaunchDescription([
        display_launch,
        planner_launch,
    ])
