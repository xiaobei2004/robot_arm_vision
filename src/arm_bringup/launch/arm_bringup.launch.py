from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    # 启动 display.launch.py（含 model + controllers + rviz）
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                FindPackageShare('arm_description').find('arm_description'),
                'launch',
                'display.launch.py'
            )
        ])
    )

    # 可在这里添加 planner / vision 等
    return LaunchDescription([
        display_launch,
    ])
