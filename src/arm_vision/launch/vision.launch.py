from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    arm_vision_dir = get_package_share_directory('arm_vision')
    camera_param_file = os.path.join(arm_vision_dir, 'config', 'camera_params.yaml')
    
    return LaunchDescription([
        Node(
            package='arm_vision',
            executable='yolov8_node',
            name='yolov8_detector',
            output='screen',
            parameters=[{
                'rgb_camera_info_url': 'file://' + camera_param_file,
                'depth_camera_info_url': 'file://' + camera_param_file,
                'model_path': 'best.pt',   # 默认从当前运行目录加载，可改为包内路径
            }],
        ),
    ])
