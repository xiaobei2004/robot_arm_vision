from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'arm_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Visual detection module using YOLOv8 for robot grasping',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detector_node = arm_vision.detector_node:main',
            'yolov8_node = arm_vision.yolov8_node:main',
            'image_viewer = arm_vision.image_viewer:main',
        ],
    },
)
