from setuptools import setup, find_packages

package_name = 'arm_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/planner.launch.py']),
        ('share/' + package_name + '/config', ['config/planner_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='6-axis arm path planner',
    license='MIT',
    entry_points={
        'console_scripts': [
            'planner_node = arm_planner.planner_node:main',   # ← 必须存在这一行
        ],
    },
)
