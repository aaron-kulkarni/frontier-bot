from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'turtlebot3_autonav_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', []),
        ('lib/' + package_name, []),
    ],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs'],
    zip_safe=True,
    maintainer='aaronk',
    maintainer_email='aaronk@todo.todo',
    description='Python nodes for TurtleBot3 autonomous navigation (initial pose publisher, etc).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_pub = turtlebot3_autonav_py.initial_pose_pub:main',
        ],
    },
)
