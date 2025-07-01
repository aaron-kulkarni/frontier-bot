from setuptools import setup, find_packages
import glob

package_name = 'turtlebot3_initial_pose_pub'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaronk',
    maintainer_email='aaronk@todo.todo',
    description='Standalone initial pose publisher node for TurtleBot3 simulation (ROS 2).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_pub = turtlebot3_initial_pose_pub.initial_pose_pub:main',
        ],
    },
)
