from setuptools import find_packages, setup
import glob

package_name = 'turtlebot3_autonav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']) + ['turtlebot3_autonav'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ],
    scripts=['scripts/initial_pose_pub.py'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaronk',
    maintainer_email='aaronk@todo.todo',
    description='Autonomous TurtleBot3 simulation, mapping, and navigation launch package for ROS 2 Kilted.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_pub = scripts.initial_pose_pub:main',
        ],
    },
)
