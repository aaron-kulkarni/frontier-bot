#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import sys

from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
    # Set TURTLEBOT3_MODEL environment variable
    turtlebot3_model = LaunchConfiguration('turtlebot3_model')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds', 'empty_world.world'
    ))

    # Paths
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_tb3_nav2 = get_package_share_directory('turtlebot3_navigation2')

    # Launch Gazebo with TurtleBot3 in empty world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Launch navigation2.launch.py with slam enabled (handles both SLAM and Nav2, and launches RViz2)
    nav2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_nav2, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'True',
        }.items()
    )

    explorer_node = Node(
        package='turtlebot3_autonav',
        executable='explorer_node',
        name='explorer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Node to publish initial pose inside map bounds at startup (Python node workaround)
    # initial_pose_publisher = Node(
    #     package='turtlebot3_initial_pose_pub',
    #     executable='initial_pose_pub',
    #     name='initial_pose_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     prefix=sys.executable + ' '
    # )

    # Group actions for clarity
    return LaunchDescription([
        DeclareLaunchArgument(
            'turtlebot3_model',
            default_value='burger',
            description='TurtleBot3 model type [burger, waffle, waffle_pi]'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_tb3_gazebo, 'worlds', 'empty_world.world'),
            description='Gazebo world file'
        ),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model),

        # Start Gazebo and robot first
        gazebo_launch,

        # Publish initial pose after Gazebo starts, before navigation stack
        # TimerAction(
        #     period=6.0,
        #     actions=[initial_pose_publisher]
        # ),

        # Delay launching navigation and explorer_node to allow Gazebo and /clock to start
        TimerAction(
            period=7.0,
            actions=[
                GroupAction([
                    nav2_slam_launch,
                    explorer_node
                ])
            ]
        ),
    ])
