#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, ThisLaunchFileDir
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set TURTLEBOT3_MODEL environment variable
    turtlebot3_model = LaunchConfiguration('turtlebot3_model')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds', 'empty.world'
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

    # Launch SLAM Toolbox via navigation2.launch.py with slam enabled
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_nav2, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'True',
        }.items()
    )

    # Launch RViz2 is handled by navigation2.launch.py, so do not launch explicitly here

    # Launch Nav2 bringup (after SLAM is running)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_nav2, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

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
            default_value=os.path.join(pkg_tb3_gazebo, 'worlds', 'empty.world'),
            description='Gazebo world file'
        ),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model),

        # Start Gazebo and robot
        gazebo_launch,

        # Start SLAM Toolbox
        slam_launch,

        # Start explorer_node (autonomous exploration)
        Node(
            package='turtlebot3_autonav',
            executable='explorer_node',
            name='explorer_node',
            output='screen'
        ),

        # Start RViz2 (handled by navigation2.launch.py)

        # Start Nav2 after a short delay to allow SLAM to initialize
        TimerAction(
            period=7.0,
            actions=[nav2_launch]
        ),
    ])
