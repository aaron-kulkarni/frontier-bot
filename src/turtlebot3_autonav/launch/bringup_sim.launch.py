#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import Node

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
    pkg_tb3_nav2 = get_package_share_directory('nav2_bringup')
    pkg_tb3_autonav = get_package_share_directory('turtlebot3_autonav')

    # Define spawn position
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')

    # Use default Nav2 params instead of custom ones to avoid YAML parsing issues
    nav2_params_path = os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')

    # Launch Gazebo with TurtleBot3 in empty world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )

    # Launch bringup_launch.py with slam enabled (handles both SLAM and Nav2, and launches RViz2)
    nav2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'True',
            'params_file': nav2_params_path,
            'map': '',
        }.items()
    )

    explorer_node = Node(
        package='turtlebot3_autonav',
        executable='explorer_node',
        name='explorer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
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
            default_value=os.path.join(pkg_tb3_gazebo, 'worlds', 'empty_world.world'),
            description='Gazebo world file'
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Initial x position of the robot'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial y position of the robot'
        ),
        DeclareLaunchArgument(
            'z_pose',
            default_value='0.0',
            description='Initial z position of the robot'
        ),
        DeclareLaunchArgument(
            'yaw_pose',
            default_value='0.0',
            description='Initial yaw orientation of the robot'
        ),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model),

        # Start Gazebo and robot first
        gazebo_launch,

        # Delay launching navigation, RViz2, and explorer_node to allow Gazebo and /clock to start
        TimerAction(
            period=8.0,
            actions=[
                GroupAction([
                    nav2_slam_launch,
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        output='screen',
                        arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
                        additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')}
                    ),
                    explorer_node
                ])
            ]
        ),

        # Commenting out initial pose publisher until we fix package installation issues
        # TimerAction(
        #     period=12.0,
        #     actions=[
        #         Node(
        #             package='turtlebot3_autonav_py',
        #             executable='initial_pose_pub',
        #             name='initial_pose_pub',
        #             output='screen',
        #             parameters=[
        #                 {'use_sim_time': use_sim_time},
        #                 {'pose_x': x_pose},
        #                 {'pose_y': y_pose},
        #                 {'pose_yaw': yaw_pose}
        #             ]
        #         )
        #     ]
        # ),
    ])
