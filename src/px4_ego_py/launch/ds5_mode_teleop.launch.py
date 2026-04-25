#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node',
            output='screen',
            parameters=[{
                'device_id': 0,
                'autorepeat_rate': 50.0,
            }],
        ),
        Node(
            package='px4_ego_py',
            executable='ds5_mode_teleop',
            name='ds5_mode_teleop',
            output='screen',
        ),
    ])
