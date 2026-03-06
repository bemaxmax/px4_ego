#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joy_topic = LaunchConfiguration('joy_topic')
    device_id = LaunchConfiguration('device_id')
    autorepeat_rate = LaunchConfiguration('autorepeat_rate')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_topic',
            default_value='/joy',
            description='Topic published by joy/game_controller_node',
        ),
        DeclareLaunchArgument(
            'device_id',
            default_value='0',
            description='Joystick device index used by game_controller_node',
        ),
        DeclareLaunchArgument(
            'autorepeat_rate',
            default_value='50.0',
            description='Joy message repeat rate (Hz) when stick is held',
        ),
        Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node',
            output='screen',
            parameters=[{
                'device_id': ParameterValue(device_id, value_type=int),
                'autorepeat_rate': ParameterValue(autorepeat_rate, value_type=float),
            }],
            remappings=[('/joy', joy_topic)],
        ),
        Node(
            package='px4_ego_py',
            executable='ds5_mode_teleop',
            name='ds5_mode_teleop',
            output='screen',
            parameters=[{
                'joy_topic': ParameterValue(joy_topic, value_type=str),
            }],
        ),
    ])
