from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_name = LaunchConfiguration("model_name")
    config_file = PathJoinSubstitution([
        FindPackageShare("llm_ros2_agent"),
        "config",
        "voice_command_min.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_name",
            default_value="MiniMax-M2.7",
        ),
        Node(
            package="llm_ros2_agent",
            executable="voice_command",
            name="voice_command",
            parameters=[
                config_file,
                {"model_name": model_name},
            ],
            output="screen",
        ),
        Node(
            package="llm_ros2_agent",
            executable="feishu_bridge",
            name="feishu_bridge",
            output="screen",
        ),
        Node(
            package="llm_ros2_agent",
            executable="n8n_result_bridge",
            name="n8n_result_bridge",
            output="screen",
        ),
        Node(
            package="llm_ros2_agent",
            executable="report_generator",
            name="report_generator",
            output="screen",
        ),
    ])
