import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取与拼接默认路径
    px4_ego_dir = get_package_share_directory(
        'px4_ego_py')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    urdf_path = os.path.join(px4_ego_dir, 'urdf', 'px4_frame_links.urdf')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()
    
    default_map_yaml_path = os.path.join(px4_ego_dir, 'map', 'room.yaml')
    default_nav2_param_path = os.path.join(px4_ego_dir, 'config', 'nav2_params.yaml')

    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    map_yaml_path = launch.substitutions.LaunchConfiguration('map')
    nav2_param_path = launch.substitutions.LaunchConfiguration('params_file')
    start_cmd_vel_bridge = launch.substitutions.LaunchConfiguration('start_cmd_vel_bridge')

    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value='true',
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=default_map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=default_nav2_param_path,
                                             description='Full path to param file to load'),
        launch.actions.DeclareLaunchArgument(
            'start_cmd_vel_bridge',
            default_value='true',
            description='Start cmd_vel to PositionCommand bridge node if true',
        ),

        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
            output='screen'),
        launch_ros.actions.Node(
            package='px4_ego_py',
            executable='cmd_vel_to_pos_cmd',
            name='cmd_vel_to_pos_cmd',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=launch.conditions.IfCondition(start_cmd_vel_bridge),
            output='screen'),

        launch_ros.actions.Node(
            package='px4_odom_tf',
            executable='px4_odom_tf_publisher',
            name='px4_odom_tf_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
            
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
