from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    px4_ego_share = get_package_share_directory('px4_ego_py')
    gazebo_model_root = os.path.expanduser('~/.simulation-gazebo/models')

    micro_xrce_start_cmd = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        shell=False,
        output='screen',
    )

    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    px4_bin = os.path.join(px4_dir, 'build/px4_sitl_default/bin/px4')
    px4_instance0 = ExecuteProcess(
        cmd=[px4_bin, '-i', '0'],
        additional_env={
            'PX4_GZ_STANDALONE': '1',
            'PX4_SYS_AUTOSTART': '4001',
            # 当前默认从围栏外起飞。
            # 原始出生点（围栏外）: 9.6637802124023438,-0.086192898452281952,0.18868870969861746,0,0,3.141592653589793
            'PX4_GZ_MODEL_POSE': '9.6637802124023438,-0.086192898452281952,0.18868870969861746,0,0,3.141592653589793',
            # 'PX4_GZ_MODEL_POSE': '0.9771640000000000,0.1227600000000000,2.6886887096986175,0,0,3.141592653589793',
            'PX4_SIM_MODEL': 'x500_lidar_depth_3d',
        },
        output='screen',
    )

    # 配置gazebo环境变量并启动gazebo仿真
    # 仿真世界和模型资源由simulation-gazebo脚本从~/.simulation-gazebo目录加载
    gazebo_simulation_cmd = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(px4_ego_share, 'scripts', 'simulation-gazebo'),
        ],
        output='screen',
    )

    best_effort_depth_img_pub = Node(
        package='px4_bridge_cpp',
        executable='depth_gz_bridge',
        name='depth_img_transfer',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    lidar_points_to_world_node = Node(
        package='px4_bridge_cpp',
        executable='lidar_points_to_world',
        name='lidar_points_to_world',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    urdf_path = os.path.join(px4_ego_share, 'urdf', 'px4_frame_links.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    bridge_config = os.path.join(
        px4_ego_share,
        'config',
        'ros_gz_bridge_lidar_depth_3d.yaml',
    )

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    gazebo_truth_to_visual_odometry_node = Node(
        package='px4_bridge_cpp',
        executable='gazebo_truth_to_visual_odometry',
        name='gazebo_truth_to_visual_odometry',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    offboard_control_node = Node(
        package='px4_ego_py',
        executable='offboard_control_test',
        name='offboard_control',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    websocket_bridge_node = Node(
        package='px4_ego_py',
        executable='websocket_bridge',
        name='websocket_bridge',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    target_detector_node = Node(
        package='detect',
        executable='target_detector',
        name='target_detector',
        output='screen',
    )

    return LaunchDescription([
        gazebo_simulation_cmd,#启动gazebo仿真
        px4_instance0,#激活无人机并设置位置
        ros_gz_bridge_node,#gz->ros桥接节点
        micro_xrce_start_cmd,#启动MicroXRCEAgent
        offboard_control_node,#PX4 offboard控制节点
        best_effort_depth_img_pub,#发布深度图桥接节点
        gazebo_truth_to_visual_odometry_node,#输入到视觉里程计的真值位姿转换节点
        websocket_bridge_node,#网页桥接节点
        target_detector_node,#目标检测节点
        # robot_state_publisher_node,#发布无人机状态
        # lidar_points_to_world_node,#将激光雷达点云转换到世界坐标系的节点，用于ego视觉里程计
    ])
