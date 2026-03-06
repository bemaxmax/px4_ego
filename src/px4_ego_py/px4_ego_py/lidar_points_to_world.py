#!/usr/bin/env python3

import math
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class LidarPointsToWorld(Node):
    """Transform lidar point clouds into the world frame expected by ego_planner."""

    def __init__(self) -> None:
        super().__init__('lidar_points_to_world')

        self.declare_parameter('input_cloud_topic', '/lidar_points')
        self.declare_parameter('output_cloud_topic', '/lidar_points_world')
        self.declare_parameter('odom_topic', '/mavros/local_position/odom')
        self.declare_parameter('output_frame_id', 'world')
        self.declare_parameter('lidar_offset_x', 0.12)
        self.declare_parameter('lidar_offset_y', 0.0)
        self.declare_parameter('lidar_offset_z', 0.315)
        self.declare_parameter('lidar_roll', 0.0)
        self.declare_parameter('lidar_pitch', 0.0)
        self.declare_parameter('lidar_yaw', 0.0)

        input_cloud_topic = str(self.get_parameter('input_cloud_topic').value)
        output_cloud_topic = str(self.get_parameter('output_cloud_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        self.output_frame_id = str(self.get_parameter('output_frame_id').value)

        self.lidar_translation = np.array(
            [
                float(self.get_parameter('lidar_offset_x').value),
                float(self.get_parameter('lidar_offset_y').value),
                float(self.get_parameter('lidar_offset_z').value),
            ],
            dtype=np.float32,
        )
        self.lidar_rotation = self.rotation_matrix_from_rpy(
            float(self.get_parameter('lidar_roll').value),
            float(self.get_parameter('lidar_pitch').value),
            float(self.get_parameter('lidar_yaw').value),
        )

        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            input_cloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            odom_qos,
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            output_cloud_topic,
            qos_profile_sensor_data,
        )

        self.latest_position: Optional[np.ndarray] = None
        self.latest_rotation: Optional[np.ndarray] = None
        self.waiting_for_odom_logged = False

        self.get_logger().info(
            f'Point cloud bridge ready: {input_cloud_topic} -> {output_cloud_topic}, '
            f'odom={odom_topic}, frame={self.output_frame_id}'
        )

    @staticmethod
    def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float32,
        )

    @staticmethod
    def rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
        sr = math.sin(roll)
        cr = math.cos(roll)
        sp = math.sin(pitch)
        cp = math.cos(pitch)
        sy = math.sin(yaw)
        cy = math.cos(yaw)

        return np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=np.float32,
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=np.float32,
        )
        q = msg.pose.pose.orientation
        self.latest_rotation = self.quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)

    def cloud_callback(self, msg: PointCloud2) -> None:
        if self.latest_position is None or self.latest_rotation is None:
            if not self.waiting_for_odom_logged:
                self.waiting_for_odom_logged = True
                self.get_logger().warn('Waiting for odometry before transforming lidar points')
            return

        if self.waiting_for_odom_logged:
            self.waiting_for_odom_logged = False
            self.get_logger().info('Received odometry, lidar point cloud transformation is active')

        points = point_cloud2.read_points_numpy(
            msg,
            field_names=['x', 'y', 'z'],
            skip_nans=True,
        )
        if points.size == 0:
            return

        local_points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        points_in_base = (self.lidar_rotation @ local_points.T).T + self.lidar_translation
        points_in_world = (self.latest_rotation @ points_in_base.T).T + self.latest_position

        output_msg = point_cloud2.create_cloud_xyz32(msg.header, points_in_world.tolist())
        output_msg.header.stamp = msg.header.stamp
        output_msg.header.frame_id = self.output_frame_id
        self.cloud_pub.publish(output_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarPointsToWorld()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
