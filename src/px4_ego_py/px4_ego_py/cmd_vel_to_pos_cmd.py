#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


class CmdVelToPosCmdBridge(Node):
    """Bridge Nav2 cmd_vel to quadrotor PositionCommand for offboard_control_test."""

    def __init__(self):
        super().__init__('cmd_vel_to_pos_cmd')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('pos_cmd_topic', '/drone_0_planning/pos_cmd')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('position_lookahead_sec', 0.50)
        self.declare_parameter('yaw_lookahead_sec', 0.35)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.pos_cmd_topic = str(self.get_parameter('pos_cmd_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.update_rate_hz = max(float(self.get_parameter('update_rate_hz').value), 1.0)
        self.cmd_vel_timeout_sec = max(float(self.get_parameter('cmd_vel_timeout_sec').value), 0.0)
        self.position_lookahead_sec = max(
            float(self.get_parameter('position_lookahead_sec').value),
            0.0,
        )
        self.yaw_lookahead_sec = max(float(self.get_parameter('yaw_lookahead_sec').value), 0.0)

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cmd_vel_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback,
            qos_profile_sub,
        )
        self.vehicle_visual_odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            self.vehicle_visual_odom_callback,
            qos_profile_sub,
        )

        self.pos_cmd_pub = self.create_publisher(PositionCommand, self.pos_cmd_topic, 10)

        self.current_position = None
        self.current_yaw = 0.0
        self.latest_cmd_linear_x = 0.0
        self.latest_cmd_linear_y = 0.0
        self.latest_cmd_linear_z = 0.0
        self.latest_cmd_yaw_rate = 0.0
        self.last_cmd_vel_time = None
        self.pose_source = 'none'
        self.pose_wait_log_sent = False

        timer_period = 1.0 / self.update_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'cmd_vel bridge ready: {self.cmd_vel_topic} -> {self.pos_cmd_topic}'
        )

    def quaternion_to_yaw(self, w, x, y, z):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def convert_yaw_between_fmu_and_ros(self, yaw_value):
        return math.atan2(math.cos(yaw_value), math.sin(yaw_value))

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def update_current_pose(self, ros_x, ros_y, ros_z, ros_yaw, source):
        self.current_position = [float(ros_x), float(ros_y), float(ros_z)]
        self.current_yaw = float(ros_yaw)
        if self.pose_source != source:
            self.pose_source = source
            self.get_logger().info(f'Pose source switched to {source}')

    def vehicle_local_position_callback(self, msg):
        self.update_current_pose(
            ros_x=msg.y,
            ros_y=msg.x,
            ros_z=-msg.z,
            ros_yaw=self.convert_yaw_between_fmu_and_ros(msg.heading),
            source='vehicle_local_position',
        )

    def vehicle_visual_odom_callback(self, msg):
        q_att = msg.q
        yaw_ned = self.quaternion_to_yaw(q_att[0], q_att[1], q_att[2], q_att[3])
        self.update_current_pose(
            ros_x=msg.position[1],
            ros_y=msg.position[0],
            ros_z=-msg.position[2],
            ros_yaw=self.convert_yaw_between_fmu_and_ros(yaw_ned),
            source='vehicle_visual_odometry',
        )

    def cmd_vel_callback(self, msg):
        self.latest_cmd_linear_x = float(msg.linear.x)
        self.latest_cmd_linear_y = float(msg.linear.y)
        self.latest_cmd_linear_z = float(msg.linear.z)
        self.latest_cmd_yaw_rate = float(msg.angular.z)
        self.last_cmd_vel_time = self.get_clock().now()

    def has_recent_cmd_vel(self):
        if self.last_cmd_vel_time is None:
            return False
        if self.cmd_vel_timeout_sec <= 0.0:
            return True

        age_sec = (self.get_clock().now().nanoseconds - self.last_cmd_vel_time.nanoseconds) / 1e9
        return age_sec <= self.cmd_vel_timeout_sec

    def body_to_world_planar_velocity(self, forward_velocity, left_velocity):
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        world_x = forward_velocity * cos_yaw - left_velocity * sin_yaw
        world_y = forward_velocity * sin_yaw + left_velocity * cos_yaw
        return world_x, world_y

    def publish_pos_cmd(self, target_position, target_velocity, target_yaw, target_yaw_rate):
        msg = PositionCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.position.x = float(target_position[0])
        msg.position.y = float(target_position[1])
        msg.position.z = float(target_position[2])
        msg.velocity.x = float(target_velocity[0])
        msg.velocity.y = float(target_velocity[1])
        msg.velocity.z = float(target_velocity[2])
        msg.acceleration.x = 0.0
        msg.acceleration.y = 0.0
        msg.acceleration.z = 0.0
        msg.yaw = float(target_yaw)
        msg.yaw_dot = float(target_yaw_rate)
        self.pos_cmd_pub.publish(msg)

    def timer_callback(self):
        if self.current_position is None:
            if not self.pose_wait_log_sent:
                self.pose_wait_log_sent = True
                self.get_logger().warn('Waiting for PX4 pose before forwarding cmd_vel')
            return

        if self.pose_wait_log_sent:
            self.pose_wait_log_sent = False
            self.get_logger().info('Received PX4 pose, cmd_vel bridge is active')

        has_recent_cmd = self.has_recent_cmd_vel()
        if not has_recent_cmd:
            return

        target_vel_x, target_vel_y = self.body_to_world_planar_velocity(
            self.latest_cmd_linear_x,
            self.latest_cmd_linear_y,
        )

        target_position = [
            self.current_position[0] + target_vel_x * self.position_lookahead_sec,
            self.current_position[1] + target_vel_y * self.position_lookahead_sec,
            self.current_position[2] + self.latest_cmd_linear_z * self.position_lookahead_sec,
        ]
        target_yaw = self.wrap_angle(self.current_yaw + self.latest_cmd_yaw_rate * self.yaw_lookahead_sec)
        target_velocity = [target_vel_x, target_vel_y, self.latest_cmd_linear_z]
        self.publish_pos_cmd(
            target_position=target_position,
            target_velocity=target_velocity,
            target_yaw=target_yaw,
            target_yaw_rate=self.latest_cmd_yaw_rate,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPosCmdBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
