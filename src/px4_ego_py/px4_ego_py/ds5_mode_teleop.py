#!/usr/bin/env python3
# 将DS5手柄输入转换为无人机的位姿命令，并发布模式切换指令。

import math
import rclpy
from px4_msgs.msg import VehicleLocalPosition
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class DS5Teleop(Node):
    def __init__(self):
        super().__init__('ds5_mode_teleop')

        self.mode_pub = self.create_publisher(String, '/mode_key', 10)
        self.pos_cmd_pub = self.create_publisher(PositionCommand, '/drone_0_planning/pos_cmd', 10)

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        self.current_position = None
        self.current_yaw = 0.0

    def vehicle_local_position_callback(self, msg):
        self.current_position = (msg.y, msg.x, -msg.z)
        # PX4 local position heading uses FMU yaw; convert it to the ROS yaw used here.
        self.current_yaw = math.atan2(math.cos(msg.heading), math.sin(msg.heading))

    def joy_callback(self, msg):
        
        if self.current_position is None:
            return

        if msg.buttons[2] == 1:
            self.publish_mode_command('t')
        if msg.buttons[1] == 1:
            self.publish_mode_command('o')
        if msg.buttons[0] == 1:
            self.publish_mode_command('l')

        current_x, current_y, current_z = self.current_position
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        target_vel_x = msg.axes[1] * cos_yaw - msg.axes[0] * sin_yaw
        target_vel_y = msg.axes[1] * sin_yaw + msg.axes[0] * cos_yaw
        target_yaw = self.current_yaw + msg.axes[2] * 0.35
        target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))

        pos_cmd = PositionCommand()
        pos_cmd.header.stamp = self.get_clock().now().to_msg()
        pos_cmd.header.frame_id = 'map'
        pos_cmd.position.x = current_x + target_vel_x * 0.50
        pos_cmd.position.y = current_y + target_vel_y * 0.50
        pos_cmd.position.z = current_z + msg.axes[3] * 0.50
        pos_cmd.velocity.x = target_vel_x
        pos_cmd.velocity.y = target_vel_y
        pos_cmd.velocity.z = msg.axes[3]
        pos_cmd.acceleration.x = 0.0
        pos_cmd.acceleration.y = 0.0
        pos_cmd.acceleration.z = 0.0
        pos_cmd.yaw = target_yaw
        pos_cmd.yaw_dot = msg.axes[2]
        self.pos_cmd_pub.publish(pos_cmd)

    def publish_mode_command(self, mode_key):
        msg = String()
        msg.data = mode_key
        self.mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DS5Teleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
