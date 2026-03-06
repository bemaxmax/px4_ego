#!/usr/bin/env python3

import math

import rclpy
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class DS5Teleop(Node):
    """Translate a DS5 joystick into mode commands and offboard setpoints."""

    def __init__(self):
        super().__init__('ds5_mode_teleop')

        self.joy_topic = self.declare_parameter('joy_topic', '/joy').value
        self.update_rate_hz = self.declare_float_param('update_rate_hz', 50.0, minimum=1.0)
        self.max_xy_speed = self.declare_float_param('max_xy_speed', 0.80, absolute=True)
        self.max_z_speed = self.declare_float_param('max_z_speed', 0.50, absolute=True)
        self.max_yaw_rate = self.declare_float_param('max_yaw_rate', 0.80, absolute=True)
        self.position_lookahead_sec = self.declare_float_param('position_lookahead_sec',0.50,minimum=0.0,)
        self.yaw_lookahead_sec = self.declare_float_param('yaw_lookahead_sec',0.35,minimum=0.0,)
        self.joy_timeout_sec = self.declare_float_param('joy_timeout_sec',0.50,minimum=0.0,)

        self.left_x_axis = self.declare_int_param('left_x_axis', 0)
        self.left_y_axis = self.declare_int_param('left_y_axis', 1)
        self.right_x_axis = self.declare_int_param('right_x_axis', 2)
        self.right_y_axis = self.declare_int_param('right_y_axis', 3)
        self.left_x_sign = self.declare_float_param('left_x_sign', 1.0)
        self.left_y_sign = self.declare_float_param('left_y_sign', 1.0)
        self.right_x_sign = self.declare_float_param('right_x_sign', 1.0)
        self.right_y_sign = self.declare_float_param('right_y_sign', 1.0)

        self.mode_buttons = (
            (self.declare_int_param('square_button', 2), 't', False),
            (self.declare_int_param('circle_button', 1), 'o', True),
            (self.declare_int_param('cross_button', 0), 'l', False),
            (self.declare_int_param('triangle_button', 3), 'p', False),
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.mode_pub = self.create_publisher(String, '/mode_key', 10)
        self.pos_cmd_pub = self.create_publisher(PositionCommand, '/drone_0_planning/pos_cmd', 10)

        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)
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

        self.previous_buttons = []
        self.current_position = None
        self.current_yaw = 0.0
        self.target_forward_vel = 0.0
        self.target_left_vel = 0.0
        self.target_vel_z = 0.0
        self.target_yaw_rate = 0.0
        self.last_active_input_time = None

        timer_period = 1.0 / self.update_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            'DS5 teleop ready. Default mapping assumes joy/game_controller_node for /joy '
            '(LX=0, LY=1, RX=2, RY=3). Planar translation is body-frame.'
        )

    def declare_float_param(self, name, default, minimum=None, absolute=False):
        value = float(self.declare_parameter(name, default).value)
        if absolute:
            value = abs(value)
        if minimum is not None:
            value = max(value, minimum)
        return value

    def declare_int_param(self, name, default):
        return int(self.declare_parameter(name, default).value)

    def quaternion_to_yaw(self, w, x, y, z):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def convert_yaw_between_fmu_and_ros(self, yaw_value):
        # offboard_control_test uses the same transform in the opposite direction.
        return math.atan2(math.cos(yaw_value), math.sin(yaw_value))

    def vehicle_local_position_callback(self, msg):
        self.set_pose(
            x=msg.y,
            y=msg.x,
            z=-msg.z,
            yaw=self.convert_yaw_between_fmu_and_ros(msg.heading),
        )

    def vehicle_visual_odom_callback(self, msg):
        yaw_ned = self.quaternion_to_yaw(*msg.q)
        self.set_pose(
            x=msg.position[1],
            y=msg.position[0],
            z=-msg.position[2],
            yaw=self.convert_yaw_between_fmu_and_ros(yaw_ned),
        )

    def set_pose(self, x, y, z, yaw):
        self.current_position = (float(x), float(y), float(z))
        self.current_yaw = float(yaw)

    def joy_callback(self, msg):
        if self.has_active_input(msg):
            self.last_active_input_time = self.get_clock().now()
        self.target_forward_vel = (
            self.left_y_sign * self.read_axis(msg.axes, self.left_y_axis) * self.max_xy_speed
        )
        self.target_left_vel = (
            self.left_x_sign * self.read_axis(msg.axes, self.left_x_axis) * self.max_xy_speed
        )
        self.target_vel_z = (
            self.right_y_sign * self.read_axis(msg.axes, self.right_y_axis) * self.max_z_speed
        )
        self.target_yaw_rate = (
            self.right_x_sign * self.read_axis(msg.axes, self.right_x_axis) * self.max_yaw_rate
        )

        buttons = list(msg.buttons)
        if len(self.previous_buttons) != len(buttons):
            self.previous_buttons = [0] * len(buttons)

        for button_index, mode_key, prime_setpoint in self.mode_buttons:
            if not self.is_button_rising(buttons, button_index):
                continue
            if prime_setpoint:
                self.publish_current_command()
            self.publish_mode_command(mode_key)

        self.previous_buttons = buttons

    def is_button_rising(self, buttons, button_index):
        if button_index < 0 or button_index >= len(buttons):
            return False

        current_pressed = buttons[button_index] == 1
        previous_pressed = self.previous_buttons[button_index] == 1
        return current_pressed and not previous_pressed

    def timer_callback(self):
        if self.current_position is None or not self.has_recent_active_input():
            return

        self.publish_current_command()

    def read_axis(self, axes, axis_index):
        if axis_index < 0 or axis_index >= len(axes):
            return 0.0

        return float(axes[axis_index])

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def has_active_input(self, msg):
        if any(button != 0 for button in msg.buttons):
            return True
        return any(axis != 0.0 for axis in msg.axes)

    def has_recent_active_input(self):
        if self.last_active_input_time is None:
            return False
        if self.joy_timeout_sec <= 0.0:
            return True

        age_sec = (
            self.get_clock().now().nanoseconds - self.last_active_input_time.nanoseconds
        ) / 1e9
        return age_sec <= self.joy_timeout_sec

    def publish_mode_command(self, mode_key):
        msg = String()
        msg.data = mode_key
        self.mode_pub.publish(msg)
        self.get_logger().info(f'Published mode command: {mode_key}')

    def publish_current_command(self):
        if self.current_position is None:
            return

        current_x, current_y, current_z = self.current_position
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        target_vel_x = self.target_forward_vel * cos_yaw - self.target_left_vel * sin_yaw
        target_vel_y = self.target_forward_vel * sin_yaw + self.target_left_vel * cos_yaw
        target_yaw = self.wrap_angle(
            self.current_yaw + self.target_yaw_rate * self.yaw_lookahead_sec
        )

        msg = PositionCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.position.x = current_x + target_vel_x * self.position_lookahead_sec
        msg.position.y = current_y + target_vel_y * self.position_lookahead_sec
        msg.position.z = current_z + self.target_vel_z * self.position_lookahead_sec
        msg.velocity.x = target_vel_x
        msg.velocity.y = target_vel_y
        msg.velocity.z = self.target_vel_z
        msg.acceleration.x = 0.0
        msg.acceleration.y = 0.0
        msg.acceleration.z = 0.0
        msg.yaw = target_yaw
        msg.yaw_dot = self.target_yaw_rate
        self.pos_cmd_pub.publish(msg)


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
