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

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('max_xy_speed', 0.80)
        self.declare_parameter('max_z_speed', 0.50)
        self.declare_parameter('max_yaw_rate', 0.80)
        self.declare_parameter('position_lookahead_sec', 0.50)
        self.declare_parameter('yaw_lookahead_sec', 0.35)
        self.declare_parameter('joy_timeout_sec', 0.50)

        # The default indexes match joy/game_controller_node.
        self.declare_parameter('left_x_axis', 0)
        self.declare_parameter('left_y_axis', 1)
        self.declare_parameter('right_x_axis', 2)
        self.declare_parameter('right_y_axis', 3)
        self.declare_parameter('left_x_sign', 1.0)
        self.declare_parameter('left_y_sign', 1.0)
        self.declare_parameter('right_x_sign', 1.0)
        self.declare_parameter('right_y_sign', 1.0)
        self.declare_parameter('square_button', 2)
        self.declare_parameter('circle_button', 1)
        self.declare_parameter('cross_button', 0)
        self.declare_parameter('triangle_button', 3)

        self.joy_topic = self.get_parameter('joy_topic').value
        self.update_rate_hz = max(float(self.get_parameter('update_rate_hz').value), 1.0)
        self.deadzone = abs(float(self.get_parameter('deadzone').value))
        self.max_xy_speed = abs(float(self.get_parameter('max_xy_speed').value))
        self.max_z_speed = abs(float(self.get_parameter('max_z_speed').value))
        self.max_yaw_rate = abs(float(self.get_parameter('max_yaw_rate').value))
        self.position_lookahead_sec = max(
            float(self.get_parameter('position_lookahead_sec').value),
            0.0,
        )
        self.yaw_lookahead_sec = max(
            float(self.get_parameter('yaw_lookahead_sec').value),
            0.0,
        )
        self.joy_timeout_sec = max(
            float(self.get_parameter('joy_timeout_sec').value),
            0.0,
        )

        self.left_x_axis = int(self.get_parameter('left_x_axis').value)
        self.left_y_axis = int(self.get_parameter('left_y_axis').value)
        self.right_x_axis = int(self.get_parameter('right_x_axis').value)
        self.right_y_axis = int(self.get_parameter('right_y_axis').value)
        self.left_x_sign = float(self.get_parameter('left_x_sign').value)
        self.left_y_sign = float(self.get_parameter('left_y_sign').value)
        self.right_x_sign = float(self.get_parameter('right_x_sign').value)
        self.right_y_sign = float(self.get_parameter('right_y_sign').value)
        self.square_button = int(self.get_parameter('square_button').value)
        self.circle_button = int(self.get_parameter('circle_button').value)
        self.cross_button = int(self.get_parameter('cross_button').value)
        self.triangle_button = int(self.get_parameter('triangle_button').value)

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
        self.target_vel_x = 0.0
        self.target_vel_y = 0.0
        self.target_vel_z = 0.0
        self.target_yaw_rate = 0.0
        self.last_joy_msg_time = None

        timer_period = 1.0 / self.update_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            'DS5 teleop ready. Default mapping assumes joy/game_controller_node for /joy '
            '(LX=0, LY=1, RX=2, RY=3).'
        )

    def quaternion_to_yaw(self, w, x, y, z):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def convert_yaw_between_fmu_and_ros(self, yaw_value):
        # offboard_control_test uses the same transform in the opposite direction.
        return math.atan2(math.cos(yaw_value), math.sin(yaw_value))

    def vehicle_local_position_callback(self, msg):
        self.update_current_pose(
            ros_x=msg.y,
            ros_y=msg.x,
            ros_z=-msg.z,
            ros_yaw=self.convert_yaw_between_fmu_and_ros(msg.heading),
        )

    def vehicle_visual_odom_callback(self, msg):
        q_att = msg.q
        yaw_ned = self.quaternion_to_yaw(q_att[0], q_att[1], q_att[2], q_att[3])
        self.update_current_pose(
            ros_x=msg.position[1],
            ros_y=msg.position[0],
            ros_z=-msg.position[2],
            ros_yaw=self.convert_yaw_between_fmu_and_ros(yaw_ned),
        )

    def update_current_pose(self, ros_x, ros_y, ros_z, ros_yaw):
        self.current_position = [float(ros_x), float(ros_y), float(ros_z)]
        self.current_yaw = float(ros_yaw)

    def joy_callback(self, msg):
        self.last_joy_msg_time = self.get_clock().now()
        self.target_vel_x = (
            -self.left_x_sign * self.read_axis(msg.axes, self.left_x_axis) * self.max_xy_speed
        )
        self.target_vel_y = (
            self.left_y_sign * self.read_axis(msg.axes, self.left_y_axis) * self.max_xy_speed
        )
        self.target_vel_z = (
            self.right_y_sign * self.read_axis(msg.axes, self.right_y_axis) * self.max_z_speed
        )
        self.target_yaw_rate = (
            self.right_x_sign * self.read_axis(msg.axes, self.right_x_axis) * self.max_yaw_rate
        )

        buttons = list(msg.buttons)

        if not self.previous_buttons or len(self.previous_buttons) != len(buttons):
            self.previous_buttons = [0] * len(buttons)

        self.handle_button_press(buttons, self.square_button, 't', prime_setpoint=False)
        self.handle_button_press(buttons, self.circle_button, 'o', prime_setpoint=True)
        self.handle_button_press(buttons, self.cross_button, 'l', prime_setpoint=False)
        self.handle_button_press(buttons, self.triangle_button, 'p', prime_setpoint=False)

        self.previous_buttons = buttons

    def handle_button_press(self, buttons, button_index, mode_key, prime_setpoint):
        if not self.is_button_rising(buttons, button_index):
            return

        if prime_setpoint:
            self.publish_current_command()

        self.publish_mode_command(mode_key)

    def is_button_rising(self, buttons, button_index):
        if button_index < 0 or button_index >= len(buttons):
            return False

        current_pressed = buttons[button_index] == 1
        previous_pressed = self.previous_buttons[button_index] == 1
        return current_pressed and not previous_pressed

    def timer_callback(self):
        if self.current_position is None:
            return
        if not self.has_recent_joy_message():
            return

        self.publish_current_command()

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def read_axis(self, axes, axis_index):
        if axis_index < 0 or axis_index >= len(axes):
            return 0.0

        value = float(axes[axis_index])
        return self.apply_deadzone(value)

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def has_recent_joy_message(self):
        if self.last_joy_msg_time is None:
            return False
        if self.joy_timeout_sec <= 0.0:
            return True

        age_sec = (
            self.get_clock().now().nanoseconds - self.last_joy_msg_time.nanoseconds
        ) / 1e9
        return age_sec <= self.joy_timeout_sec

    def build_target_from_current_pose(self):
        if self.current_position is None:
            return None, None

        target_position = [
            self.current_position[0] + self.target_vel_x * self.position_lookahead_sec,
            self.current_position[1] + self.target_vel_y * self.position_lookahead_sec,
            self.current_position[2] + self.target_vel_z * self.position_lookahead_sec,
        ]
        target_yaw = self.wrap_angle(
            self.current_yaw + self.target_yaw_rate * self.yaw_lookahead_sec
        )
        return target_position, target_yaw

    def publish_mode_command(self, mode_key):
        msg = String()
        msg.data = mode_key
        self.mode_pub.publish(msg)
        self.get_logger().info(f'Published mode command: {mode_key}')

    def publish_current_command(self):
        target_position, target_yaw = self.build_target_from_current_pose()
        if target_position is None:
            return

        msg = PositionCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.position.x = target_position[0]
        msg.position.y = target_position[1]
        msg.position.z = target_position[2]
        msg.velocity.x = self.target_vel_x
        msg.velocity.y = self.target_vel_y
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
