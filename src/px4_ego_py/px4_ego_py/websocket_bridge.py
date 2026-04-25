import base64
import json
import math
import threading

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import BatteryStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
from websockets.exceptions import ConnectionClosed
from websockets.sync.server import serve


HOST = '0.0.0.0'
PORT = 8766
MAX_POINT_COUNT = 2000


class WebSocketBridgeNode(Node):
    def __init__(self):
        super().__init__('websocket_bridge')

        self.websocket = None

        self.create_subscription(Odometry, '/ego/odom_world', self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/cloud_registered', self.pointcloud_callback, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/detect/image/compressed', self.detect_image_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/detect/targets', self.detect_result_callback, 10)
        self.create_subscription(BatteryStatus, '/fmu/out/battery_status_v1', self.battery_callback, qos_profile_sensor_data)

        self.ws_server = serve(self.handle_client, HOST, PORT, max_size=None)
        self.get_logger().info(f'WebSocket bridge ready at ws://{HOST}:{PORT}')
        self.ws_thread = threading.Thread(target=self.ws_server.serve_forever, daemon=True)
        self.ws_thread.start()

    def destroy_node(self):
        self.ws_server.shutdown()
        self.ws_thread.join(timeout=1.0)
        return super().destroy_node()

    def broadcast(self, message_type, data):
        message_text = json.dumps({
            'type': message_type,
            'data': data,
        }, ensure_ascii=False)

        if self.websocket is None:
            # self.get_logger().warning('No WebSocket client connected, skipping broadcast')
            return

        try:
            self.websocket.send(message_text)
        except ConnectionClosed:
            self.websocket = None

    def handle_client(self, websocket):
        self.websocket = websocket

        try:
            while True:
                websocket.recv()
        except ConnectionClosed:
            pass

    def odom_callback(self, msg):
        roll, pitch, yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ])
        telemetry = {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'frame_id': msg.header.frame_id,
            'position': {
                'x': float(msg.pose.pose.position.x),
                'y': float(msg.pose.pose.position.y),
                'z': float(msg.pose.pose.position.z),
            },
            'attitude': {
                'roll': math.degrees(roll),
                'pitch': math.degrees(pitch),
                'yaw': (math.degrees(yaw) + 360.0) % 360.0,
            },
            'linear_velocity': {
                'x': float(msg.twist.twist.linear.x),
                'y': float(msg.twist.twist.linear.y),
                'z': float(msg.twist.twist.linear.z),
            },
        }
        self.broadcast('telemetry', telemetry)

    def battery_callback(self, msg):
        battery = {
            'connected': bool(msg.connected),
            'voltage_v': float(msg.voltage_v),
            'battery_percent': None if msg.remaining < 0.0 else max(0.0, min(100.0, float(msg.remaining) * 100.0)),
            'warning': int(msg.warning),
        }
        self.broadcast('battery', battery)

    def pointcloud_callback(self, msg):
        total_points = int(msg.width) * int(msg.height)
        stride = max(total_points // MAX_POINT_COUNT, 1)
        points = []

        for index, point in enumerate(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)):
            if index % stride != 0:
                continue

            points.append([float(point[0]), float(point[1]), float(point[2])])
            if len(points) >= MAX_POINT_COUNT:
                break

        self.broadcast('pointcloud', {
            'points': points,
        })

    def detect_image_callback(self, msg):
        self.broadcast('frame', {
            'data': base64.b64encode(bytes(msg.data)).decode('ascii'),
        })

    def detect_result_callback(self, msg):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        self.broadcast('detections', detections)


def main():
    rclpy.init()
    node = WebSocketBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
