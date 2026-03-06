#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('key_publisher')
        self.pub = self.create_publisher(String, '/mode_key', 10)
        self.get_logger().info("键盘指令发布节点启动，等待输入字母...")

    def run(self):
        while rclpy.ok():
            key = input("请输入字母并回车 >>> ").strip()

            if not key:
                continue

            msg = String()
            msg.data = key
            self.pub.publish(msg)
            self.get_logger().info(f"已发布字母：{key}")


def run_keyboard_mode():
    rclpy.init()
    node = KeyboardPublisher()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


def main():
    run_keyboard_mode()


if __name__ == "__main__":
    main()
