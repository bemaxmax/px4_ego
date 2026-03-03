#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


REPO_ROOT = Path(__file__).resolve().parent
PACKAGE_ROOT = REPO_ROOT / 'src' / 'px4_ego_py'
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


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


def run_ds5_mode():
    from px4_ego_py.ds5_mode_teleop import main as ds5_main

    ds5_main()


def parse_args():
    parser = argparse.ArgumentParser(
        description='默认启动 DS5 手柄模式；如需旧键盘模式，请加 --keyboard。'
    )
    parser.add_argument(
        '--keyboard',
        action='store_true',
        help='使用原来的键盘输入模式，而不是 DS5 手柄模式',
    )
    return parser.parse_args()


def main():
    args = parse_args()

    if args.keyboard:
        run_keyboard_mode()
        return

    run_ds5_mode()


if __name__ == "__main__":
    main()
