#!/usr/bin/env python3
import subprocess
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from llm_ros2_agent.model_parser import create_model_client
from llm_ros2_agent.model_parser import parse_command


class VoiceCommand(Node):
    def __init__(self):
        super().__init__("voice_command")
        self.declare_parameter("model_name", Parameter.Type.STRING)
        self.declare_parameter("system_prompt", Parameter.Type.STRING)
        self.model_name = self.get_parameter("model_name").value
        self.system_prompt = self.get_parameter("system_prompt").value
        self.client = create_model_client(self.model_name)
        self.bag_play_process = None

        self.voice_text_sub = self.create_subscription(String, "/voice_text", self.voice_text_callback, 10)
        self.mode_pub = self.create_publisher(String, "/mode_key", 10)
        self.reply_pub = self.create_publisher(String, "/model_reply", 10)
        self.mission_finished_pub = self.create_publisher(String, "/mission_finished", 10)

    def publish_mode_key(self, value: str):
        msg = String()
        msg.data = value
        self.mode_pub.publish(msg)
        self.get_logger().info(f"已发布 /mode_key: {value}")

    def voice_text_callback(self, msg):
        text = msg.data.strip()

        try:
            cmd = parse_command(
                text,
                self.client,
                self.model_name,
                self.system_prompt,
            )
        except Exception as exc:
            self.get_logger().info(f"模型解析失败: {exc}")
            reply_msg = String()
            reply_msg.data = "模型暂时繁忙，请稍后再试。"
            self.reply_pub.publish(reply_msg)
            return

        self.get_logger().info(f"模型解析结果: {cmd}")
        self.execute_command(cmd)

    def execute_command(self, cmd: dict):
        if cmd.get("type") == "tool":
            if cmd.get("tool_name") == "publish_mode_key":
                mode_key = cmd.get("mode_key")
                self.publish_mode_key(mode_key)
                reply = {
                    "t": "已为您执行起飞命令。",
                    "o": "已经切换到外部控制模式。",
                    "l": "已为您执行降落命令。",
                }[mode_key]
            elif cmd.get("tool_name") == "play_rosbag":
                reply = "正在进行自主探索。"
                bag_dir = "/home/ubuntu22/px4_ego/rosbag2_2026_04_13-05_02_23"
                self.bag_play_process = subprocess.Popen(["ros2", "bag", "play", bag_dir])
                Thread(target=self.wait_rosbag_finished, daemon=True).start()
        else:
            reply = cmd.get("reply")

        reply_msg = String()
        reply_msg.data = reply
        self.reply_pub.publish(reply_msg)

    # rosbag 播放结束后发布任务完成信息，供报告生成等后续流程触发。
    def wait_rosbag_finished(self):
        return_code = self.bag_play_process.wait()
        if return_code != 0:
            return

        finished_msg = String()
        finished_msg.data = "rosbag_finished"
        self.mission_finished_pub.publish(finished_msg)

        reply_msg = String()
        reply_msg.data = "自主探索已完成，正在准备生成火灾辅助决策报告。"
        self.reply_pub.publish(reply_msg)


def main():
    rclpy.init()
    node = VoiceCommand()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.bag_play_process and node.bag_play_process.poll() is None:
            node.bag_play_process.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
