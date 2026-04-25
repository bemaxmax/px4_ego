#!/usr/bin/env python3
import json
import re
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from pathlib import Path
from queue import Empty
from queue import Queue
from threading import Thread

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TARGET_IMAGE_PATTERN = re.compile(
    r"^(?P<target_id>[^_]+)_(?P<category>.+)_x_(?P<x>-?\d+(?:\.\d+)?)_y_(?P<y>-?\d+(?:\.\d+)?)_conf_(?P<confidence>\d+(?:\.\d+)?)\.[^.]+$"
)


class N8nResultBridge(Node):
    def __init__(self):
        super().__init__("n8n_result_bridge")
        self.pending_results = Queue()
        self.reply_pub = self.create_publisher(String, "/model_reply", 10)
        self.recognized_target_pub = self.create_publisher(String, "/recognized_targets", 10)
        self.image_dir = Path("/home/ubuntu22/px4_ego/saved_targets")
        self.create_timer(0.1, self.publish_pending_results)

        # 接收 n8n 回调结果，再转成飞书桥已经能识别的图片路径和文本消息。
        self.http_server = ThreadingHTTPServer(("0.0.0.0", 8000), self.create_handler())
        self.http_thread = Thread(target=self.http_server.serve_forever, daemon=True)
        self.http_thread.start()

    def create_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                content_length = int(self.headers.get("Content-Length", "0"))
                raw_body = self.rfile.read(content_length)
                payload = json.loads(raw_body.decode("utf-8"))

                node.pending_results.put(payload)
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(b'{"ok": true}')

            def log_message(self, format, *args):
                return

        return Handler

    def publish_pending_results(self):
        while True:
            try:
                payload = self.pending_results.get_nowait()
            except Empty:
                break

            target_id = str(payload["target_id"]).strip()
            
            # 发布图片路径。
            image_path = None
            for image_path in self.image_dir.glob(f"{target_id}_*.jpg"):
                image_msg = String()
                image_msg.data = str(image_path)
                self.reply_pub.publish(image_msg)
                break

            target_category = payload["category"]
            target_position = ""
            if image_path is not None:
                match = TARGET_IMAGE_PATTERN.match(image_path.name)
                if match:
                    target_position = f"({float(match.group('x')):.2f}, {float(match.group('y')):.2f})"
            
            # 发布目标 ID 供报告生成。
            target_msg = String()
            target_msg.data = json.dumps({
                "target_id": target_id,
            }, ensure_ascii=False)
            self.recognized_target_pub.publish(target_msg)

            # 发布文本消息。
            text_msg = String()
            text_msg.data = (
                "🚨 发现新目标\n\n"
                f"🏷️ 目标类别： {target_category}\n"
                f"🧭 状态估计： {payload['status_evaluation']}\n"
                f"📍 目标位置： {target_position}\n"
                f"⚠️ 风险等级： {payload['risk_level']}"
            )
            self.reply_pub.publish(text_msg)

    def destroy_node(self):
        self.http_server.shutdown()
        self.http_server.server_close()
        super().destroy_node()


def main():
    rclpy.init()
    node = N8nResultBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
