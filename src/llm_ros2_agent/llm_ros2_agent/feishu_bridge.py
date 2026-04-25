#!/usr/bin/env python3
import json
import os
from pathlib import Path
from queue import Empty
from queue import Queue
from threading import Thread

import lark_oapi as lark
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FeishuBridge(Node):
    def __init__(self):
        super().__init__("feishu_bridge")
        self.chat_id = None
        self.pending_texts = Queue()

        self.app_id = os.environ["FEISHU_APP_ID"]
        self.app_secret = os.environ["FEISHU_APP_SECRET"]
        self.verification_token = os.environ["FEISHU_VERIFICATION_TOKEN"]
        self.encrypt_key = os.environ["FEISHU_ENCRYPT_KEY"]
        
        self.voice_text_pub = self.create_publisher(String, "/voice_text", 10)
        self.model_reply_sub = self.create_subscription(String, "/model_reply", self.model_reply_callback, 10)
        self.create_timer(0.1, self.publish_pending_texts)

        self.lark_client = lark.Client.builder() \
            .app_id(self.app_id) \
            .app_secret(self.app_secret) \
            .build()

        self.ws_client = lark.ws.Client(
            self.app_id,
            self.app_secret,
            event_handler=self.create_event_handler(),
            log_level=lark.LogLevel.INFO,
        )
        # 飞书长连接事件循环会阻塞，单独起线程转发文本到 ROS 2。
        self.ws_thread = Thread(target=self.ws_client.start, daemon=True)
        self.ws_thread.start()

    def create_event_handler(self):
        return lark.EventDispatcherHandler.builder(
            self.encrypt_key,
            self.verification_token,
        ).register_p2_im_message_receive_v1(
            self.do_p2_im_message_receive_v1
        ).build()

    def do_p2_im_message_receive_v1(self, data):
        if data.event.sender.sender_type != "user":
            return

        content = json.loads(data.event.message.content)
        text = content.get("text", "").strip()

        self.chat_id = data.event.message.chat_id
        self.pending_texts.put(text)#这是单独的线程，直接放到队列里，主线程定时检查并转发到 ROS 2。
        self.get_logger().info(f"收到飞书指令: {text}")

    def model_reply_callback(self, msg):
        if self.chat_id is None:
            self.get_logger().warn(f"飞书还未建立对话，无法回执: {msg.data}")
            return

        reply = msg.data.strip()
        suffix = Path(reply).suffix.lower()
        if suffix in {".jpg", ".pdf"}:
            reply_path = Path(reply)
            if reply_path.is_file():
                if suffix == ".jpg":
                    self.send_back_image(self.chat_id, reply_path)
                    return
                self.send_back_file(self.chat_id, reply_path)
                return

        self.send_back_text(self.chat_id, reply)

    # 回传文本消息
    def send_back_text(self, chat_id, text):
        request_body = lark.im.v1.CreateMessageRequestBody.builder() \
            .receive_id(chat_id) \
            .msg_type("text") \
            .content(json.dumps({"text": text}, ensure_ascii=False)) \
            .build()

        request = lark.im.v1.CreateMessageRequest.builder() \
            .receive_id_type("chat_id") \
            .request_body(request_body) \
            .build()

        response = self.lark_client.im.v1.message.create(request)
        if not response.success():
            self.get_logger().warn(f"飞书回执发送失败: code={response.code}, msg={response.msg}")

    # 回传pdf消息
    def send_back_file(self, chat_id, file_path):
        with open(file_path, "rb") as file_obj:
            upload_request_body = lark.im.v1.CreateFileRequestBody.builder() \
                .file_type("pdf") \
                .file_name(file_path.name) \
                .file(file_obj) \
                .build()

            upload_request = lark.im.v1.CreateFileRequest.builder() \
                .request_body(upload_request_body) \
                .build()

            upload_response = self.lark_client.im.v1.file.create(upload_request)

        if not upload_response.success():
            self.get_logger().warn(f"飞书文件上传失败: code={upload_response.code}, msg={upload_response.msg}")
            return

        request_body = lark.im.v1.CreateMessageRequestBody.builder() \
            .receive_id(chat_id) \
            .msg_type("file") \
            .content(json.dumps({"file_key": upload_response.data.file_key}, ensure_ascii=False)) \
            .build()

        request = lark.im.v1.CreateMessageRequest.builder() \
            .receive_id_type("chat_id") \
            .request_body(request_body) \
            .build()

        response = self.lark_client.im.v1.message.create(request)
        if not response.success():
            self.get_logger().warn(f"飞书文件回执发送失败: code={response.code}, msg={response.msg}")

    # 回传图片消息
    def send_back_image(self, chat_id, image_path):
        with open(image_path, "rb") as image_obj:
            upload_request_body = lark.im.v1.CreateImageRequestBody.builder() \
                .image_type("message") \
                .image(image_obj) \
                .build()

            upload_request = lark.im.v1.CreateImageRequest.builder() \
                .request_body(upload_request_body) \
                .build()

            upload_response = self.lark_client.im.v1.image.create(upload_request)

        if not upload_response.success():
            self.get_logger().warn(f"飞书图片上传失败: code={upload_response.code}, msg={upload_response.msg}")
            return

        request_body = lark.im.v1.CreateMessageRequestBody.builder() \
            .receive_id(chat_id) \
            .msg_type("image") \
            .content(json.dumps({"image_key": upload_response.data.image_key}, ensure_ascii=False)) \
            .build()

        request = lark.im.v1.CreateMessageRequest.builder() \
            .receive_id_type("chat_id") \
            .request_body(request_body) \
            .build()

        response = self.lark_client.im.v1.message.create(request)
        if not response.success():
            self.get_logger().warn(f"飞书图片回执发送失败: code={response.code}, msg={response.msg}")

    # 将飞书指令转发到 ROS 2 的 /voice_text 话题，每0.1秒检查一次队列。
    def publish_pending_texts(self):
        while True:
            try:
                text = self.pending_texts.get_nowait()
            except Empty:
                break

            msg = String()
            msg.data = text
            self.voice_text_pub.publish(msg)
            self.get_logger().info(f"已转发到 /voice_text: {text}")


def main():
    rclpy.init()
    node = FeishuBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
