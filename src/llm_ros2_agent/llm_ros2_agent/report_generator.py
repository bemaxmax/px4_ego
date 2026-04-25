#!/usr/bin/env python3
import json
import os
import re
from datetime import datetime
from pathlib import Path
from threading import Thread

import rclpy
from openai import OpenAI
from rclpy.node import Node
from std_msgs.msg import String


MODEL = "qwen3.6-plus"
OUTPUT_DIR = Path("/home/ubuntu22/px4_ego/output/reports")
IMAGE_DIR = Path("/home/ubuntu22/px4_ego/saved_targets")
TEMPLATE_PATH = Path("/home/ubuntu22/px4_ego/example.html")
TARGET_IMAGE_PATTERN = re.compile(
    r"^(?P<target_id>[^_]+)_(?P<category>.+)_x_(?P<x>-?\d+(?:\.\d+)?)_y_(?P<y>-?\d+(?:\.\d+)?)_conf_(?P<confidence>\d+(?:\.\d+)?)\.[^.]+$"
)
CLASS_MAP = {
    "fire": "火源",
    "barrel": "桶状容器",
    "tfk": "通风口",
    "pdx": "配电箱",
}
RISK_MAP = {
    "fire": "高",
    "barrel": "高",
    "pdx": "中",
    "tfk": "无",
}


class ReportGenerator(Node):
    # 初始化报告生成节点，订阅目标 id 和任务完成信号。
    def __init__(self):
        super().__init__("report_generator")
        self.targets = {}
        self.report_running = False

        self.create_subscription(String, "/recognized_targets", self.target_callback, 10)
        self.create_subscription(String, "/mission_finished", self.mission_finished_callback, 10)
        self.reply_pub = self.create_publisher(String, "/model_reply", 10)

    # 收到通过二次识别的目标 id 后，查找图片并缓存目标信息。
    def target_callback(self, msg):
        payload = json.loads(msg.data)
        target_id = str(payload["target_id"]).strip()
        image_path = sorted(IMAGE_DIR.glob(f"{target_id}_*.jpg"))[0]

        match = TARGET_IMAGE_PATTERN.match(image_path.name)
        category = match.group("category")
        x = float(match.group("x"))
        y = float(match.group("y"))
        confidence = float(match.group("confidence"))
        risk_level = RISK_MAP.get(category, "待复核")

        self.targets[target_id] = {
            "id": target_id,
            "category": category,
            "类别": CLASS_MAP.get(category, category),
            "坐标": f"({x:.2f}, {y:.2f})",
            "置信度": f"{confidence * 100:.0f}%",
            "风险等级": risk_level,
        }

    # 收到任务完成信号后，启动后台线程生成报告。
    def mission_finished_callback(self, msg):
        if self.report_running:
            return

        self.report_running = True
        Thread(target=self.generate_report, daemon=True).start()

    # 汇总已缓存目标，调用大模型并输出报告文件。
    def generate_report(self):
        try:
            targets = sorted(self.targets.values(), key=lambda target: int(target["id"]))
            report_html = self.call_llm(targets)
            html_path = self.write_report(report_html)
        except Exception as exc:
            self.report_running = False
            self.get_logger().warn(f"火灾救援辅助决策报告生成失败: {exc}")
            self.publish_reply(f"火灾救援辅助决策报告生成失败: {exc}")
            return

        self.publish_reply(f"火灾救援辅助决策报告已生成。\nHTML: {html_path}")
        self.publish_reply(str(html_path))

    # 调用大模型，参考 example.html 直接生成完整 HTML 报告。
    def call_llm(self, targets):
        api_key = os.environ.get("QWEN_API_KEY") or os.environ["DASHSCOPE_API_KEY"]
        client = OpenAI(
            api_key=api_key,
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
        )
        template_html = TEMPLATE_PATH.read_text(encoding="utf-8")
        generated_at = datetime.now().strftime("%Y年%m月%d日 %H:%M:%S")

        prompt = (
            "请参考下面的 HTML 模板，基于目标数据生成完整的《火灾辅助救援决策报告》HTML 文件。\n"
            "只输出从 <!DOCTYPE html> 开始到 </html> 结束的完整 HTML，不要输出 Markdown、代码块或解释文字。\n"
            f"报告生成时间：{generated_at}\n\n"
            f"HTML模板：\n{template_html}\n\n"
            f"目标数据：\n{json.dumps(targets, ensure_ascii=False, indent=2)}"
        )

        response = client.chat.completions.create(
            model=MODEL,
            messages=[
                {
                    "role": "system",
                    "content": "你是一名火灾救援现场辅助决策报告编写助手，只根据用户给出的模板和结构化数据生成 HTML 报告。",
                },
                {
                    "role": "user",
                    "content": prompt,
                },
            ],
        )
        return response.choices[0].message.content.strip()

    # 将大模型生成的完整 HTML 报告写入文件。
    def write_report(self, report_html):
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_path = OUTPUT_DIR / f"fire_rescue_decision_report_{timestamp}"
        html_path = base_path.with_suffix(".html")

        html_path.write_text(report_html, encoding="utf-8")
        return html_path

    # 通过 /model_reply 发布文本或文件路径给飞书桥。
    def publish_reply(self, text):
        msg = String()
        msg.data = text
        self.reply_pub.publish(msg)


# 启动 ROS 2 节点并进入事件循环。
def main():
    rclpy.init()
    node = ReportGenerator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
