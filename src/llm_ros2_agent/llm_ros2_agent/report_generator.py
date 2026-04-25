#!/usr/bin/env python3
import html
import json
import os
import re
from datetime import datetime
from pathlib import Path
from threading import Thread

import rclpy
from openai import OpenAI
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.styles import getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import Image as ReportImage
from reportlab.platypus import Paragraph
from reportlab.platypus import SimpleDocTemplate
from reportlab.platypus import Spacer
from reportlab.platypus import Table
from reportlab.platypus import TableStyle
from rclpy.node import Node
from std_msgs.msg import String


MODEL = "qwen3.6-plus"
OUTPUT_DIR = Path("/home/ubuntu22/px4_ego/output/reports")
IMAGE_DIR = Path("/home/ubuntu22/px4_ego/saved_targets")
TARGET_IMAGE_PATTERN = re.compile(
    r"^(?P<target_id>[^_]+)_(?P<category>.+)_x_(?P<x>-?\d+(?:\.\d+)?)_y_(?P<y>-?\d+(?:\.\d+)?)_conf_(?P<confidence>\d+(?:\.\d+)?)\.[^.]+$"
)
CLASS_MAP = {
    "fire": "火源",
    "barrel": "桶状容器",
    "tfk": "通风口",
    "pdx": "配电箱",
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

        self.targets[target_id] = {
            "id": target_id,
            "类别": CLASS_MAP.get(match.group("category"), match.group("category")),
            "坐标": f"({float(match.group('x')):.2f}, {float(match.group('y')):.2f})",
            "检测置信度": f"{float(match.group('confidence')):.2f}",
        }

    # 收到任务完成信号后，启动后台线程生成报告。
    def mission_finished_callback(self, msg):
        if self.report_running:
            return

        self.report_running = True
        Thread(target=self.generate_report, daemon=True).start()

    # 汇总已缓存目标，调用大模型并输出报告文件。
    def generate_report(self):
        targets = list(self.targets.values())
        report_text = self.call_llm(targets)
        pdf_path, json_path = self.write_report(targets, report_text)

        self.publish_reply(
            f"火灾救援辅助决策报告已生成。\nJSON: {json_path}"
        )
        self.publish_reply(str(pdf_path))

    # 调用大模型，根据目标类别、坐标和置信度生成报告正文。
    def call_llm(self, targets):
        api_key = os.environ.get("QWEN_API_KEY") or os.environ["DASHSCOPE_API_KEY"]
        client = OpenAI(
            api_key=api_key,
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
        )

        prompt = (
            "请基于以下已通过二次识别的无人机巡检目标，生成一份《火灾救援辅助决策报告》。\n"
            "要求：\n"
            "1. 标题固定为《火灾救援辅助决策报告》。\n"
            "2. 使用 #、##、### 标题层级组织正文，包含任务概况、火灾风险研判、辅助决策建议、复核与处置优先级。\n"
            "3. 不要在正文中逐项列出目标识别清单，程序会自动生成目标识别清单表格。\n"
            "4. 不要编造未给出的目标、坐标或现场信息；风险研判只能基于目标类别和坐标关系。\n"
            "5. 语言正式、简洁，适合救援现场辅助决策。\n\n"
            f"目标数据：\n{json.dumps(targets, ensure_ascii=False, indent=2)}"
        )

        response = client.chat.completions.create(
            model=MODEL,
            temperature=0.2,
            max_tokens=1800,
            messages=[
                {
                    "role": "system",
                    "content": "你是一名火灾救援现场辅助决策报告编写助手，只根据用户给出的结构化数据写报告。",
                },
                {
                    "role": "user",
                    "content": prompt,
                },
            ],
        )
        return response.choices[0].message.content.strip()

    # 将报告正文和目标数据写入 PDF 和 JSON 文件。
    def write_report(self, targets, report_text):
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_path = OUTPUT_DIR / f"fire_rescue_decision_report_{timestamp}"
        pdf_path = base_path.with_suffix(".pdf")
        json_path = base_path.with_suffix(".json")

        json_path.write_text(
            json.dumps({
                "generated_at": datetime.now().isoformat(timespec="seconds"),
                "model": MODEL,
                "targets": targets,
                "report": report_text,
            }, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        self.build_pdf(pdf_path, report_text, targets)
        return pdf_path, json_path

    # 生成 PDF 报告，并把每个目标对应的图片插入文档。
    def build_pdf(self, pdf_path, report_text, targets):
        pdfmetrics.registerFont(UnicodeCIDFont("STSong-Light"))
        styles = getSampleStyleSheet()
        title_style = ParagraphStyle(
            "TitleCN",
            parent=styles["Title"],
            fontName="STSong-Light",
            fontSize=24,
            leading=30,
            alignment=TA_CENTER,
            spaceAfter=16,
        )
        heading_style = ParagraphStyle(
            "HeadingCN",
            parent=styles["Heading2"],
            fontName="STSong-Light",
            fontSize=13,
            leading=18,
            spaceBefore=8,
            spaceAfter=5,
        )
        normal_style = ParagraphStyle(
            "NormalCN",
            parent=styles["Normal"],
            fontName="STSong-Light",
            fontSize=10.5,
            leading=15,
        )

        doc = SimpleDocTemplate(
            str(pdf_path),
            pagesize=A4,
            leftMargin=16 * mm,
            rightMargin=16 * mm,
            topMargin=15 * mm,
            bottomMargin=15 * mm,
        )
        story = []
        title_added = False
        for line in report_text.splitlines():
            line = line.strip()
            if not line:
                story.append(Spacer(1, 4))
                continue
            if not title_added and "火灾救援辅助决策报告" in line:
                story.append(Paragraph(self.paragraph_text(line.lstrip("#").strip()), title_style))
                title_added = True
            elif line.startswith("# "):
                story.append(Paragraph(self.paragraph_text(line[2:]), title_style))
            elif line.startswith("## "):
                story.append(Paragraph(self.paragraph_text(line[3:]), heading_style))
            elif line.startswith("### "):
                story.append(Paragraph(self.paragraph_text(line[4:]), heading_style))
            else:
                story.append(Paragraph(self.paragraph_text(line), normal_style))
                story.append(Spacer(1, 2))

        table_rows = [["ID", "类别", "坐标", "检测置信度"]]
        for target in targets:
            table_rows.append([target["id"], target["类别"], target["坐标"], target["检测置信度"]])

        target_table = Table(table_rows, colWidths=[24 * mm, 38 * mm, 48 * mm, 32 * mm], repeatRows=1)
        target_table.setStyle(TableStyle([
            ("FONTNAME", (0, 0), (-1, -1), "STSong-Light"),
            ("FONTSIZE", (0, 0), (-1, -1), 9),
            ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#E8EEF7")),
            ("GRID", (0, 0), (-1, -1), 0.3, colors.grey),
            ("ALIGN", (0, 0), (-1, -1), "CENTER"),
            ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ]))

        story.append(Spacer(1, 8))
        story.append(Paragraph("目标识别清单", heading_style))
        story.append(target_table)
        story.append(Spacer(1, 8))
        story.append(Paragraph("目标图片", heading_style))
        for target in targets:
            story.append(Paragraph(
                self.paragraph_text(
                    f"目标 {target['id']} | {target['类别']} | 坐标 {target['坐标']} | "
                    f"置信度 {target['检测置信度']}"
                ),
                normal_style,
            ))
            image_path = sorted(IMAGE_DIR.glob(f"{target['id']}_*.jpg"))[0]
            width, height = self.image_size(image_path, 105 * mm, 55 * mm)
            story.append(ReportImage(str(image_path), width=width, height=height))
            story.append(Spacer(1, 6))

        doc.build(story)

    # 按最大宽高等比例缩放图片，避免 PDF 中图片过大。
    def image_size(self, image_path, max_width, max_height):
        image = ImageReader(str(image_path))
        width, height = image.getSize()
        scale = min(max_width / width, max_height / height)
        return width * scale, height * scale

    # 转义文本，确保中文报告内容能安全写入 PDF 段落。
    def paragraph_text(self, text):
        return html.escape(text).replace("\n", "<br/>")

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
