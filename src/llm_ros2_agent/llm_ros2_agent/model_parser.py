#!/usr/bin/env python3
# 创建模型，并解析模型输出为标准的json格式，供后续执行。
import json
import os
import re

from openai import OpenAI


def create_model_client(model_name):
    if model_name.startswith("MiniMax-"):
        return OpenAI(
            api_key=os.environ["MINIMAX_API_KEY"],
            base_url="https://api.minimaxi.com/v1",
        )
    elif model_name.startswith("qwen"):
        return OpenAI(
            api_key=os.environ["QWEN_API_KEY"],
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
        )


def parse_command(user_text, client, model_name, system_prompt):
    response = client.chat.completions.create(
        model=model_name,
        max_tokens=1000,
        messages=[
            {"role": "system", "content": system_prompt},
            {
                "role": "user",
                "content": user_text,
            },
        ],
        tools=[
            {
                "type": "function",
                "function": {
                    "name": "publish_mode_key",
                    "description": "发布无人机模式控制指令。t 表示起飞，o 表示外部控制或离机模式，l 表示降落。",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "mode_key": {
                                "type": "string",
                                "enum": ["t", "o", "l"],
                                "description": "要发布到 /mode_key 的控制字符。",
                            },
                        },
                        "required": ["mode_key"],
                    },
                },
            },
            {
                "type": "function",
                "function": {
                    "name": "play_rosbag",
                    "description": "播放当前目录下的 rosbag2 数据，用于开始自主探索。",
                    "parameters": {
                        "type": "object",
                        "properties": {},
                    },
                },
            },
        ],
    )

    message = response.choices[0].message
    for tool_call in message.tool_calls or []:
        if tool_call.function.name == "publish_mode_key":
            tool_input = json.loads(tool_call.function.arguments)
            return {
                "type": "tool",
                "tool_name": "publish_mode_key",
                "mode_key": tool_input.get("mode_key"),
            }
        elif tool_call.function.name == "play_rosbag":
            return {
                "type": "tool",
                "tool_name": "play_rosbag",
            }

    content = re.sub(r"<think>.*?</think>\s*", "", message.content or "", flags=re.DOTALL).strip()
    return {
        "type": "reply",
        "reply": content,
    }
