from setuptools import find_packages
from setuptools import setup


package_name = "llm_ros2_agent"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            [
                "config/voice_command_min.yaml",
            ],
        ),
        ("share/" + package_name + "/launch", ["launch/voice_command.launch.py"]),
    ],
    install_requires=["setuptools", "openai", "lark-oapi", "reportlab"],
    zip_safe=True,
    maintainer="ubuntu22",
    maintainer_email="ubuntu22@todo.todo",
    description="ROS 2 package for parsing Chinese voice commands.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "voice_command = llm_ros2_agent.voice_command:main",
            "feishu_bridge = llm_ros2_agent.feishu_bridge:main",
            "n8n_result_bridge = llm_ros2_agent.n8n_result_bridge:main",
            "report_generator = llm_ros2_agent.report_generator:main",
        ],
    },
)
