# px4_ego

这是一个基于 ROS 2 Humble 的 PX4/Gazebo 仿真与智能巡检工作区，当前主流程围绕
PX4 SITL、Gazebo Harmonic、Offboard 位置控制、RGB-D 目标检测、网页数据桥接，以及
飞书/大模型/n8n 辅助决策闭环展开。

## 默认环境

- OS: Ubuntu 22.04
- ROS 2: Humble
- Gazebo: Harmonic
- PX4: v1.14.x
- PX4 仿真目录: `~/PX4-Autopilot`
- Gazebo 模型: `~/.simulation-gazebo/models/x500_lidar_depth_3d/model.sdf`
- Gazebo 世界: `~/.simulation-gazebo/worlds/flyfield4.sdf`
- YOLO: yolov10n，当前权重路径为 `src/detect/models/best.pt`

当前代码里部分资源路径使用了 `/home/ubuntu22/px4_ego` 绝对路径，建议保持仓库放在该目录。
如果移动工作区，需要同步修改检测模型、目标图片、报告输出和 rosbag 路径。

## 包结构

| 包 | 作用 |
| --- | --- |
| `px4_ego_py` | PX4 Offboard 状态机、DS5 手柄控制、WebSocket 数据桥 |
| `px4_bridge_cpp` | C++ 桥接节点：Gazebo 真值里程计、深度图 QoS 转发、点云坐标转换、`/cmd_vel` 到位置指令 |
| `detect` | RGB-D 目标检测，保存目标图片并发布检测结果 |
| `llm_ros2_agent` | 中文语音/飞书指令、大模型解析、n8n 回调、报告生成 |
| `px4_msgs` | PX4 v1.14 消息定义 |
| `quadrotor_msgs` | EGO/quadrotor 相关消息，主要使用 `PositionCommand` |

## 依赖准备

先确保 ROS 2 Humble、Gazebo Harmonic、PX4 SITL 和 Micro XRCE-DDS Agent 已安装，并且
`MicroXRCEAgent`、`gz`、`ros2` 都在当前终端 PATH 中。

常用 ROS 依赖可用 rosdep 安装：

```bash
cd /home/ubuntu22/px4_ego
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

检测、飞书和大模型相关 Python 依赖：

```bash
python3 -m pip install --user ultralytics torch opencv-python openai lark-oapi websockets requests
```

Gazebo 资源目录需要存在：

```bash
ls ~/.simulation-gazebo/models/x500_lidar_depth_3d/model.sdf
ls ~/.simulation-gazebo/worlds/flyfield4.sdf
ls ~/.simulation-gazebo/server.config
```

PX4 可执行文件需要存在：

```bash
ls ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

## 构建

```bash
cd /home/ubuntu22/px4_ego
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

之后每个新终端都需要：

```bash
cd /home/ubuntu22/px4_ego
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 启动主仿真流程

```bash
ros2 launch px4_ego_py lidar_3d_offboard_position_mode.launch.py
```

该 launch 会启动：

- Gazebo Harmonic，并加载 `flyfield4.sdf`
- PX4 SITL，机型为 `x500_lidar_depth_3d`
- `MicroXRCEAgent udp4 -p 8888`
- `ros_gz_bridge`，桥接 RGB、深度、IMU、点云和 `/clock`
- `gazebo_truth_to_visual_odometry`，将 Gazebo 真值发布到 `/ego/odom_world` 和 `/fmu/in/vehicle_visual_odometry`
- `offboard_control_test`，接收 `/mode_key` 与 `/drone_0_planning/pos_cmd` 并控制 PX4
- `depth_gz_bridge`，将 `/depth_camera` 转为 `/depth_camera_bestef`
- `target_detector`，执行 RGB-D 目标检测
- `websocket_bridge`，向网页端发布遥测、检测图像、检测结果、点云和电池信息

## 飞行控制

### DS5 手柄

另开终端启动手柄节点：

```bash
ros2 launch px4_ego_py ds5_mode_teleop.launch.py
```

当前映射来自 `sensor_msgs/Joy` 的默认索引：

- `buttons[2]`: 发布 `t`，起飞并进入 Offboard 悬停
- `buttons[1]`: 发布 `o`，使用 `/drone_0_planning/pos_cmd` 进行 Offboard 位置控制
- `buttons[0]`: 发布 `l`，降落
- `axes[1]`: 机体系前后速度
- `axes[0]`: 机体系左右速度
- `axes[3]`: 高度控制
- `axes[2]`: yaw 控制

飞行前可以检查手柄实际映射：

```bash
ros2 topic echo /joy
```

### 键盘模式

兼容简单调试：

```bash
python3 mode_key.py
```

可输入：

- `t`: 起飞、解锁并进入 Offboard 悬停
- `o`: 转发最新 `/drone_0_planning/pos_cmd` 到 PX4
- `l`: 降落

### `/cmd_vel` 兼容桥

如果需要把平面速度命令转为 `PositionCommand`：

```bash
ros2 run px4_bridge_cpp cmd_vel_to_pos_cmd
```

默认订阅 `/cmd_vel` 和 `/fmu/out/vehicle_local_position_v1`，发布
`/drone_0_planning/pos_cmd`。

## 目标检测与网页桥

`detect/target_detector` 订阅：

- `/rgb_camera`
- `/depth_camera_bestef`
- `/ego/odom_world`

发布：

- `/detect/image/compressed`
- `/detect/targets`

检测到的新目标会保存到 `saved_targets/`，并向
`http://localhost:5678/webhook/new_targets` 发送图片和目标类别。

注意：`target_detector` 启动时会清空 `saved_targets/` 目录。

网页端可连接：

```text
ws://localhost:8766
```

WebSocket 当前转发遥测、点云、检测图像、检测结果和电池状态。

## 飞书、大模型与 n8n

语音/飞书链路启动：

```bash
ros2 launch llm_ros2_agent voice_command.launch.py
```

该 launch 会启动：

- `voice_command`: 订阅 `/voice_text`，调用大模型解析中文指令
- `feishu_bridge`: 飞书长连接消息桥，转发飞书文本到 `/voice_text`
- `n8n_result_bridge`: 监听 `0.0.0.0:8000`，接收 n8n 二次识别结果
- `report_generator`: 监听目标识别和任务完成信号，生成火灾救援辅助决策报告

需要提前设置环境变量：

```bash
export MINIMAX_API_KEY=...
export QWEN_API_KEY=...
export FEISHU_APP_ID=...
export FEISHU_APP_SECRET=...
export FEISHU_VERIFICATION_TOKEN=...
export FEISHU_ENCRYPT_KEY=...
```

报告输出到：

```text
output/reports/
```

`voice_command` 收到“开始自主探索”类指令时，会播放当前仓库中的：

```text
rosbag2_2026_04_13-05_02_23/
```

## 关键话题

控制相关：

- `/mode_key`: 模式指令，当前主要使用 `t`、`o`、`l`
- `/drone_0_planning/pos_cmd`: `quadrotor_msgs/PositionCommand`
- `/fmu/in/offboard_control_mode`: PX4 Offboard 心跳
- `/fmu/in/trajectory_setpoint`: PX4 位置 setpoint
- `/fmu/in/vehicle_command`: PX4 解锁、模式切换、降落命令
- `/fmu/out/vehicle_local_position_v1`: PX4 本地位置
- `/fmu/out/vehicle_status_v2`: PX4 状态
- `/fmu/in/vehicle_visual_odometry`: 给 PX4 的视觉里程计输入

仿真与感知相关：

- `/clock`: Gazebo 仿真时间
- `/imu`: Gazebo IMU
- `/rgb_camera`: RGB 图像
- `/rgb_camera/camera_info`: RGB 相机内参
- `/depth_camera`: 原始深度图
- `/depth_camera_bestef`: Best Effort QoS 深度图
- `/lidar_points`: 3D lidar 点云
- `/ego/odom_world`: Gazebo 真值里程计，ENU/world 坐标

智能识别与交互相关：

- `/detect/image/compressed`: 带检测框的压缩图像
- `/detect/targets`: 检测结果 JSON
- `/voice_text`: 中文指令文本
- `/model_reply`: 大模型/报告/图片路径回执
- `/recognized_targets`: n8n 复核后的目标 ID
- `/mission_finished`: 自主探索结束信号

## 常见检查

PX4 没有 ROS 话题时，先确认 XRCE Agent：

```bash
MicroXRCEAgent udp4 -p 8888
```

Gazebo 启动失败时，检查 `~/.simulation-gazebo` 下的模型、世界和 `server.config`。

检测没有结果时，依次检查：
