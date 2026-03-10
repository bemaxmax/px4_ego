# PX4 状态机与 EGO 轨迹指令发布

本项目用于管理飞行状态机，并向规划侧发布轨迹/位置控制指令。

## 主要节点

- `offboard_control_test`：状态机主节点，负责模式切换与控制流程。
- `ds5_mode_teleop`：读取 DS5 手柄输入，发布模式按键与 `PositionCommand`。
- `cmd_vel_to_pos_cmd`：将 `cmd_vel` 转为位置控制指令（用于 nav2 场景）。

## 构建与基础启动

```bash
git clone https://github.com/DongnanHu6556/px4_ego.git
cd px4_ego
colcon build
source install/setup.bash
ros2 run px4_ego_py offboard_control_test
```

状态机默认先在手动控制相关流程下运行，之后可通过 DS5 切换模式并发送 offboard setpoint。

## DS5 启动方式（推荐）

使用一个 launch 自动启动 `joy/game_controller_node` 和 `px4_ego_py/ds5_mode_teleop`：

```bash
cd px4_ego
source install/setup.bash
ros2 launch px4_ego_py ds5_mode_teleop.launch.py
```

可选参数示例：

```bash
ros2 launch px4_ego_py ds5_mode_teleop.launch.py device_id:=0 autorepeat_rate:=50.0 joy_topic:=/joy
```

## Nav2 启动指引

`nav2.launch.py` 会同时启动以下内容：

- `robot_state_publisher`：发布 `base_footprint -> base_link -> 传感器` 的静态 TF
- `px4_odom_tf_publisher`：将 PX4 本地位姿转换为 `odom -> base_footprint`
- `cmd_vel_to_pos_cmd`：将 Nav2 的 `/cmd_vel` 转为 `/drone_0_planning/pos_cmd`
- `nav2_bringup`：启动 map server、AMCL、planner、controller 等导航模块
- `rviz2`：打开 Nav2 默认 RViz 配置

### 启动前准备

- 保持 `offboard_control_test` 运行，它负责接收 `/drone_0_planning/pos_cmd` 并发送 PX4 控制指令。
- 确认已经有可用的 `/scan`、`/tf`、`/map` 以及 PX4 位姿反馈。
- 在实际飞行中，先手动起飞并稳定悬停，再让 Nav2 接管水平导航。

### 推荐启动顺序

终端 1：启动状态机主节点

```bash
cd px4_ego
source install/setup.bash
ros2 run px4_ego_py offboard_control_test
```

终端 2：启动手柄或键盘模式输入

```bash
ros2 launch px4_ego_py ds5_mode_teleop.launch.py
```

或：

```bash
python3 mode_key.py
```

终端 3：启动 Nav2

仿真环境下（有 `/clock`）：

```bash
ros2 launch px4_ego_py nav2.launch.py
```

真实环境或没有 `/clock` 时：

```bash
ros2 launch px4_ego_py nav2.launch.py use_sim_time:=false
```

可选参数示例：

```bash
ros2 launch px4_ego_py nav2.launch.py \
  use_sim_time:=false \
  map:=/absolute/path/to/map.yaml \
  params_file:=/absolute/path/to/nav2_params.yaml
```

### RViz 中的使用流程

1. 先确认固定坐标系为 `map`，并且能看到地图、激光和 TF。
2. 点击 `2D Pose Estimate`，在地图上给出无人机初始位姿。
3. 使用手柄或键盘先完成起飞，使无人机进入稳定悬停。
4. 在 RViz 中点击 `Nav2 Goal` 发送目标点。
5. 收到第一条 `/cmd_vel` 后，`cmd_vel_to_pos_cmd` 会自动发布一次 `o`，请求进入 offboard 控制。

### 常用检查项

如果 RViz 中无法正常定位或导航，优先检查：

```bash
ros2 topic echo /scan
ros2 topic echo /fmu/out/vehicle_local_position_v1
ros2 topic echo /drone_0_planning/pos_cmd
ros2 topic echo /cmd_vel
```

如果要看 Nav2 launch 支持的参数：

```bash
ros2 launch px4_ego_py nav2.launch.py --show-args
```

## DS5 按键与摇杆映射

- Square：发布 `t`，触发 `takeoff`
- Triangle：发布 `p`，保持当前位置（position mode）
- Circle：发布 `o`，切换到 offboard mode
- Cross：发布 `l`，执行降落
- 左摇杆：offboard 下的 XY 平移（机体系）
- 右摇杆纵向：offboard 下的 Z 控制
- 右摇杆横向：offboard 下的 yaw 控制

## `ds5_mode_teleop` 行为说明

- 检测到非零手柄输入后，节点以 `50 Hz` 持续发布 setpoint。
- 若 `joy_timeout_sec`（默认 `0.5 s`）内手柄输入持续全 0，节点会停止发布 setpoint。
- 位置与 yaw 目标由“当前真实位姿 + 短时间前视”生成，可避免飞行器受限时目标累积漂移。

## offboard 超时保护

- `offboard_control_test` 会监测 `/drone_0_planning/pos_cmd` 是否持续更新。
- 若 `planning_pos_cmd_timeout_sec`（默认 `0.5 s`）内没有新的 `pos_cmd`，会切换到当前位姿 hover，但仍保持在 offboard。

## 手柄映射检查

飞行前建议先确认当前手柄映射：

```bash
ros2 topic echo /joy
```

`ros2 run joy game_controller_node` 的默认轴假设如下：

- `axes[1] > 0`：前推，对应 ENU `+Y`
- `axes[0] > 0`：左推，对应 ENU `-X`
- `axes[3] > 0`：上推，对应 ENU `+Z`
- `axes[2] > 0`：右推，对应 yaw 向右

如映射不一致，可覆盖参数，例如：

```bash
ros2 run px4_ego_py ds5_mode_teleop --ros-args -p right_y_axis:=4 -p right_x_sign:=1.0
```

## 键盘输入（兼容旧流程）

`mode_key.py` 现在仅保留键盘模式，不再包含 DS5 手柄模式。

```bash
cd px4_ego
python3 mode_key.py
```

- `t`：`takeoff`
- `p`：保持当前位置（position mode）
- `o`：切换到 offboard mode（若无轨迹指令会回到 position mode）
- `l`：降落
- `d`：disarm

输入 `t` 后，飞行器会自动执行 `takeoff`。

## 关键话题

- `/mode_key`：模式按键输入话题
- `/joy`：手柄输入话题
- `/drone_0_planning/pos_cmd`：位置控制指令话题
- `/fmu/out/vehicle_local_position_v1`：本地位置反馈话题
- `/fmu/in/vehicle_visual_odometry`：视觉里程计输入话题

<img width="1830" height="1042" alt="takeoff_screenshot" src="https://github.com/user-attachments/assets/883905a2-4168-4a9b-98cc-df09a062ec74" />
