# State machine for state management and ego planner trajectory command publish
```
git clone https://github.com/DongnanHu6556/px4_ego.git
cd px4_ego
colcon build
source install/setup.bash
ros2 run px4_ego_py offboard_control_test
```
The state machine runs in munual control at first. Then we can use a DS5 controller to switch different states and send offboard setpoints:

```
cd px4_ego
source install/setup.bash
ros2 run joy game_controller_node
ros2 run px4_ego_py ds5_mode_teleop
```
- Square: publish `t` for "takeoff"
- Triangle: publish `p` for "hover at current position (position mode)"
- Circle: publish `o` for "switch to offboard mode"
- Cross: publish `l` for "land"
- Left stick: XY translation in offboard (body frame)
- Right stick vertical: Z translation in offboard
- Right stick horizontal: yaw control in offboard

The DS5 teleop node now uses a deadzone and a timer-based closed-loop setpoint:

- `deadzone` defaults to `0.05`, so tiny stick noise is filtered out.
- After `/joy` starts publishing, the node publishes at `50 Hz` while joystick messages remain fresh.
- If no `/joy` message arrives within `joy_timeout_sec` (default `0.5 s`), the node stops publishing setpoints.
- Position and yaw targets are always generated from the latest real vehicle pose plus a short lookahead, which avoids target windup if the drone is blocked or velocity-limited.

Before flight, verify your actual joystick mapping:

```
ros2 topic echo /joy
```

Default axis assumptions for `ros2 run joy game_controller_node` are:

- `axes[1] > 0`: forward -> ENU `+Y`
- `axes[0] > 0`: left -> ENU `-X`
- `axes[3] > 0`: up -> ENU `+Z`
- `axes[2] > 0`: stick right -> yaw right

If your mapping is different, override the teleop parameters, for example:

```
ros2 run px4_ego_py ds5_mode_teleop --ros-args -p right_y_axis:=4 -p right_x_sign:=1.0
```

If you still want the old keyboard input:

```
cd px4_ego
python3 mode_key.py --keyboard
```

- `t` means "takeoff"
- `p` means "hover at current position (position mode)"
- `o` means "switch to offboard mode. (If there is no trajectory command, it will return to position mode)"
- `l` means "land"
- `d` means "disarm"
When you input 't', the drone will auto takeoff.
<img width="1830" height="1042" alt="takeoff_screenshot" src="https://github.com/user-attachments/assets/883905a2-4168-4a9b-98cc-df09a062ec74" />
