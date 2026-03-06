import math
import time

import numpy as np
import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleLocalPosition, VehicleCommand, VehicleOdometry
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


class OffboardControl(Node):

    def __init__(self):
        super().__init__('ego_pos_command_publisher')
        self.planning_pos_cmd_timeout_sec = max(
            float(self.declare_parameter('planning_pos_cmd_timeout_sec', 0.5).value),
            0.0,
        )

        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.latest_sim_time = None

        # self.status_sub_v0 = self.create_subscription(
        #     VehicleStatus,
        #     'fmu/out/vehicle_status',
        #     self.vehicle_status_callback,
        #     qos_profile_sub)
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            qos_profile_sub)
        self.status_sub_v1 = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v2',
            self.vehicle_status_callback,
            qos_profile_sub)
        #获取本地位置信息，包含位置、速度和姿态等数据，在仿真中使用
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.vehicle_local_position_callback, qos_profile_sub)
        #获取视觉里程计信息，包含位置、速度和姿态等数据，在真实实验中使用
        self.vehicle_visual_odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.vehicle_visual_odom_callback, qos_profile_sub)
        #接收来自规划模块的位置信息，包含期望位置、速度和航向等数据，用于生成飞行控制命令
        self.planning_pos_cmd_sub = self.create_subscription(
            PositionCommand,'/drone_0_planning/pos_cmd', self.planning_pos_cmd_callback, qos_profile_sub)
    
        #接收控制模式切换命令，包含'm'（manual）、't'（takeoff）、'p'（position hold）、'o'（offboard control）和'l'（land）等模式，用于切换飞行控制策略
        self.mode_cmd_sub = self.create_subscription(
            String, '/mode_key', self.mode_cmd_callback, qos_profile_sub)

        # 发布离线控制模式心跳信号
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', qos_profile_pub)
        #发布目标点（正常）
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, 'fmu/in/trajectory_setpoint', qos_profile_pub)
        #发布控制模式（不正常）
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile_pub)

        self.control_mode = 'm'#初始控制模式为manual（手动），通过订阅'/mode_key'话题接收控制模式切换命令
        self.offboard_setpoint_counter = 0
        self.land_trigger_counter = 0
        
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_local_position_received = False
        self.vehicle_visual_odom = VehicleOdometry()
        self.vehicle_visual_odom_received = False
        self.vehicle_status = VehicleStatus()

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        self.pos_cmd_received = False
        self.takeoff_hover_des_set = False
        self.offboard_hover_des_set = False
        self.hover_setpoint = TrajectorySetpoint()
        # Note: no parameter callbacks are used to prevent sudden inflight changes of radii and omega
        # which would result in large discontinuities in setpoints
        self.theta = 0.0
        self.latest_planning_msg = None
        self.last_planning_msg_time = None
        self.last_control_state_log = None
        self.last_offboard_control_log = None
        self.last_arm_state = None

        self.ros_to_fmu = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])

    def clock_callback(self, msg):
        self.latest_sim_time = msg.clock

    def current_time_msg(self):
        if self.latest_sim_time is not None:
            return self.latest_sim_time
        return self.get_clock().now().to_msg()

    def current_time_us(self):
        current_time = self.current_time_msg()
        return current_time.sec * 1000000 + current_time.nanosec // 1000

    #从四元数提取航向角（yaw）
    def quaternion_to_yaw(self, w, x, y, z):
        # yaw (Z axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    #订阅fmu/out/vehicle_status_v1话题
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        if self.last_arm_state is None:
            self.last_arm_state = msg.arming_state
        elif msg.arming_state != self.last_arm_state:
            if msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("Armed successfully")
            elif msg.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info("Disarmed successfully")
            self.last_arm_state = msg.arming_state

        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.vehicle_status = msg

    #订阅fmu/out/vehicle_local_position_v1话题
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        self.vehicle_local_position_received = True

    #订阅/fmu/in/vehicle_visual_odometry话题
    def vehicle_visual_odom_callback(self, msg):
        self.vehicle_visual_odom = msg
        self.vehicle_visual_odom_received = True
        
    #订阅/drone_0_planning/pos_cmd话题
    def planning_pos_cmd_callback(self, msg):
        self.latest_planning_msg = msg
        self.pos_cmd_received = True
        self.last_planning_msg_time = time.monotonic()
    
    #订阅/mode_key话题
    def mode_cmd_callback(self, msg):
        self.control_mode = msg.data
    
    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.current_time_us()
        self.publisher_offboard_mode.publish(msg)

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
    
    def enter_position_mode(self):
        """Exit offboard mode and switch to position mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,   # 1 = custom mode
            param2=4.0    # 4 = POSITION mode
        )
        self.get_logger().info("Exiting OFFBOARD mode → Switching to POSITION mode")
    
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def takeoff(self):
        """Takeoff to a specified altitude (default 1m)."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=-1.0)
        self.get_logger().info(f"Taking off to 1.0 meters")
    
    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
    
    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.current_time_us()
        self.publisher_vehicle_command.publish(msg)

    def log_control_state_once(self, message):
        if self.last_control_state_log == message:
            return
        self.last_control_state_log = message
        self.get_logger().info(message)

    def planning_pos_cmd_is_fresh(self):
        if not self.pos_cmd_received or self.last_planning_msg_time is None:
            return False
        if self.planning_pos_cmd_timeout_sec <= 0.0:
            return True
        return (
            time.monotonic() - self.last_planning_msg_time
        ) <= self.planning_pos_cmd_timeout_sec

    def invalidate_stale_planning_command(self):
        if self.planning_pos_cmd_is_fresh() or not self.pos_cmd_received:
            return

        self.latest_planning_msg = None
        self.pos_cmd_received = False
        self.get_logger().info("planning pos_cmd timed out")

    def position_msg_pub(self):
        msg = TrajectorySetpoint()
        #判断为仿真环境还是实际环境，并根据接收到的位置信息生成起飞悬停的设定点，发布给PX4进行控制
        if (self.vehicle_local_position_received and not self.vehicle_visual_odom_received):
            if not self.takeoff_hover_des_set:
                #初始化起飞悬停设定点为当前位置的水平坐标和-0.9米的高度，航向与当前位置一致
                self.hover_setpoint.position[0] = self.vehicle_local_position.x 
                self.hover_setpoint.position[1] = self.vehicle_local_position.y 
                self.hover_setpoint.position[2] = -0.9
                self.hover_setpoint.yaw = self.vehicle_local_position.heading
                self.takeoff_hover_des_set = True
            if self.takeoff_hover_des_set:
                msg.position = self.hover_setpoint.position
                msg.yaw = self.hover_setpoint.yaw
        elif (self.vehicle_visual_odom_received):
            if not self.takeoff_hover_des_set:
                self.hover_setpoint.position[0] = self.vehicle_visual_odom.position[0]
                self.hover_setpoint.position[1] = self.vehicle_visual_odom.position[1]
                self.hover_setpoint.position[2] = -0.9
                q_att = self.vehicle_visual_odom.q
                self.hover_setpoint.yaw = self.quaternion_to_yaw(q_att[0],q_att[1],q_att[2],q_att[3])
                self.takeoff_hover_des_set = True
            if self.takeoff_hover_des_set:
                msg.position = self.hover_setpoint.position
                msg.yaw = self.hover_setpoint.yaw
        msg.timestamp = self.current_time_us()
        self.publisher_trajectory.publish(msg)
    
    # VehicleLocalPosition /fmu/out/vehicle_local_position_v1 for simulation
    # VehicleOdometry /fmu/in/vehicle_visual_odometry for real experiment
    def hover_cmd_pub(self):
        if self.last_offboard_control_log != "hover mode in offboard":
            self.last_offboard_control_log = "hover mode in offboard"
            self.get_logger().info("hover mode in offboard")
        msg = TrajectorySetpoint()
        if (self.vehicle_local_position_received and not self.vehicle_visual_odom_received):
            if not self.offboard_hover_des_set:
                self.hover_setpoint.position[0] = self.vehicle_local_position.x
                self.hover_setpoint.position[1] = self.vehicle_local_position.y
                self.hover_setpoint.position[2] = self.vehicle_local_position.z
                self.hover_setpoint.yaw = self.vehicle_local_position.heading
                self.offboard_hover_des_set = True
            if self.offboard_hover_des_set:
                msg.position = self.hover_setpoint.position
                msg.yaw = self.hover_setpoint.yaw
        elif (self.vehicle_visual_odom_received):
            if not self.offboard_hover_des_set:
                self.hover_setpoint.position[0] = self.vehicle_visual_odom.position[0]
                self.hover_setpoint.position[1] = self.vehicle_visual_odom.position[1]
                self.hover_setpoint.position[2] = self.vehicle_visual_odom.position[2]
                q_att = self.vehicle_visual_odom.q
                self.hover_setpoint.yaw = self.quaternion_to_yaw(q_att[0],q_att[1],q_att[2],q_att[3])
                self.offboard_hover_des_set = True
            if self.offboard_hover_des_set:
                msg.position = self.hover_setpoint.position
                msg.yaw = self.hover_setpoint.yaw
        msg.timestamp = self.current_time_us()
        self.publisher_trajectory.publish(msg)
    
    def ego_cmd_pub(self):
        if self.last_offboard_control_log != "offboard velocity":
            self.last_offboard_control_log = "offboard velocity"
            self.get_logger().info("flying in offboard mode")
        msg = TrajectorySetpoint()
        msg.timestamp = self.current_time_us()
        planning_position = np.array([self.latest_planning_msg.position.x, 
                                    self.latest_planning_msg.position.y, 
                                    self.latest_planning_msg.position.z])
        planning_velocity = np.array([self.latest_planning_msg.velocity.x,
                                    self.latest_planning_msg.velocity.y,
                                    self.latest_planning_msg.velocity.z])
        inv_ros_to_fmu = np.linalg.inv(self.ros_to_fmu)
        planning_position = inv_ros_to_fmu @ planning_position
        planning_velocity = inv_ros_to_fmu @ planning_velocity
        msg.position[0] = planning_position[0]
        msg.position[1] = planning_position[1]
        msg.position[2] = planning_position[2]
        msg.velocity[0] = planning_velocity[0]
        msg.velocity[1] = planning_velocity[1]
        msg.velocity[2] = planning_velocity[2]
        yaw_enu = self.latest_planning_msg.yaw
        msg.yaw = np.arctan2(np.cos(yaw_enu),np.sin(yaw_enu))
        # msg.yaw = np.arccos(np.sin(self.latest_planning_msg.yaw))
        self.publisher_trajectory.publish(msg)
    


    def cmdloop_callback(self):
        self.publish_offboard_control_heartbeat_signal()
        if self.control_mode == 'm':
            self.log_control_state_once("manual control")
            return

        if self.control_mode == 't':
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                # self.publish_offboard_control_heartbeat_signal()
                self.engage_offboard_mode()
            if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.arm()
            self.offboard_hover_des_set = False
            self.position_msg_pub()
            self.log_control_state_once("takeoff")
            return

        if self.control_mode == 'p':
            self.takeoff_hover_des_set = False
            self.hover_cmd_pub()
            self.log_control_state_once("position mode")
            return

        if (self.control_mode == 'o' and not self.pos_cmd_received):
            self.in_position_hold = True
            # self.enter_position_mode()
            self.takeoff_hover_des_set = False
            self.hover_cmd_pub()
            return

        if (self.control_mode == 'o' and self.pos_cmd_received):
            self.invalidate_stale_planning_command()
            self.log_control_state_once("offboard control mode")
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                # self.publish_offboard_control_heartbeat_signal()
                self.engage_offboard_mode()
            if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                    # self.rate_setpoint_msg_pub()
                    # self.position_msg_pub()
                    if self.latest_planning_msg is not None:
                        self.offboard_hover_des_set = False
                        self.takeoff_hover_des_set = False
                        self.ego_cmd_pub()
            return
                        

        if self.control_mode == 'l':
            self.land()
            self.log_control_state_once("land mode")
            return
        
        if self.control_mode == 'd':
            self.disarm()
            return


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


    # def ego_vel_cmd_pub(self):
    #     msg = TrajectorySetpoint()
    #     msg.timestamp = self.current_time_us()
    #     planning_position = np.array([self.latest_planning_msg.position.x, 
    #                                 self.latest_planning_msg.position.y, 
    #                                 self.latest_planning_msg.position.z])
    #     planning_velocity = np.array([self.latest_planning_msg.velocity.x,
    #                                 self.latest_planning_msg.velocity.y,
    #                                 self.latest_planning_msg.velocity.z])
    #     inv_ros_to_fmu = np.linalg.inv(self.ros_to_fmu)
    #     planning_position = inv_ros_to_fmu @ planning_position
    #     planning_velocity = inv_ros_to_fmu @ planning_velocity
    #     # msg.position[0] = planning_position[0]
    #     # msg.position[1] = planning_position[1]
    #     # msg.position[2] = planning_position[2]
    #     # msg.velocity[0] = planning_velocity[0]
    #     # msg.velocity[1] = planning_velocity[1]
    #     # msg.velocity[2] = planning_velocity[2]
    #     current_position = np.array([0.0, 0.0, 0.0])
    #     if (self.vehicle_local_position_received and not self.vehicle_visual_odom_received):
    #         current_position[0] = self.vehicle_local_position.x
    #         current_position[1] = self.vehicle_local_position.y
    #         current_position[2] = self.vehicle_local_position.z
    #     elif (self.vehicle_visual_odom_received):
    #         current_position[0] = self.vehicle_visual_odom.position[0]
    #         current_position[1] = self.vehicle_visual_odom.position[1]
    #         current_position[2] = self.vehicle_visual_odom.position[2]
    #     msg.position[0] = float('nan')
    #     msg.position[1] = float('nan')
    #     msg.position[2] = float('nan')
    #     msg.velocity[0] = 0.98* (planning_position[0] - current_position[0]) + planning_velocity[0]
    #     msg.velocity[1] = 0.98* (planning_position[1] - current_position[1]) + planning_velocity[1]
    #     msg.velocity[2] = 1.00* (planning_position[2] - current_position[2]) + planning_velocity[2]
    #     yaw_enu = self.latest_planning_msg.yaw
    #     msg.yaw = np.arctan2(np.cos(yaw_enu),np.sin(yaw_enu))
    #     # msg.yaw = np.arccos(np.sin(self.latest_planning_msg.yaw))
    #     self.publisher_trajectory.publish(msg)
