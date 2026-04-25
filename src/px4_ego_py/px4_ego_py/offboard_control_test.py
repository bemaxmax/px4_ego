import numpy as np
import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleLocalPosition, VehicleCommand
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class OffboardControl(Node):

    def __init__(self):
        super().__init__('ego_pos_command_publisher')

        #接收来自PX4的飞行状态信息
        self.status_sub_v1 = self.create_subscription(VehicleStatus, 'fmu/out/vehicle_status_v2', self.vehicle_status_callback, qos_profile_sensor_data)
        #获取本地位置信息
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.vehicle_local_position_callback, qos_profile_sensor_data)
        #接收飞行控制命令
        self.planning_pos_cmd_sub = self.create_subscription(
            PositionCommand, '/drone_0_planning/pos_cmd', self.planning_pos_cmd_callback, 10)
        #接收控制模式切换命令
        self.mode_cmd_sub = self.create_subscription(String, '/mode_key', self.mode_cmd_callback, 10)

        # 发布离线控制模式心跳信号
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', 10)
        #发布目标点
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, 'fmu/in/trajectory_setpoint', 10)
        #发布控制模式
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.control_mode = 'm'#初始控制模式为manual（手动）
        
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_local_position_received = False
        self.vehicle_status = VehicleStatus()

        self.timer = self.create_timer(0.02, self.cmdloop_callback)

        self.vehicle_status_received = False        
        self.pos_cmd_received = False
        self.takeoff_hover_des_set = False
        self.latest_planning_msg = None

    def current_time(self):
        return self.get_clock().now().nanoseconds // 1000

    #订阅fmu/out/vehicle_status_v1话题
    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.vehicle_status_received = True

    #订阅fmu/out/vehicle_local_position_v1话题
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        self.vehicle_local_position_received = True
        
    #订阅/drone_0_planning/pos_cmd话题
    def planning_pos_cmd_callback(self, msg):
        self.latest_planning_msg = msg
        self.pos_cmd_received = True
    
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
        msg.timestamp = self.current_time()
        self.publisher_offboard_mode.publish(msg)

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
    
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
    
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
        msg.timestamp = self.current_time()
        self.publisher_vehicle_command.publish(msg)

    def position_msg_pub(self):
        msg = TrajectorySetpoint()
        #根据接收到的本地位置信息生成起飞悬停设定点，发布给PX4进行控制
        if not self.takeoff_hover_des_set:
            self.hover_setpoint = TrajectorySetpoint()
            #初始化起飞悬停设定点为当前位置上方1.0米，航向与当前位置一致
            self.hover_setpoint.position[0] = self.vehicle_local_position.x 
            self.hover_setpoint.position[1] = self.vehicle_local_position.y 
            self.hover_setpoint.position[2] = self.vehicle_local_position.z - 3.5
            self.hover_setpoint.yaw = self.vehicle_local_position.heading
            self.takeoff_hover_des_set = True
        msg.position = self.hover_setpoint.position
        msg.yaw = self.hover_setpoint.yaw
        msg.timestamp = self.current_time()
        self.publisher_trajectory.publish(msg)

    def ego_cmd_pub(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.current_time()
        msg.position[0] = self.latest_planning_msg.position.y
        msg.position[1] = self.latest_planning_msg.position.x
        msg.position[2] = -self.latest_planning_msg.position.z
        msg.velocity[0] = self.latest_planning_msg.velocity.y
        msg.velocity[1] = self.latest_planning_msg.velocity.x
        msg.velocity[2] = -self.latest_planning_msg.velocity.z
        yaw_enu = self.latest_planning_msg.yaw
        msg.yaw = np.arctan2(np.cos(yaw_enu),np.sin(yaw_enu))
        self.publisher_trajectory.publish(msg)

    def cmdloop_callback(self):

        if not self.vehicle_local_position_received or not self.vehicle_status_received:
            # self.get_logger().warn('未收到本地数据,等待中...')
            return
        
        self.publish_offboard_control_heartbeat_signal()

        if self.control_mode == 't':
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.engage_offboard_mode()
            if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.arm()
            self.position_msg_pub()
            return

        if (self.control_mode == 'o' and self.pos_cmd_received):
            self.ego_cmd_pub()
            return

        if self.control_mode == 'l':
            self.land()
            self.takeoff_hover_des_set = False
            return
        
        return


def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
