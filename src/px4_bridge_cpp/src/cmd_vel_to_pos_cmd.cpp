#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

class CmdVelToPosCmdBridge : public rclcpp::Node
{
public:
  CmdVelToPosCmdBridge()
  : Node("cmd_vel_to_pos_cmd"),
    current_position_{0.0, 0.0, 0.0}
  {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    pos_cmd_topic_ = declare_parameter<std::string>("pos_cmd_topic", "/drone_0_planning/pos_cmd");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    update_rate_hz_ = std::max(declare_parameter<double>("update_rate_hz", 50.0), 1.0);
    cmd_vel_timeout_sec_ = std::max(declare_parameter<double>("cmd_vel_timeout_sec", 0.5), 0.0);
    position_lookahead_sec_ = std::max(declare_parameter<double>("position_lookahead_sec", 0.70), 0.0);
    yaw_lookahead_sec_ = std::max(declare_parameter<double>("yaw_lookahead_sec", 0.65), 0.0);

    auto qos_profile_sub = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      rclcpp::QoS(10),
      std::bind(&CmdVelToPosCmdBridge::cmdVelCallback, this, std::placeholders::_1));
    vehicle_local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1",
      qos_profile_sub,
      std::bind(&CmdVelToPosCmdBridge::vehicleLocalPositionCallback, this, std::placeholders::_1));
    vehicle_visual_odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/in/vehicle_visual_odometry",
      qos_profile_sub,
      std::bind(&CmdVelToPosCmdBridge::vehicleVisualOdomCallback, this, std::placeholders::_1));

    pos_cmd_pub_ = create_publisher<quadrotor_msgs::msg::PositionCommand>(pos_cmd_topic_, 10);

    const auto period = rclcpp::Duration::from_seconds(1.0 / update_rate_hz_);
    timer_ = rclcpp::create_timer(
      this,
      get_clock(),
      period,
      std::bind(&CmdVelToPosCmdBridge::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "cmd_vel bridge ready: %s -> %s",
      cmd_vel_topic_.c_str(),
      pos_cmd_topic_.c_str());
  }

private:
  static double quaternionToYaw(double w, double x, double y, double z)
  {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double convertYawBetweenFmuAndRos(double yaw_value)
  {
    return std::atan2(std::cos(yaw_value), std::sin(yaw_value));
  }

  static double wrapAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void updateCurrentPose(double ros_x, double ros_y, double ros_z, double ros_yaw, const std::string & source)
  {
    current_position_ = {ros_x, ros_y, ros_z};
    current_yaw_ = ros_yaw;
    has_pose_ = true;
    if (pose_source_ != source) {
      pose_source_ = source;
      RCLCPP_INFO(get_logger(), "Pose source switched to %s", pose_source_.c_str());
    }
  }

  void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    updateCurrentPose(
      static_cast<double>(msg->y),
      static_cast<double>(msg->x),
      static_cast<double>(-msg->z),
      convertYawBetweenFmuAndRos(static_cast<double>(msg->heading)),
      "vehicle_local_position");
  }

  void vehicleVisualOdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    const auto & q = msg->q;
    const double yaw_ned = quaternionToYaw(q[0], q[1], q[2], q[3]);
    updateCurrentPose(
      static_cast<double>(msg->position[1]),
      static_cast<double>(msg->position[0]),
      static_cast<double>(-msg->position[2]),
      convertYawBetweenFmuAndRos(yaw_ned),
      "vehicle_visual_odometry");
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_linear_x_ = static_cast<double>(msg->linear.x);
    latest_cmd_linear_y_ = static_cast<double>(msg->linear.y);
    latest_cmd_linear_z_ = static_cast<double>(msg->linear.z);
    latest_cmd_yaw_rate_ = static_cast<double>(msg->angular.z);
    last_cmd_vel_time_ = get_clock()->now();
    has_cmd_vel_ = true;
  }

  bool hasRecentCmdVel()
  {
    if (!has_cmd_vel_) {
      return false;
    }
    if (cmd_vel_timeout_sec_ <= 0.0) {
      return true;
    }
    return (get_clock()->now() - last_cmd_vel_time_).seconds() <= cmd_vel_timeout_sec_;
  }

  std::pair<double, double> bodyToWorldPlanarVelocity(
    double forward_velocity,
    double left_velocity) const
  {
    const double cos_yaw = std::cos(current_yaw_);
    const double sin_yaw = std::sin(current_yaw_);
    const double world_x = forward_velocity * cos_yaw - left_velocity * sin_yaw;
    const double world_y = forward_velocity * sin_yaw + left_velocity * cos_yaw;
    return {world_x, world_y};
  }

  void publishPosCmd(
    const std::array<double, 3> & target_position,
    const std::array<double, 3> & target_velocity,
    double target_yaw,
    double target_yaw_rate)
  {
    quadrotor_msgs::msg::PositionCommand msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.position.x = target_position[0];
    msg.position.y = target_position[1];
    msg.position.z = target_position[2];
    msg.velocity.x = target_velocity[0];
    msg.velocity.y = target_velocity[1];
    msg.velocity.z = target_velocity[2];
    msg.acceleration.x = 0.0;
    msg.acceleration.y = 0.0;
    msg.acceleration.z = 0.0;
    msg.yaw = target_yaw;
    msg.yaw_dot = target_yaw_rate;
    pos_cmd_pub_->publish(msg);
  }

  void timerCallback()
  {
    if (!has_pose_) {
      if (!pose_wait_log_sent_) {
        pose_wait_log_sent_ = true;
        RCLCPP_WARN(get_logger(), "Waiting for PX4 pose before forwarding cmd_vel");
      }
      return;
    }

    if (pose_wait_log_sent_) {
      pose_wait_log_sent_ = false;
      RCLCPP_INFO(get_logger(), "Received PX4 pose, cmd_vel bridge is active");
    }

    if (!hasRecentCmdVel()) {
      return;
    }

    const auto [target_vel_x, target_vel_y] = bodyToWorldPlanarVelocity(
      latest_cmd_linear_x_,
      latest_cmd_linear_y_);

    const std::array<double, 3> target_position = {
      current_position_[0] + target_vel_x * position_lookahead_sec_,
      current_position_[1] + target_vel_y * position_lookahead_sec_,
      current_position_[2] + latest_cmd_linear_z_ * position_lookahead_sec_,
    };
    const std::array<double, 3> target_velocity = {
      target_vel_x,
      target_vel_y,
      latest_cmd_linear_z_,
    };
    const double target_yaw = wrapAngle(current_yaw_ + latest_cmd_yaw_rate_ * yaw_lookahead_sec_);
    publishPosCmd(target_position, target_velocity, target_yaw, latest_cmd_yaw_rate_);
  }

  std::string cmd_vel_topic_;
  std::string pos_cmd_topic_;
  std::string frame_id_;

  double update_rate_hz_{50.0};
  double cmd_vel_timeout_sec_{0.5};
  double position_lookahead_sec_{0.70};
  double yaw_lookahead_sec_{0.65};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_visual_odom_sub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 3> current_position_;
  double current_yaw_{0.0};
  double latest_cmd_linear_x_{0.0};
  double latest_cmd_linear_y_{0.0};
  double latest_cmd_linear_z_{0.0};
  double latest_cmd_yaw_rate_{0.0};
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  std::string pose_source_{"none"};
  bool has_pose_{false};
  bool has_cmd_vel_{false};
  bool pose_wait_log_sent_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToPosCmdBridge>());
  rclcpp::shutdown();
  return 0;
}
