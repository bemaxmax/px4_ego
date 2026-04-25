#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr char kLocalPositionTopic[] = "/fmu/out/vehicle_local_position_v1";

}  // namespace

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
    update_rate_hz_ = std::max(declare_parameter<double>("update_rate_hz", 30.0), 1.0);
    cmd_vel_timeout_sec_ = std::max(declare_parameter<double>("cmd_vel_timeout_sec", 0.5), 0.0);

    auto qos_profile_sub = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      rclcpp::QoS(10),
      std::bind(&CmdVelToPosCmdBridge::cmdVelCallback, this, std::placeholders::_1));

    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      kLocalPositionTopic,
      qos_profile_sub,
      std::bind(&CmdVelToPosCmdBridge::vehicleLocalPositionCallback, this, std::placeholders::_1));

    pos_cmd_pub_ = create_publisher<quadrotor_msgs::msg::PositionCommand>(pos_cmd_topic_, 10);

    const auto period = rclcpp::Duration::from_seconds(1.0 / update_rate_hz_);
    timer_ = rclcpp::create_timer(
      this,
      get_clock(),
      period,
      std::bind(&CmdVelToPosCmdBridge::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Planar cmd_vel bridge ready: %s -> %s using %s",
      cmd_vel_topic_.c_str(),
      pos_cmd_topic_.c_str(),
      kLocalPositionTopic);
  }

private:
  static double convertHeadingToRosYaw(double heading)
  {
    return std::atan2(std::cos(heading), std::sin(heading));
  }

  static double wrapAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    if (!std::isfinite(msg->x) || !std::isfinite(msg->y) ||
      !std::isfinite(msg->z) || !std::isfinite(msg->heading))
    {
      return;
    }

    current_position_ = {
      static_cast<double>(msg->y),
      static_cast<double>(msg->x),
      static_cast<double>(-msg->z),
    };
    current_yaw_ = convertHeadingToRosYaw(static_cast<double>(msg->heading));
    has_pose_ = true;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_linear_x_ = static_cast<double>(msg->linear.x);
    latest_cmd_linear_y_ = static_cast<double>(msg->linear.y);
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
        RCLCPP_WARN(get_logger(), "Waiting for PX4 local position before forwarding cmd_vel");
      }
      return;
    }

    if (pose_wait_log_sent_) {
      pose_wait_log_sent_ = false;
      RCLCPP_INFO(get_logger(), "Received PX4 pose, cmd_vel bridge is active");
    }

    std::array<double, 3> target_position = current_position_;
    std::array<double, 3> target_velocity = {0.0, 0.0, 0.0};
    double target_yaw = current_yaw_;
    double target_yaw_rate = 0.0;

    if (!hasRecentCmdVel()) {
      hold_z_ = current_position_[2];
    } else {
      if (!hold_z_.has_value()) {
        hold_z_ = current_position_[2];
      }

      const double dt = 1.0 / update_rate_hz_;
      const auto [target_vel_x, target_vel_y] = bodyToWorldPlanarVelocity(
        latest_cmd_linear_x_,
        latest_cmd_linear_y_);

      target_position[0] += target_vel_x * dt;
      target_position[1] += target_vel_y * dt;
      target_velocity[0] = target_vel_x;
      target_velocity[1] = target_vel_y;
      target_yaw = wrapAngle(current_yaw_ + latest_cmd_yaw_rate_ * dt);
      target_yaw_rate = latest_cmd_yaw_rate_;
    }

    target_position[2] = hold_z_.value_or(current_position_[2]);
    publishPosCmd(target_position, target_velocity, target_yaw, target_yaw_rate);
  }

  std::string cmd_vel_topic_;
  std::string pos_cmd_topic_;
  std::string frame_id_;

  double update_rate_hz_{30.0};
  double cmd_vel_timeout_sec_{0.5};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 3> current_position_;
  double current_yaw_{0.0};
  double latest_cmd_linear_x_{0.0};
  double latest_cmd_linear_y_{0.0};
  double latest_cmd_yaw_rate_{0.0};
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  std::optional<double> hold_z_;
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
