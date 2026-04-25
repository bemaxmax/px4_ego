#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

constexpr char kFrameId[] = "odom";
constexpr char kChildFrameId[] = "base_footprint";
constexpr char kClockTopic[] = "/clock";
constexpr char kLocalPositionTopic[] = "/fmu/out/vehicle_local_position_v1";
constexpr char kOdomTopic[] = "/odom";
constexpr double kHalfPi = 1.5707963267948966;

}  // namespace

class Px4OdomTfPublisher : public rclcpp::Node
{
public:
  Px4OdomTfPublisher()
  : Node("px4_odom_tf_publisher")
  {
    const auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(kOdomTopic, rclcpp::QoS(10));

    clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
      kClockTopic,
      qos_profile,
      std::bind(&Px4OdomTfPublisher::clockCallback, this, std::placeholders::_1));

    local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      kLocalPositionTopic,
      qos_profile,
      std::bind(&Px4OdomTfPublisher::vehicleLocalPositionCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing %s -> %s TF and %s from %s",
      kFrameId,
      kChildFrameId,
      kOdomTopic,
      kLocalPositionTopic);
  }

private:
  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double timeMsgToSeconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
  {
    latest_sim_time_ = msg->clock;
  }

  builtin_interfaces::msg::Time currentTimeMsg() const
  {
    if (latest_sim_time_.has_value()) {
      return latest_sim_time_.value();
    }
    return this->now();
  }

  void vehicleLocalPositionCallback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    if (!std::isfinite(msg->x) || !std::isfinite(msg->y) ||
      !std::isfinite(msg->z) || !std::isfinite(msg->heading))
    {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = currentTimeMsg();
    transform.header.frame_id = kFrameId;
    transform.child_frame_id = kChildFrameId;

    // PX4 local position is NED. Convert to ROS ENU.
    const double current_x = static_cast<double>(msg->y);
    const double current_y = static_cast<double>(msg->x);
    const double yaw_enu = normalizeAngle(kHalfPi - static_cast<double>(msg->heading));

    if (!initial_x_.has_value()) {
      initial_x_ = current_x;
      initial_y_ = current_y;
      initial_yaw_ = yaw_enu;
      RCLCPP_INFO(
        this->get_logger(),
        "Captured odom origin at x=%.3f y=%.3f yaw=%.3f rad; z will be fixed to 0.0",
        current_x,
        current_y,
        yaw_enu);
    }

    transform.transform.translation.x = current_x - initial_x_.value();
    transform.transform.translation.y = current_y - initial_y_.value();
    transform.transform.translation.z = 0.0;

    const double relative_yaw = normalizeAngle(yaw_enu - initial_yaw_.value());
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = std::sin(relative_yaw / 2.0);
    transform.transform.rotation.w = std::cos(relative_yaw / 2.0);

    tf_broadcaster_->sendTransform(transform);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = transform.header;
    odom_msg.child_frame_id = transform.child_frame_id;
    odom_msg.pose.pose.position.x = transform.transform.translation.x;
    odom_msg.pose.pose.position.y = transform.transform.translation.y;
    odom_msg.pose.pose.position.z = transform.transform.translation.z;
    odom_msg.pose.pose.orientation = transform.transform.rotation;

    if (msg->v_xy_valid && std::isfinite(msg->vx) && std::isfinite(msg->vy)) {
      const double world_vel_x = static_cast<double>(msg->vy);
      const double world_vel_y = static_cast<double>(msg->vx);
      const double cos_yaw = std::cos(relative_yaw);
      const double sin_yaw = std::sin(relative_yaw);

      odom_msg.twist.twist.linear.x = world_vel_x * cos_yaw + world_vel_y * sin_yaw;
      odom_msg.twist.twist.linear.y = -world_vel_x * sin_yaw + world_vel_y * cos_yaw;
    }
    odom_msg.twist.twist.linear.z = 0.0;

    const double stamp_sec = timeMsgToSeconds(transform.header.stamp);
    if (previous_stamp_sec_.has_value() && previous_relative_yaw_.has_value()) {
      const double dt = stamp_sec - previous_stamp_sec_.value();
      if (dt > 1e-6) {
        odom_msg.twist.twist.angular.z =
          normalizeAngle(relative_yaw - previous_relative_yaw_.value()) / dt;
      }
    }

    previous_stamp_sec_ = stamp_sec;
    previous_relative_yaw_ = relative_yaw;
    odom_pub_->publish(odom_msg);
  }

  std::optional<builtin_interfaces::msg::Time> latest_sim_time_;
  std::optional<double> initial_x_;
  std::optional<double> initial_y_;
  std::optional<double> initial_yaw_;
  std::optional<double> previous_stamp_sec_;
  std::optional<double> previous_relative_yaw_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4OdomTfPublisher>());
  rclcpp::shutdown();
  return 0;
}
