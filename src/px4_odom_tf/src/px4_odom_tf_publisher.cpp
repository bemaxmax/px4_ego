#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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
      "Publishing %s -> %s TF from %s",
      kFrameId,
      kChildFrameId,
      kLocalPositionTopic);
  }

private:
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
    transform.transform.translation.x = msg->y;
    transform.transform.translation.y = msg->x;
    transform.transform.translation.z = -msg->z;

    const double yaw_enu = kHalfPi - static_cast<double>(msg->heading);
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = std::sin(yaw_enu / 2.0);
    transform.transform.rotation.w = std::cos(yaw_enu / 2.0);

    tf_broadcaster_->sendTransform(transform);
  }

  std::optional<builtin_interfaces::msg::Time> latest_sim_time_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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
