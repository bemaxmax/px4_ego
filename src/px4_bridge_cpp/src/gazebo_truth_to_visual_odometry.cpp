// 订阅/dynamic_pose/info话题
// 发送gazebo真实位姿信息到视觉里程计话题到
// /ego/odom_world和/fmu/in/vehicle_visual_odometry
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <gz/math.hh>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport.hh>
#include <nav_msgs/msg/odometry.hpp>

#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr char kDefaultModelName[] = "x500_lidar_depth_3d_0";
constexpr char kPoseTopic[] = "/world/default/dynamic_pose/info";
constexpr char kOdomTopic[] = "/ego/odom_world";
constexpr char kOdomFrameId[] = "world";
constexpr char kVisualOdometryTopic[] = "/fmu/in/vehicle_visual_odometry";
constexpr double kMinDtSec = 0.001;
constexpr double kMaxDtSec = 0.1;
constexpr int8_t kQuality = 100;

}  // namespace

class GazeboTruthToVisualOdometry : public rclcpp::Node
{
public:
  GazeboTruthToVisualOdometry()
  : Node("gazebo_truth_to_visual_odometry")
  {
    model_name_ = declare_parameter<std::string>("model_name", kDefaultModelName);

    const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .best_effort()
      .durability_volatile();
    const auto visual_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .durability_volatile();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(kOdomTopic, odom_qos);
    visual_odometry_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      kVisualOdometryTopic,
      visual_qos);

    const bool subscribed = gz_node_.Subscribe(
      kPoseTopic,
      &GazeboTruthToVisualOdometry::poseInfoCallback,
      this);

    if (!subscribed) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to subscribe to Gazebo pose topic %s",
        kPoseTopic);
      throw std::runtime_error("gazebo pose subscription failed");
    }

    RCLCPP_INFO(
      get_logger(),
      "Forwarding Gazebo truth from %s (model %s) to %s and %s",
      kPoseTopic,
      model_name_.c_str(),
      kOdomTopic,
      kVisualOdometryTopic);
  }

private:
  static std::array<double, 3> enuToNedVector(const std::array<double, 3> & value_enu)
  {
    return {
      value_enu[1],
      value_enu[0],
      -value_enu[2],
    };
  }

  static gz::math::Quaterniond rotateQuaternionToFrdNed(
    const gz::math::Quaterniond & q_flu_to_enu)
  {
    static const auto q_flu_to_frd = gz::math::Quaterniond(0, 1, 0, 0);
    static const auto q_enu_to_ned = gz::math::Quaterniond(0, 0.70711, 0.70711, 0);

    return q_enu_to_ned * q_flu_to_enu * q_flu_to_frd.Inverse();
  }

  void poseInfoCallback(const gz::msgs::Pose_V & msg)
  {
    const auto pose_it = std::find_if(
      msg.pose().begin(),
      msg.pose().end(),
      [this](const gz::msgs::Pose & pose) {return pose.name() == model_name_;});
    if (pose_it == msg.pose().end()) {
      return;
    }

    const auto & stamp = msg.header().stamp();
    builtin_interfaces::msg::Time ros_stamp;
    ros_stamp.sec = static_cast<int32_t>(stamp.sec());
    ros_stamp.nanosec = static_cast<uint32_t>(stamp.nsec());
    const uint64_t stamp_us = static_cast<uint64_t>(stamp.sec()) * 1000000ULL +
      static_cast<uint64_t>(stamp.nsec()) / 1000ULL;

    const auto position_enu = std::array<double, 3>{
      pose_it->position().x(),
      pose_it->position().y(),
      pose_it->position().z(),
    };
    std::array<double, 3> velocity_enu = {0.0, 0.0, 0.0};

    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      if (have_previous_pose_ && stamp_us > previous_stamp_us_) {
        const double dt = std::clamp(
          static_cast<double>(stamp_us - previous_stamp_us_) * 1e-6,
          kMinDtSec,
          kMaxDtSec);
        velocity_enu[0] = (position_enu[0] - previous_position_enu_[0]) / dt;
        velocity_enu[1] = (position_enu[1] - previous_position_enu_[1]) / dt;
        velocity_enu[2] = (position_enu[2] - previous_position_enu_[2]) / dt;
      }

      previous_position_enu_ = position_enu;
      previous_stamp_us_ = stamp_us;
      have_previous_pose_ = true;
    }

    publishOdom(ros_stamp, *pose_it, position_enu, velocity_enu);
    publishVehicleVisualOdometry(stamp_us, *pose_it, position_enu, velocity_enu);
  }

  void publishOdom(
    const builtin_interfaces::msg::Time & ros_stamp,
    const gz::msgs::Pose & pose,
    const std::array<double, 3> & position_enu,
    const std::array<double, 3> & velocity_enu)
  {
    const auto & orientation = pose.orientation();

    nav_msgs::msg::Odometry odom{};
    odom.header.stamp = ros_stamp;
    odom.header.frame_id = kOdomFrameId;

    odom.pose.pose.position.x = position_enu[0];
    odom.pose.pose.position.y = position_enu[1];
    odom.pose.pose.position.z = position_enu[2];
    odom.pose.pose.orientation.x = orientation.x();
    odom.pose.pose.orientation.y = orientation.y();
    odom.pose.pose.orientation.z = orientation.z();
    odom.pose.pose.orientation.w = orientation.w();

    odom.twist.twist.linear.x = velocity_enu[0];
    odom.twist.twist.linear.y = velocity_enu[1];
    odom.twist.twist.linear.z = velocity_enu[2];

    odom_pub_->publish(odom);
  }

  void publishVehicleVisualOdometry(
    uint64_t stamp_us,
    const gz::msgs::Pose & pose,
    const std::array<double, 3> & position_enu,
    const std::array<double, 3> & velocity_enu)
  {
    const auto position_ned = enuToNedVector(position_enu);
    const auto velocity_ned = enuToNedVector(velocity_enu);

    const gz::math::Quaterniond q_flu_to_enu(
      pose.orientation().w(),
      pose.orientation().x(),
      pose.orientation().y(),
      pose.orientation().z());
    const gz::math::Quaterniond q_frd_to_ned = rotateQuaternionToFrdNed(q_flu_to_enu);

    px4_msgs::msg::VehicleOdometry odom{};
    odom.timestamp = stamp_us;
    odom.timestamp_sample = stamp_us;
    odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    odom.position = {
      static_cast<float>(position_ned[0]),
      static_cast<float>(position_ned[1]),
      static_cast<float>(position_ned[2]),
    };
    odom.q = {
      static_cast<float>(q_frd_to_ned.W()),
      static_cast<float>(q_frd_to_ned.X()),
      static_cast<float>(q_frd_to_ned.Y()),
      static_cast<float>(q_frd_to_ned.Z()),
    };
    odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    odom.velocity = {
      static_cast<float>(velocity_ned[0]),
      static_cast<float>(velocity_ned[1]),
      static_cast<float>(velocity_ned[2]),
    };
    const float nan = std::numeric_limits<float>::quiet_NaN();
    odom.angular_velocity = {nan, nan, nan};
    odom.quality = kQuality;

    visual_odometry_pub_->publish(odom);
  }

  bool have_previous_pose_{false};
  std::string model_name_;
  uint64_t previous_stamp_us_{0};
  std::array<double, 3> previous_position_enu_{0.0, 0.0, 0.0};
  std::mutex state_mutex_;

  gz::transport::Node gz_node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odometry_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboTruthToVisualOdometry>());
  rclcpp::shutdown();
  return 0;
}
