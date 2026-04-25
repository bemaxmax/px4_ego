#include <array>
#include <cmath>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class LidarPointsToWorld : public rclcpp::Node
{
public:
  LidarPointsToWorld()
  : Node("lidar_points_to_world")
  {
    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto sensor_qos = rclcpp::SensorDataQoS();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_,
      sensor_qos,
      std::bind(&LidarPointsToWorld::cloudCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      odom_qos,
      std::bind(&LidarPointsToWorld::odomCallback, this, std::placeholders::_1));
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, sensor_qos);

    RCLCPP_INFO(
      get_logger(),
      "Point cloud bridge ready: %s -> %s, odom=%s, frame=%s",
      input_cloud_topic_.c_str(),
      output_cloud_topic_.c_str(),
      odom_topic_.c_str(),
      output_frame_id_.c_str());
  }

private:
  using Matrix3 = std::array<double, 9>;
  using Vector3 = std::array<double, 3>;

  static Matrix3 rotationMatrixFromRpy(double roll, double pitch, double yaw)
  {
    const double sr = std::sin(roll);
    const double cr = std::cos(roll);
    const double sp = std::sin(pitch);
    const double cp = std::cos(pitch);
    const double sy = std::sin(yaw);
    const double cy = std::cos(yaw);

    return {
      cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
      -sp, cp * sr, cp * cr};
  }

  static Matrix3 rotationMatrixFromQuaternion(double x, double y, double z, double w)
  {
    const double xx = x * x;
    const double yy = y * y;
    const double zz = z * z;
    const double xy = x * y;
    const double xz = x * z;
    const double yz = y * z;
    const double wx = w * x;
    const double wy = w * y;
    const double wz = w * z;

    return {
      1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
      2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
      2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)};
  }

  static Vector3 multiply(const Matrix3 & m, const Vector3 & v)
  {
    return {
      m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
      m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
      m[6] * v[0] + m[7] * v[1] + m[8] * v[2]};
  }

  static Vector3 add(const Vector3 & a, const Vector3 & b)
  {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
  }

  static Vector3 transformPoint(const Matrix3 & rotation, const Vector3 & translation, const Vector3 & point)
  {
    return add(multiply(rotation, point), translation);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_position_ = {
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z};

    const auto & q = msg->pose.pose.orientation;
    latest_rotation_ = rotationMatrixFromQuaternion(q.x, q.y, q.z, q.w);
    has_odom_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!has_odom_) {
      if (!waiting_for_odom_logged_) {
        waiting_for_odom_logged_ = true;
        RCLCPP_WARN(get_logger(), "Waiting for odometry before transforming lidar points");
      }
      return;
    }

    if (waiting_for_odom_logged_) {
      waiting_for_odom_logged_ = false;
      RCLCPP_INFO(get_logger(), "Received odometry, lidar point cloud transformation is active");
    }

    sensor_msgs::msg::PointCloud2 output_msg;
    output_msg.header = msg->header;
    output_msg.header.frame_id = output_frame_id_;
    output_msg.height = 1;
    output_msg.is_bigendian = false;
    output_msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(output_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    const size_t point_count = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    modifier.resize(point_count);

    size_t valid_point_count = 0;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
      sensor_msgs::PointCloud2Iterator<float> out_x(output_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> out_y(output_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> out_z(output_msg, "z");

      for (size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z) {
        const double x = *iter_x;
        const double y = *iter_y;
        const double z = *iter_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        const Vector3 local_point{x, y, z};
        const Vector3 point_in_base = transformPoint(lidar_rotation_, lidar_translation_, local_point);
        const Vector3 point_in_world = transformPoint(latest_rotation_, latest_position_, point_in_base);

        *out_x = static_cast<float>(point_in_world[0]);
        *out_y = static_cast<float>(point_in_world[1]);
        *out_z = static_cast<float>(point_in_world[2]);
        ++out_x;
        ++out_y;
        ++out_z;
        ++valid_point_count;
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Failed to read point cloud fields x/y/z: %s",
        ex.what());
      return;
    }

    if (valid_point_count == 0) {
      return;
    }

    modifier.resize(valid_point_count);
    cloud_pub_->publish(output_msg);
  }

  std::string input_cloud_topic_{"/lidar_points"};
  std::string output_cloud_topic_{"/lidar_points_world"};
  std::string odom_topic_{"/ego/odom_world"};
  std::string output_frame_id_{"world"};

  Vector3 lidar_translation_{0.12, 0.0, 0.315};
  Matrix3 lidar_rotation_{rotationMatrixFromRpy(0.0, 0.0, 0.0)};
  Vector3 latest_position_{0.0, 0.0, 0.0};
  Matrix3 latest_rotation_{rotationMatrixFromRpy(0.0, 0.0, 0.0)};
  bool has_odom_{false};
  bool waiting_for_odom_logged_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPointsToWorld>());
  rclcpp::shutdown();
  return 0;
}
