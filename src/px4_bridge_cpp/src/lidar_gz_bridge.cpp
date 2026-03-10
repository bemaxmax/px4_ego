#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class LidarGzBridge : public rclcpp::Node
{
public:
  LidarGzBridge()
  : Node("lidar_points_transfer")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lidar_points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/lidar_points_bestef");
    default_frame_id_ = declare_parameter<std::string>("default_frame_id", "lidar_link");

    auto sensor_qos = rclcpp::SensorDataQoS();
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      sensor_qos,
      std::bind(&LidarGzBridge::lidarCallback, this, std::placeholders::_1));
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, sensor_qos);
  }

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 out = *msg;
    out.header.stamp = get_clock()->now();
    if (out.header.frame_id.empty()) {
      out.header.frame_id = default_frame_id_;
    }
    publisher_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string default_frame_id_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarGzBridge>());
  rclcpp::shutdown();
  return 0;
}
