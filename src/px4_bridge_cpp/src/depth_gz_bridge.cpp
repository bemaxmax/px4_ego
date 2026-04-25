#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class DepthGzBridge : public rclcpp::Node
{
public:
  DepthGzBridge()
  : Node("depth_img_transfer")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/depth_camera");
    output_topic_ = declare_parameter<std::string>("output_topic", "/depth_camera_bestef");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_link");

    auto sensor_qos = rclcpp::SensorDataQoS();
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      sensor_qos,
      std::bind(&DepthGzBridge::depthCallback, this, std::placeholders::_1));
    publisher_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, sensor_qos);
  }

private:
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    sensor_msgs::msg::Image out;
    out.header.stamp = msg->header.stamp;
    if (out.header.stamp.sec == 0 && out.header.stamp.nanosec == 0) {
      out.header.stamp = get_clock()->now();
    }
    out.header.frame_id = frame_id_;
    out.height = msg->height;
    out.width = msg->width;
    out.encoding = "32FC1";
    out.is_bigendian = msg->is_bigendian;
    out.step = msg->width * sizeof(float);
    out.data = msg->data;
    publisher_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthGzBridge>());
  rclcpp::shutdown();
  return 0;
}
