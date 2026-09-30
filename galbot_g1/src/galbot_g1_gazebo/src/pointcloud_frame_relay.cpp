// Republishes a PointCloud2 with a different frame_id.
//
// Gazebo stamps all streams of an RGB-D sensor with one frame. We give it the ROS
// optical frame (correct for images and camera_info), but its point cloud is expressed
// in the sensor body frame (x forward, z up), so the cloud is re-stamped here.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloudFrameRelay : public rclcpp::Node
{
public:
  PointCloudFrameRelay()
  : Node("pointcloud_frame_relay"),
    frame_id_(declare_parameter<std::string>("frame_id"))
  {
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("output", rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::UniquePtr cloud) {
        cloud->header.frame_id = frame_id_;
        publisher_->publish(std::move(cloud));
      });
  }

private:
  const std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFrameRelay>());
  rclcpp::shutdown();
  return 0;
}
