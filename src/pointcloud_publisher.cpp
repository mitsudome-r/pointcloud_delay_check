#include <cmath>

// PCL
#include "pcl_conversions/pcl_conversions.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloudPublisher : public rclcpp::Node{
public:
  PointCloudPublisher(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  sensor_msgs::msg::PointCloud2 msg_;
  void onTimer();
};

PointCloudPublisher::PointCloudPublisher(const rclcpp::NodeOptions & node_options)
: Node("pointcloud_publisher", node_options)
{
  const double rate = this->declare_parameter("rate", 10.0);
  const int width = this->declare_parameter("width", 700);
  const int length = this->declare_parameter("length", 1000);

  // create pointcloud message
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (int x = 0; x < width; x++)
  {
    for (int y = 0; y < length; y++)
    {
      pcl::PointXYZI tmp_point;
      tmp_point.x = static_cast<float>(x) / width * 100.0;
      tmp_point.y = static_cast<float>(y) / length * 100.0;
      tmp_point.z = 0.0;
      tmp_point.intensity = 1.0;
      cloud.push_back(tmp_point);
    }
  }

  pcl::toROSMsg(cloud, msg_);
  msg_.header.frame_id = "map";

  // set timer
  auto timer_callback = std::bind(&PointCloudPublisher::onTimer, this);
  double period_s = 1.0 / rate;
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", rclcpp::SensorDataQoS());
}

void PointCloudPublisher::onTimer()
{
  msg_.header.stamp = this->now();
  pub_->publish(msg_);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudPublisher)
