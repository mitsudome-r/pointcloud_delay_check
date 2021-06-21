#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"

using TopicType = sensor_msgs::msg::PointCloud2;

class PointCloudDelayChecker : public rclcpp::Node
{
public:
  explicit PointCloudDelayChecker(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<TopicType>::SharedPtr sub_;
  void callback(std::shared_ptr<TopicType> msg);
};

PointCloudDelayChecker::PointCloudDelayChecker(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("delay_checker", node_options)
{
  sub_ = create_subscription<TopicType>("input_topic", rclcpp::SensorDataQoS(),
    std::bind(&PointCloudDelayChecker::callback, this, std::placeholders::_1));
}

void PointCloudDelayChecker::callback(std::shared_ptr<TopicType> msg)
{
  double delay = (now() - rclcpp::Time(msg->header.stamp)).seconds();
  RCLCPP_INFO_STREAM(get_logger(), "delay " << delay);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudDelayChecker)
