#include <memory>

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"

using TopicType = sensor_msgs::PointCloud2;

class PointCloudDelayChecker
{
public:
  explicit PointCloudDelayChecker();

private:
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  void callback(const TopicType & msg);
};

PointCloudDelayChecker::PointCloudDelayChecker() : pnh_("~")
{
  sub_ = nh_.subscribe("input_topic", 1, &PointCloudDelayChecker::callback, this, ros::TransportHints().tcpNoDelay(true));
}

void PointCloudDelayChecker::callback(const TopicType & msg)
{
  double delay = (ros::Time::now() - msg.header.stamp).toSec();
  ROS_INFO_STREAM("delay " << delay);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pointcloud_delay_check");
  PointCloudDelayChecker node;
  ros::spin();
  return 0;
}