#include <memory>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "sensor_msgs/PointCloud2.h"

using TopicType = sensor_msgs::PointCloud2;

class PointCloudDelayCheckerNodelet : public nodelet::Nodelet{
public:
  virtual void onInit();

private:
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  void callback(const TopicType & msg);
};

void PointCloudDelayCheckerNodelet::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  sub_ = nh_.subscribe("input_topic", 1, &PointCloudDelayCheckerNodelet::callback, this, ros::TransportHints().tcpNoDelay(true));
}

void PointCloudDelayCheckerNodelet::callback(const TopicType & msg)
{
  double delay = (ros::Time::now() - msg.header.stamp).toSec();
  ROS_INFO_STREAM("delay " << delay);
}

PLUGINLIB_EXPORT_CLASS(PointCloudDelayCheckerNodelet, nodelet::Nodelet)
