#include <cmath>

// PCL
#include "pcl_conversions/pcl_conversions.h"

#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "sensor_msgs/PointCloud2.h"

class PointCloudPublisherNodelet : public nodelet::Nodelet{
public:
  virtual void onInit();

private:
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;

  ros::Timer timer_;
  ros::Publisher pub_;

  sensor_msgs::PointCloud2 msg_;
  void onTimer(const ros::TimerEvent&);
};

void PointCloudPublisherNodelet::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  double rate;
  int width, length;
  pnh_.param<double>("rate", rate, 10.0);
  pnh_.param<int>("width", width, 700);
  pnh_.param<int>("length", length, 1000);

  // create pointcloud message
  ROS_INFO_STREAM("creating pointcloud with " << width * length << " points.");
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
  timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &PointCloudPublisherNodelet::onTimer, this);
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
}

void PointCloudPublisherNodelet::onTimer(const ros::TimerEvent&)
{
  msg_.header.stamp = ros::Time::now();
  pub_.publish(msg_);
}

PLUGINLIB_EXPORT_CLASS(PointCloudPublisherNodelet, nodelet::Nodelet)
