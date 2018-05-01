#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class TransformHokuyo3dPointCloud{

public:

  ros::NodeHandle n_;
  tf::TransformListener listener_;
  ros::Publisher scan_pub_;
  ros::Subscriber scan_sub_;

  TransformHokuyo3dPointCloud(ros::NodeHandle n):
    n_(n)
  {
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/points_raw",1);
    scan_sub_ = n_.subscribe("/hokuyo3d/hokuyo_cloud", 10, &TransformHokuyo3dPointCloud::scanCallback, this);
  }

  void scanCallback (const sensor_msgs::PointCloud scan_in)
  {
    sensor_msgs::PointCloud msg1;
    sensor_msgs::PointCloud2 cloud;
    try
    {
      ROS_INFO("Receive time=%u.%u", scan_in.header.stamp.sec, scan_in.header.stamp.nsec);

      listener_.waitForTransform("/hokuyo3d", "/robot_pose", scan_in.header.stamp, ros::Duration(0.05));
      ROS_INFO("Callback123");
      listener_.transformPointCloud("hokuyo3d", scan_in.header.stamp, scan_in, "robot_pose", msg1);
      ROS_INFO("Callback4456");
      sensor_msgs::convertPointCloudToPointCloud2(msg1, cloud);
      ROS_INFO("Callback789");

    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }

    scan_pub_.publish(cloud);
  }
};

int main(int argc, char** argv)
{

  // 初期化のためのAPI
  ros::init(argc, argv, "tf4hokuyo3d");
  // ノードハンドラの宣言
  ros::NodeHandle n;

  TransformHokuyo3dPointCloud tf4hokuyo3d_node(n);

  ROS_INFO("Loop Start.");

  ros::spin();

  ROS_INFO("Loop End.");

  return 0;
}
