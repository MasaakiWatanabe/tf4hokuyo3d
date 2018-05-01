#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <nav_msgs/Odometry.h>

class TransformOdom{

public:

  ros::NodeHandle n_;
  ros::Subscriber scan_sub_;
  ros::Publisher scan_pub_;

  TransformOdom(ros::NodeHandle n):
    n_(n)
  {
    scan_sub_ = n_.subscribe("/icasr_mini/odom", 10, &TransformOdom::scanCallback, this);
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/odom_pose",1);
  }

  void scanCallback (const nav_msgs::Odometry odom_in)
  {
    sensor_msgs::PointCloud2 cloud2;
    try
    {
      ROS_INFO("Receive time=%u.%u", scan_in.header.stamp.sec, scan_in.header.stamp.nsec);

      tf_listener_.waitForTransform("/hokuyo3d", "/robot_pose", scan_in.header.stamp, ros::Duration(0.05));
      tf_listener_.transformPointCloud("hokuyo3d", scan_in.header.stamp, scan_in, "robot_pose", cloud);
      sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);

    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }

    scan_pub_.publish(cloud2);
  }
};

int main(int argc, char** argv)
{

  // 初期化のためのAPI
  ros::init(argc, argv, "tf4odom");
  // ノードハンドラの宣言
  ros::NodeHandle n;

  TransformOdom tf4odom_node(n);

  ROS_INFO("Loop Start.");

  ros::spin();

  ROS_INFO("Loop End.");

  return 0;
}
