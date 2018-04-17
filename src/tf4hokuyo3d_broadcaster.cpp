#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

std::string turtle_name;

void tfCallback(const sensor_msgs::PointCloud2 a_msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(M_PI, 0.0, 0.0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "hokuyo3d", "/hokuyo3d_pose"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf4hokuyo3d_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/hokuyo3d/hokuyo_cloud2", 10, &tfCallback);

  ros::spin();
  return 0;
};
