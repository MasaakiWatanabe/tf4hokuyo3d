#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

sensor_msgs::PointCloud2 msg;
sensor_msgs::PointCloud msg1;


// Subscribeする対象のトピックが更新されたら呼び出されるコールバック関数
// 引数にはトピックにPublishされるメッセージの型と同じ型を定義する
void tfCallback(const sensor_msgs::PointCloud a_msg)
{
    tf::TransformListener listener;
    //tf::StampedTransform transform;
    try{
      //listener.lookupTransform("/hokuyo3d", "/robot_pose", ros::Time(0), transform);
      listener.transformPointCloud("hokuyo3d", a_msg.header.stamp, a_msg, "robot_pose", msg1);

      sensor_msgs::convertPointCloudToPointCloud2(msg1, msg);
      //msg = msg1;

      printf("Receive width=%d\n", msg.width);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.1).sleep();
    }

}

int main(int argc, char **argv)
{
  // 初期化のためのAPI
  ros::init(argc, argv, "tf4hokuyo3d");
  // ノードハンドラの宣言
  ros::NodeHandle n;

  // Subscriberとしてpara_inputというトピックがSubscribeし、トピックが更新されたときは
  // chatterCallbackという名前のコールバック関数を実行する
  ros::Subscriber sub = n.subscribe("/hokuyo3d/hokuyo_cloud", 10, tfCallback);

  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("point_raw", 10);

  ros::Rate loop_rate(200);


  int count = 0;
  while (ros::ok())//ノードが実行中は基本的にros::ok()=1
  {
    ros::spinOnce();

    //printf("LOOP=%d\n", count);

    if(sub){
      pub.publish(msg);//PublishのAPI
    }

    loop_rate.sleep();

    count++;
  }


  return 0;
}
