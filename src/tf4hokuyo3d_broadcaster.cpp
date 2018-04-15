#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

sensor_msgs::PointCloud2 msg;


// Subscribeする対象のトピックが更新されたら呼び出されるコールバック関数
// 引数にはトピックにPublishされるメッセージの型と同じ型を定義する
void tfCallback(const sensor_msgs::PointCloud2 a_msg)
{
  msg = a_msg;
  printf("Receive width=%d\n", a_msg.width);
}

int main(int argc, char **argv)
{
  // 初期化のためのAPI
  ros::init(argc, argv, "tf4hokuyo3d");
  // ノードハンドラの宣言
  ros::NodeHandle n;

  // Subscriberとしてpara_inputというトピックがSubscribeし、トピックが更新されたときは
  // chatterCallbackという名前のコールバック関数を実行する
  ros::Subscriber sub = n.subscribe("hokuyo3d/point_cloud2", 10, tfCallback);

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
