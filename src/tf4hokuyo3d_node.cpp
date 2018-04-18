#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

sensor_msgs::PointCloud2 msg;
sensor_msgs::PointCloud msg1;
tf::TransformListener listener;

class tflistener{
	private:
		tf::TransformListener listener_;

	public:
		ros::Subscriber sub;
		ros::NodeHandle n;

		// Subscribeする対象のトピックが更新されたら呼び出されるコールバック関数
		// 引数にはトピックにPublishされるメッセージの型と同じ型を定義する
		void tfCallback(const sensor_msgs::PointCloud a_msg)
		{
		    //tf::TransformListener listener;
		    tf::StampedTransform transform;
		    ROS_INFO("Callback");
		    try{
		      ROS_INFO("Receive time=%u.%u, width=%d", a_msg.header.stamp.sec, a_msg.header.stamp.nsec, msg.width);

		      //listener.lookupTransform("/hokuyo3d", "/robot_pose", a_msg.header.stamp, transform);
		      listener_.waitForTransform("/hokuyo3d", "/robot_pose", a_msg.header.stamp, ros::Duration(0.05));
		      ROS_INFO("Callback123");
		      listener_.transformPointCloud("hokuyo3d", a_msg.header.stamp, a_msg, "robot_pose", msg1);
		      ROS_INFO("Callback4456");
		      sensor_msgs::convertPointCloudToPointCloud2(msg1, msg);
		      //msg = msg1;
		      ROS_INFO("Callback789");

		    }
		    catch (tf::TransformException &ex) {
		      ROS_ERROR("%s",ex.what());
		      ros::Duration(0.1).sleep();
		    }
		      ROS_INFO("Callback end");
		}


		tflistener(){
			  sub = n.subscribe("/hokuyo3d/hokuyo_cloud", 10, tfCallback);
		};

		~tflistener();

};


int main(int argc, char **argv)
{
  // 初期化のためのAPI
  ros::init(argc, argv, "tf4hokuyo3d");
  // ノードハンドラの宣言
  //ros::NodeHandle n;
  //クラス生成
  tflistener tflistener_c;

  // Subscriberとしてpara_inputというトピックがSubscribeし、トピックが更新されたときは
  // chatterCallbackという名前のコールバック関数を実行する
  //ros::Subscriber sub = n.subscribe("/hokuyo3d/hokuyo_cloud", 10, tflistener_c.tfCallback);

  ros::Publisher pub = tflistener_c.n.advertise<sensor_msgs::PointCloud2>("point_raw", 10);

  ros::Rate loop_rate(200);


  ROS_INFO("MAIN LOOP");
  int count = 0;
  while (ros::ok())//ノードが実行中は基本的にros::ok()=1
  {
    ros::spinOnce();

    //printf("LOOP=%d\n", count);
	    ROS_INFO("LOOP");

    if(tflistener_c.sub){
	    ROS_INFO("pub");
      pub.publish(msg);//PublishのAPI
    }

    loop_rate.sleep();

    count++;
  }


  return 0;
}
