#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include "kinova_arm_moveit_demo/targetsVector.h"	//自定义消息类型，所有识别定位结果
#include "kinova_arm_moveit_demo/targetState.h"	//自定义消息类型，单个识别定位结果

int flag=0;

void detect_target_CB(const std_msgs::Int8 &msg)
{
	//ROS_INFO("receive");
	if(msg.data==1)
	{
		flag=1;
		ROS_INFO("receive 1");
	}
	else if(msg.data==2)
    {
		flag=2;
		ROS_INFO("receive 2");
	}
    else
	{
		flag=0;
		ROS_INFO("receive 3");
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "visual_detect_simulate");
	ros::NodeHandle node_handle;

	ros::Publisher detect_result_pub = node_handle.advertise<kinova_arm_moveit_demo::targetsVector>("detect_result",10);
  
	ros::Subscriber detect_target_sub = node_handle.subscribe("detect_target",10,detect_target_CB);

	kinova_arm_moveit_demo::targetsVector target_vec;
    target_vec.targets.resize(10);
	for(int i=9;i>=0;i--)
	{
		target_vec.targets[i].tag=i;
		target_vec.targets[i].x=-0.27;
		target_vec.targets[i].y=0.55;
		target_vec.targets[i].z=0.03;
	}
	


	while(flag!=2 && ros::ok())
	{
		if(flag==1)		//模拟发送检测结果
		{
			flag=0;
			detect_result_pub.publish(target_vec);
			ros::Duration(1.0).sleep();
			target_vec.targets.pop_back();
			ROS_INFO("targets");
		}
		ros::Duration(0.5).sleep();
		ros::spinOnce();
	}
  

	return 0;
}
