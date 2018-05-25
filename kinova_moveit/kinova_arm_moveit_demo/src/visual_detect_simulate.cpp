#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include "kinova_arm_moveit_demo/targetsVector.h"	//自定义消息类型，所有识别定位结果
#include "kinova_arm_moveit_demo/targetState.h"	//自定义消息类型，单个识别定位结果

int flag=0;

void detect_target_CB(const std_msgs::Int8 &msg)
{
	if(msg.data==1)
	{
		flag=1;
	}
	if(msg.data==2)
    {
		flag=2;
	}
    else
	{
		flag=0;
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "our_pick_place");
	ros::NodeHandle node_handle;

	ros::Publisher detect_result_pub = node_handle.advertise<kinova_arm_moveit_demo::targetsVector>("detect_result",10);
  
	ros::Subscriber detect_target_sub = node_handle.subscribe("detect_target",10,detect_target_CB);

	kinova_arm_moveit_demo::targetsVector target_vec;
    kinova_arm_moveit_demo::targetState target_sta;
    target_sta.tag=9;
	target_sta.x=0.3;
	target_sta.y=0.6;
	target_sta.z=0.1;
	for(int i=9;i<=0;i--)
	{
		target_vec.targets.push_back(target_sta);
		target_sta.tag--;
	}
	


	while(flag!=2)
	{
		if(flag==1)		//模拟发送检测结果
		{
			flag=0;
			detect_result_pub.publish(target_vec);
			ros::Duration(1.0).sleep();
			target_vec.targets.pop_back();
		}
		ros::Duration(0.5).sleep();
	}
  

	return 0;
}
