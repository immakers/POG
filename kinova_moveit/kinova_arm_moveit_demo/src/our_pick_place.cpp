#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <kinova_driver/kinova_ros_types.h>

#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>

#include <iostream>
#include <vector>

//手指控制头文件 add by yang 20180418
#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"

#include <std_msgs/Int8.h>
#include "kinova_arm_moveit_demo/targetsVector.h"	//自定义消息类型，所有识别定位结果
#include "kinova_arm_moveit_demo/targetState.h"	//自定义消息类型，单个识别定位结果

using namespace std;

//手指client类型自定义
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> Finger_actionlibClient;

//全局变量
const double FINGER_MAX = 6400;	//手指开合程度：0完全张开，6400完全闭合
const int n_MAX=10;			//同一物品最大抓取次数
vector<kinova_arm_moveit_demo::targetState> targets;	//视觉定位结果
bool getTargets=0;	//当接收到视觉定位结果时getTargets置1，执行完放置后置0

//定义机器人类型，手指控制 added by yang 20180418
std::string kinova_robot_type = "j2s7s300";
std::string Finger_action_address = "/" + kinova_robot_type + "_driver/fingers_action/finger_positions";    //手指控制服务器的名称

//定义手指控制client added by yang 20180418
Finger_actionlibClient client(Finger_action_address, true);

//输入函数，接收需要抓取的目标标签,如果标签数为0，则返回false
bool getTags(vector<int>& targetsTag);
//接收到detect_result消息的回调函数，将消息内容赋值到全局变量targets里面
void detectResultCB(const kinova_arm_moveit_demo::targetsVector &msg);
//如果当前待抓取目标存在返回1,并且更新curTargetPoint，如果当前目标不存在但还有需要抓取的目标返回2，如果全部抓完返回3
int haveGoal(const vector<int>& targetsTag, const int& cur_target, kinova_arm_moveit_demo::targetState& curTargetPoint);
//手抓控制函数，输入0-1之间的控制量，控制手抓开合程度，0完全张开，1完全闭合
bool fingerControl(double finger_turn);


int main(int argc, char **argv)
{
	/*************************************/
	/********初始化设置*******************/
	/*************************************/
	ros::init(argc, argv, "our_pick_place");
	ros::NodeHandle node_handle;  
	ros::AsyncSpinner spinner(1);
	spinner.start();
	//等待rviz启动，最后集成用一个launch文件启动时需要
	//ros::Duration(10.0).sleep();
	
	moveit::planning_interface::MoveGroup arm_group("arm");
	arm_group.setEndEffectorLink( "j2s7s300_end_effector");
	moveit::planning_interface::MoveGroup finger_group("gripper");

	//实物控制的话，可删掉这两句－－删掉是为了减少Rviz的使用所占用的时间
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;
        
        ROS_INFO("Waiting for action server to start.");
        client.waitForServer();
        ROS_INFO("Action server started, waiting for goal.");
	//发布消息和订阅消息
	ros::Publisher detectTarget_pub = node_handle.advertise<std_msgs::Int8>("dectet_target", 10);  //让visual_detect节点检测目标
	ros::Subscriber detectResult_sub = node_handle.subscribe("detect_result", 10, detectResultCB);				//接收visual_detect检测结果
	ros::Duration(0.3).sleep();

	int n=0;		//记录对同一目标抓取的次数
	std_msgs::Int8 detectTarget;
	
	/*************************************/
	/********目标输入*********************/
	/*************************************/
	vector<int> targetsTag;		//需要抓取的目标物的标签
	int cur_target=0;					//当前抓取目标的序号
  if(!getTags(targetsTag))	//如果标签输入为0则程序直接退出
	{
		detectTarget.data=2;		//让visual_detect节点退出
		detectTarget_pub.publish(detectTarget);
		ros::Duration(1.0).sleep();
		return 1;
	}

	/*************************************/
	/********目标抓取*********************/
	/*************************************/
	while(ros::ok())
	{
		if(getTargets==1)
		{
			//判断当前抓取目标是否存在
			kinova_arm_moveit_demo::targetState curTargetPoint;    //当前抓取点的xyz,后续考虑加姿态
			int goalState=haveGoal(targetsTag,cur_target,curTargetPoint);
			if(goalState==1 && n<n_MAX)		//当前如果目标存在且抓取次数未达上限
			{
				n++;		//当前抓取次数+1
				//进行抓取放置，要求抓取放置后返回初始位置
				//周佩

				getTargets=0;		//执行完抓取置0，等待下一次视觉检测结果
				//让visual_detect节点进行检测
				detectTarget.data=1;		//让visual_detect节点退出
				detectTarget_pub.publish(detectTarget);
			}
			else if(goalState==2)			//当前目标不存在但还有需要抓取的目标
			{
				cur_target++;			//更新当前目标
				n=0;							//当前抓取次数置0		
			}
			else if(goalState==3)			//所有目标抓取完成
			{
				break;
			}
		}
		ros::Duration(0.2).sleep();
	}
	
	//退出程序
	detectTarget.data=2;		//让visual_detect节点退出
	detectTarget_pub.publish(detectTarget);
	ros::Duration(1.0).sleep();
	return 0;
}

/*************************************/
/********函数定义*********************/
/*************************************/

//输入函数，接收需要抓取的目标标签,如果标签数为0，则返回false
bool getTags(vector<int>& targetsTag)
{
	int targetNum=0;
	cin>>targetNum;
	for(int i=0; i<targetNum; i++)
	{
		int tag=0;
		cin>>tag;
		targetsTag.push_back(tag);
	}
	if (targetNum==0 || targetsTag.size()==0) return false;
	else return true;
}

//接收到detect_result消息的回调函数，将消息内容赋值到全局变量targets里面
void detectResultCB(const kinova_arm_moveit_demo::targetsVector &msg)
{
	targets=msg.targets;
	getTargets=1;	//接收到视觉定位结果getTargets置1
}

//如果当前待抓取目标存在返回1,并且更新curTargetPoint，如果当前目标不存在但还有需要抓取的目标返回2，如果全部抓完返回3
int haveGoal(const vector<int>& targetsTag, const int& cur_target, kinova_arm_moveit_demo::targetState& curTargetPoint)
{

	return true;
}

//手抓控制函数，输入0-1之间的控制量，控制手抓开合程度，0完全张开，1完全闭合 added by yang 20180418
bool fingerControl(double finger_turn)
{
    if (finger_turn < 0)
    {
        finger_turn = 0.0;
    }
    else
    {
        finger_turn = std::min(finger_turn, 1.0);
    }
    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = finger_turn * FINGER_MAX;
    goal.fingers.finger2 = goal.fingers.finger1;
    goal.fingers.finger3 = goal.fingers.finger1;
    client.sendGoal(goal);
    if (client.waitForResult(ros::Duration(5.0)))
    {
        client.getResult();
        return true;
    }
    else
    {
        client.cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }
}


