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
#include "rviz_teleop_commander/targets_tag.h"		//自定义消息类型，传递要抓取的目标标签 qcrong20180430
#include "rviz_teleop_commander/grab_result.h"		//自定义消息类型，传递当前抓取的目标标签和抓取次数 qcrong20180430

using namespace std;

//手指client类型自定义
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> Finger_actionlibClient;

//全局变量
const double FINGER_MAX = 6400;	//手指开合程度：0完全张开，6400完全闭合
const int n_MAX=10;			//同一物品最大抓取次数
vector<kinova_arm_moveit_demo::targetState> targets;	//视觉定位结果
bool getTargets=0;	//当接收到视觉定位结果时getTargets置1，执行完放置后置0
geometry_msgs::Pose placePose;	//机械臂抓取放置位置,为规划方便，将放置位置设为起始位置
//moveit::planning_interface::MoveGroup arm_group("arm");//改为全局变量，方便机械臂运动规划的使用
vector<int> targetsTag;		//需要抓取的目标物的标签
bool getTargetsTag=0;	//当接收到需要抓取的目标物的标签时置1，等待结束后置0

//定义机器人类型，手指控制 added by yang 20180418
string kinova_robot_type = "j2s7s300";
string Finger_action_address = "/" + kinova_robot_type + "_driver/fingers_action/finger_positions";    //手指控制服务器的名称

//定义手指控制client added by yang 20180418
Finger_actionlibClient* client=NULL;

//输入函数，接收需要抓取的目标标签,如果标签数为0，则返回false
//bool getTags();
//接收到detect_result消息的回调函数，将消息内容赋值到全局变量targets里面
void detectResultCB(const kinova_arm_moveit_demo::targetsVector &msg);
//接收targets_tag消息的回调函数，将接收到的消息更新到targetsTag里面
void tagsCB(const rviz_teleop_commander::targets_tag &msg);
//如果当前待抓取目标存在返回1,并且更新curTargetPoint，如果当前目标不存在但还有需要抓取的目标返回2，如果全部抓完或者一个目标物都没有返回3
int haveGoal(const vector<int>& targetsTag, const int& cur_target, kinova_arm_moveit_demo::targetState& curTargetPoint);
//手抓控制函数，输入0-1之间的控制量，控制手抓开合程度，0完全张开，1完全闭合
bool fingerControl(double finger_turn);
//机械臂运动控制函数
void pickAndPlace(kinova_arm_moveit_demo::targetState curTargetPoint,
                  moveit::planning_interface::MoveGroup arm_group);
//抓取插值函数
std::vector<geometry_msgs::Pose> pickInterpolate(geometry_msgs::Pose startPose,geometry_msgs::Pose targetPose);
//放置插值函数
std::vector<geometry_msgs::Pose> placeInterpolate(geometry_msgs::Pose startPose,geometry_msgs::Pose targetPose);
//设置机械臂放置位置,经多次测试，发现必须用函数设置初始位置．
void setPlacePose();

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

    client = new Finger_actionlibClient(Finger_action_address, true);

	//实物控制的话，可删掉这两句－－删掉是为了减少Rviz的使用所占用的时间
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

    //仿真连不上，程序无法继续执行，先注释掉
/*
    while(ros::ok() && !client->waitForServer(ros::Duration(5.0)))
	{
     ROS_INFO("Waiting for the finger action server to come up");
  	}
*/
	//发布消息和订阅消息
	ros::Publisher detectTarget_pub = node_handle.advertise<std_msgs::Int8>("detect_target", 10);  //让visual_detect节点检测目标
	ros::Publisher grab_result_pub = node_handle.advertise<rviz_teleop_commander::grab_result>("grab_result", 1);  //发布抓取状态

	ros::Subscriber detectResult_sub = node_handle.subscribe("detect_result", 10, detectResultCB);				//接收visual_detect检测结果
	ros::Subscriber tags_sub = node_handle.subscribe("targets_tag", 1, tagsCB);				//接收要抓取的目标 qcrong
    
	//ros::Duration(0.3).sleep();

	int n=0;		//记录对同一目标抓取的次数
	std_msgs::Int8 detectTarget;
	
	/*************************************/
	/********目标输入*********************/
	/*************************************/
	int cur_target=0;					//当前抓取目标的序号
    ROS_INFO("waiting for tags of targets input in GUI");
    while(getTargetsTag!=1)				//等待抓取目标输入
	{
		ros::Duration(0.5).sleep();
		if(!ros::ok())
		{
			detectTarget.data=2;		//让visual_detect节点退出
			detectTarget_pub.publish(detectTarget);
			ros::Duration(1.0).sleep();
			return 1;
		}
	}
	getTargetsTag=0;	//等待完毕，getTargetsTag置0
/*
  	if(!getTags())	//如果没有标签输入则程序直接退出
	{
		detectTarget.data=2;		//让visual_detect节点退出
		detectTarget_pub.publish(detectTarget);
		ros::Duration(1.0).sleep();
		return 1;
	}
*/
	/*************************************/
	/********目标抓取*********************/
	/*************************************/
    // 先前往放置位置
    setPlacePose();
    arm_group.setPoseTarget(placePose);
    arm_group.move();
    ROS_INFO("All ready, waiting for goal.");
	
	rviz_teleop_commander::grab_result grabResultMsg;	

    //等待目标传入并执行
	while(ros::ok())
	{
		if(getTargets==1)	//收到视觉检测结果
		{
			//判断当前抓取目标是否存在
			kinova_arm_moveit_demo::targetState curTargetPoint;    //当前抓取点的xyz,后续考虑加姿态
			int goalState=haveGoal(targetsTag,cur_target,curTargetPoint);
			if(goalState==1 && n<n_MAX)		//如果当前目标存在且抓取次数未达上限
			{
				n++;		//当前抓取次数+1
				//进行抓取放置，要求抓取放置后返回初始位置
				//周佩---机械臂运动控制---执行抓取－放置－过程
                pickAndPlace(curTargetPoint,arm_group);

				getTargets=0;		//执行完抓取置0，等待下一次视觉检测结果
				//让visual_detect节点进行检测
				detectTarget.data=1;		//让visual_detect节点进行视觉检测
				detectTarget_pub.publish(detectTarget);
				//发布抓取状态
				grabResultMsg.now_target=targetsTag[cur_target];
				grabResultMsg.grab_times=n;
				grab_result_pub.publish(grabResultMsg);
			}
			else if(goalState==2)			//当前目标不存在但还有需要抓取的目标
			{
				cur_target++;			//更新当前目标
				n=0;							//当前抓取次数置0
				//发布抓取状态
				grabResultMsg.now_target=targetsTag[cur_target];
				grabResultMsg.grab_times=n;
				grab_result_pub.publish(grabResultMsg);		
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
	//发布抓取状态
	grabResultMsg.now_target=-1;
	grabResultMsg.grab_times=-1;
	grab_result_pub.publish(grabResultMsg);
	ros::Duration(1.0).sleep();
	return 0;
}

/*************************************/
/********函数定义*********************/
/*************************************/

//输入函数，接收需要抓取的目标标签,如果标签数为0，则返回false
/*
bool getTags()
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
}*/

//接收到detect_result消息的回调函数，将消息内容赋值到全局变量targets里面
void detectResultCB(const kinova_arm_moveit_demo::targetsVector &msg)
{
	targets=msg.targets;
	getTargets=1;	//接收到视觉定位结果getTargets置1
}
//接收targets_tag消息的回调函数，将接收到的消息更新到targetsTag里面
void tagsCB(const rviz_teleop_commander::targets_tag &msg)
{
	int num=msg.targetsTag.size();
    targetsTag.clear();
  	targetsTag.resize(num);
	//int i=0;
	ROS_INFO("tags of targets[%d] :",num);
	for(int i=0; i<num; i++)
	{
		targetsTag[i]=msg.targetsTag[i];
		ROS_INFO(" [%d]", targetsTag[i]);
	}
	getTargetsTag=1;	//接收到需要抓取的目标物的标签
}

//如果当前待抓取目标存在返回1,并且更新curTargetPoint，如果当前目标不存在但还有需要抓取的目标返回2，如果全部抓完或者一个目标物都没有返回3
int haveGoal(const vector<int>& targetsTag, const int& cur_target, kinova_arm_moveit_demo::targetState& curTargetPoint)
{
	int num=targets.size();		//检测到的物品的个数
	if(cur_target>=targetsTag.size() || num==0 )		//全部抓完或者一个目标物都没有返回3
	{
		return 3;
	}
	for(int i=0;i<num;i++)
	{
		if(targets[i].tag==targetsTag[cur_target])
		{
			curTargetPoint=targets[i];		//获取当前抓取物品的位置
			return 1;
		}
	}
	return 2;
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
    client->sendGoal(goal);
    if (client->waitForResult(ros::Duration(5.0)))
    {
        client->getResult();
        return true;
    }
    else
    {
        client->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }
}

void pickAndPlace(kinova_arm_moveit_demo::targetState curTargetPoint,
                  moveit::planning_interface::MoveGroup arm_group)
{
//流程介绍
//1--获得目标点并对路径进行插值
//2--执行插值后的路径
//3--到达目标点抓取物体
//4--从目标点到放置点进行插值
//5--执行插值后的路径
//6--放置物体
//7--等待下一个目标点
    
    geometry_msgs::Pose targetPose;	//定义抓取位姿
    geometry_msgs::Point point;
    geometry_msgs::Quaternion orientation;
                
    point.x = curTargetPoint.x;//获取抓取位姿
    point.y = curTargetPoint.y;
    point.z = curTargetPoint.z;//这里等待实验测量结果－－－－－－－－－－－－－－－－－－修改为固定值－－－－－－周佩

    moveit::planning_interface::MoveGroup::Plan pick_plan;
    moveit::planning_interface::MoveGroup::Plan place_plan;

    orientation.x = 1;//方向由视觉节点给定－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－Petori
    orientation.y = 0;
    orientation.z = 0;
    orientation.w = 0;

    targetPose.position = point;// 设置好目标位姿为可用的格式
    targetPose.orientation = orientation;

    //获取当前位姿
    arm_group.setStartState(*arm_group.getCurrentState());

    //抓取插值
    std::vector<geometry_msgs::Pose> pickWayPoints;
    pickWayPoints = pickInterpolate(placePose, targetPose);

    //前往抓取点
    moveit_msgs::RobotTrajectory trajectory1;
    arm_group.computeCartesianPath(pickWayPoints,
                                   0.01,  // eef_step
                                   0.0,   // jump_threshold
                                   trajectory1);

    pick_plan.trajectory_ = trajectory1;
    arm_group.execute(pick_plan);

    double tPlan1 = arm_group.getPlanningTime();
    ROS_INFO("Planning time is [%lf]s.", tPlan1);
    ROS_INFO("Go to the goal and prepare for picking .");

    //抓取动作
    fingerControl(0);
    //抓取完毕

    arm_group.setStartState(*arm_group.getCurrentState());
    //放置插值
    std::vector<geometry_msgs::Pose> placeWayPoints;
    placeWayPoints = placeInterpolate(targetPose, placePose);

    //前往放置点
    moveit_msgs::RobotTrajectory trajectory2;
    arm_group.computeCartesianPath(placeWayPoints,
                                   0.01,  // eef_step
                                   0.0,   // jump_threshold
                                   trajectory2);
    place_plan.trajectory_ = trajectory2;
    arm_group.execute(place_plan);

    double tPlan2 = arm_group.getPlanningTime();
    ROS_INFO("Planning time is [%lf]s.", tPlan2);
    ROS_INFO("Go to the goal and prepare for placing . ");

    //松开爪子
    fingerControl(1);
    //松开完毕

}
//抓取插值函数
std::vector<geometry_msgs::Pose> pickInterpolate(geometry_msgs::Pose startPose,geometry_msgs::Pose targetPose)
{
    //从放置位置前往抓取位置
    //插值后路径为＂----|＂形（先平移，后下落）
    std::vector<geometry_msgs::Pose> pickWayPoints;

    geometry_msgs::Pose midPose1;
    geometry_msgs::Pose midPose2;
    geometry_msgs::Pose midPose3;
    geometry_msgs::Pose midPose4;

    geometry_msgs::Point startPoint;
    geometry_msgs::Point targetPoint;
    geometry_msgs::Point midPoint;

    startPoint = startPose.position;
    targetPoint = targetPose.position;

    // midPose1
    midPoint.x = startPoint.x + 0.25 * (targetPoint.x - startPoint.x);
    midPoint.y = startPoint.y + 0.25 * (targetPoint.y - startPoint.y);
    midPoint.z = startPoint.z;

    midPose1.position = midPoint;
    midPose1.orientation = targetPose.orientation;

    pickWayPoints.push_back(midPose1);

    // midPose2
    midPoint.x = startPoint.x + 0.5 * (targetPoint.x - startPoint.x);
    midPoint.y = startPoint.y + 0.5 * (targetPoint.y - startPoint.y);
    midPoint.z = startPoint.z;

    midPose2.position = midPoint;
    midPose2.orientation = targetPose.orientation;

    pickWayPoints.push_back(midPose2);

    // midPose3
    midPoint.x = startPoint.x + 0.75 * (targetPoint.x - startPoint.x);
    midPoint.y = startPoint.y + 0.75 * (targetPoint.y - startPoint.y);
    midPoint.z = startPoint.z;

    midPose3.position = midPoint;
    midPose3.orientation = targetPose.orientation;

    pickWayPoints.push_back(midPose3);

    // midPose4
    midPoint.x = targetPoint.x;
    midPoint.y = targetPoint.y;
    midPoint.z = targetPoint.z;

    midPose4.position = midPoint;
    midPose4.orientation = targetPose.orientation;

    pickWayPoints.push_back(midPose4);

    // Give targetPose
    pickWayPoints.push_back(targetPose);

    return pickWayPoints;

}

//放置插值函数
std::vector<geometry_msgs::Pose> placeInterpolate(geometry_msgs::Pose startPose,geometry_msgs::Pose targetPose)
{
    //从放置位置前往抓取位置
    //插值后路径为＂|----＂形（先抬升，后平移）
    std::vector<geometry_msgs::Pose> placeWayPoints;
    geometry_msgs::Pose midPose1;
    geometry_msgs::Pose midPose2;
    geometry_msgs::Pose midPose3;
    geometry_msgs::Pose midPose4;

    geometry_msgs::Point startPoint;
    geometry_msgs::Point targetPoint;
    geometry_msgs::Point midPoint;

    startPoint = startPose.position;
    targetPoint = targetPose.position;

    // midPose1
    midPoint.x = startPoint.x;
    midPoint.y = startPoint.y;
    midPoint.z = targetPoint.z;

    midPose1.position = midPoint;
    midPose1.orientation = targetPose.orientation;

    placeWayPoints.push_back(midPose1);

    // midPose2
    midPoint.x = startPoint.x + 0.25 * (targetPoint.x - startPoint.x);
    midPoint.y = startPoint.y + 0.25 * (targetPoint.y - startPoint.y);
    midPoint.z = targetPoint.z;

    midPose2.position = midPoint;
    midPose2.orientation = targetPose.orientation;

    placeWayPoints.push_back(midPose2);

    // midPose3
    midPoint.x = startPoint.x + 0.5 * (targetPoint.x - startPoint.x);
    midPoint.y = startPoint.y + 0.5 * (targetPoint.y - startPoint.y);
    midPoint.z = targetPoint.z;

    midPose3.position = midPoint;
    midPose3.orientation = targetPose.orientation;

    placeWayPoints.push_back(midPose3);

    // midPose4
    midPoint.x = startPoint.x + 0.75 * (targetPoint.x - startPoint.x);
    midPoint.y = startPoint.y + 0.75 * (targetPoint.y - startPoint.y);
    midPoint.z = targetPoint.z;

    midPose4.position = midPoint;
    midPose4.orientation = targetPose.orientation;

    placeWayPoints.push_back(midPose4);

    // Give targetPose
    placeWayPoints.push_back(targetPose);

    return placeWayPoints;
}
void setPlacePose()
{
placePose.position.x = 0.27;
placePose.position.y = 0.55;
placePose.position.z = 0.2;
placePose.orientation.x = 1;
placePose.orientation.y = 0;
placePose.orientation.z = 0;
placePose.orientation.w = 0;
}

