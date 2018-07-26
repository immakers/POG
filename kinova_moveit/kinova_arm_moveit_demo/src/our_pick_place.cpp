//#include <ros/ros.h>
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
#include <Eigen/Eigen>
#include "robotiq_c_model_control/gripperControl.h"  //robotiq二指手

#define Simulation 1     //仿真为1，实物为0
#define UR5		//使用ur5

using namespace std;
using namespace Eigen;

//手指client类型自定义
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> Finger_actionlibClient;

//全局变量
const double FINGER_MAX = 6400;	//手指开合程度：0完全张开，6400完全闭合
const int n_MAX=10;			//同一物品最大抓取次数
vector<kinova_arm_moveit_demo::targetState> targets;	//视觉定位结果
bool getTargets=0;	//当接收到视觉定位结果时getTargets置1，执行完放置后置0
geometry_msgs::Pose placePose;	//机械臂抓取放置位置,为规划方便，将放置位置设为起始位置
//moveit::planning_interface::MoveGroup arm_group("manipulator");//改为全局变量，方便机械臂运动规划的使用
vector<int> targetsTag;		//需要抓取的目标物的标签
bool getTargetsTag=0;	//当接收到需要抓取的目标物的标签时置1，等待结束后置0

//定义机器人类型，手指控制 added by yang 20180418
string kinova_robot_type = "j2s7s300";
string Finger_action_address = "/" + kinova_robot_type + "_driver/fingers_action/finger_positions";    //手指控制服务器的名称

//定义手指控制client added by yang 20180418
Finger_actionlibClient* client=NULL;

//手眼关系
Eigen::Matrix3d base2eye_r;
Eigen::Vector3d base2eye_t;

//输入函数，接收需要抓取的目标标签,如果标签数为0，则返回false
//bool getTags();
//接收到detect_result消息的回调函数，将消息内容赋值到全局变量targets里面
void detectResultCB(const kinova_arm_moveit_demo::targetsVector &msg);
//接收targets_tag消息的回调函数，将接收到的消息更新到targetsTag里面
void tagsCB(const rviz_teleop_commander::targets_tag &msg);

//如果当前待抓取目标存在返回1,并且更新curTargetPoint，如果当前目标不存在但还有需要抓取的目标,或者抓取次数达到上限返回2，如果全部抓完或者一个目标物都没有返回3
int haveGoal(const vector<int>& targetsTag, const int& cur_target, kinova_arm_moveit_demo::targetState& curTargetPoint, const int& n);

//手抓控制函数，输入0-1之间的控制量，控制手抓开合程度，0完全张开，1完全闭合
bool fingerControl(double finger_turn);
//机械臂运动控制函数
void pickAndPlace(kinova_arm_moveit_demo::targetState curTargetPoint);
//抓取插值函数
std::vector<geometry_msgs::Pose> pickInterpolate(geometry_msgs::Pose startPose,geometry_msgs::Pose targetPose);
//放置插值函数
std::vector<geometry_msgs::Pose> placeInterpolate(geometry_msgs::Pose startPose,geometry_msgs::Pose targetPose);
//设置机械臂放置位置,经多次测试，发现必须用函数设置初始位置．
void setPlacePose();
//前往放置位置
void goPlacePose(geometry_msgs::Pose placePose);
// 转换位姿，用于控制UR实物
geometry_msgs::Pose changePoseForUR(geometry_msgs::Pose pose);

int main(int argc, char **argv)
{
	/*************************************/
	/********初始化设置*******************/
	/*************************************/
	ros::init(argc, argv, "our_pick_place");
	ros::NodeHandle node_handle;  
	ros::AsyncSpinner spinner(3);
	spinner.start();
	//等待rviz启动，最后集成用一个launch文件启动时需要
	//ros::Duration(10.0).sleep();

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
	ros::Publisher gripperPub = node_handle.advertise<robotiq_c_model_control::CModel_robot_output>("/CModelRobotOutput", 20); //robotiq手抓控制

	ros::Subscriber detectResult_sub = node_handle.subscribe("detect_result", 1, detectResultCB);    //接收visual_detect检测结果
	ros::Subscriber tags_sub = node_handle.subscribe("targets_tag", 1, tagsCB);				//接收要抓取的目标 qcrong
    
	//ros::Duration(0.3).sleep();

	int n=0;		//记录对同一目标抓取的次数
	std_msgs::Int8 detectTarget;
	//手眼关系赋值
	//手眼关系
	base2eye_r<<0.9972514277037777, -0.04687474255096129, 0.05737898965264326,
                -0.05950762379290415, -0.9681063256948148, 0.2433700574438165, 
                0.04414105405740991, -0.2461156245760625, -0.9682348200133247;

	base2eye_t<<-0.00657070285772457,-0.8888390237608956,0.5953071046884454;
	
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
	detectTarget.data=1;		//让visual_detect节点检测目标
	detectTarget_pub.publish(detectTarget);
	ros::Duration(1.0).sleep();


	/*************************************/
	/********目标抓取*********************/
	/*************************************/
    // 设置放置位置
    setPlacePose();
    // 前往放置位置
    goPlacePose(placePose);
#ifdef UR5
	gripperPubPtr = &gripperPub;
	initializeGripperMsg();

#endif
    ROS_INFO("All ready, waiting for goal.");
	
	rviz_teleop_commander::grab_result grabResultMsg;	

    //等待目标传入并执行
	while(ros::ok())
	{
		if(getTargets==1)	//收到视觉检测结果
		{
			//判断当前抓取目标是否存在
			kinova_arm_moveit_demo::targetState curTargetPoint;    //当前抓取点的xyz,后续考虑加姿态
			int goalState=haveGoal(targetsTag,cur_target,curTargetPoint,n);
			if(goalState==1 && n<n_MAX)		//如果当前目标存在且抓取次数未达上限
			{
				n++;		//当前抓取次数+1

				//发布抓取状态
				grabResultMsg.now_target=targetsTag[cur_target];
				grabResultMsg.grab_times=n;
				grab_result_pub.publish(grabResultMsg);

				//进行抓取放置，要求抓取放置后返回初始位置
				//周佩---机械臂运动控制---执行抓取－放置－过程
                pickAndPlace(curTargetPoint);
				
				getTargets=0;		//执行完抓取置0，等待下一次视觉检测结果
				//让visual_detect节点进行检测
				detectTarget.data=1;		//让visual_detect节点进行视觉检测
				detectTarget_pub.publish(detectTarget);
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
	int num = msg.targets.size();
    targets.clear();
	targets.resize(num);
	for(int i=0; i<num; i++)
	{
		targets[i]=msg.targets[i];
		//ROS_INFO("%d",msg.targets[i].tag);
		//ROS_INFO("%f %f %f",msg.targets[i].x,msg.targets[i].y,msg.targets[i].z);
	}
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


//如果当前待抓取目标存在返回1,并且更新curTargetPoint，如果当前目标不存在但还有需要抓取的目标,或者抓取次数达到上限返回2，如果全部抓完或者一个目标物都没有返回3
int haveGoal(const vector<int>& targetsTag, const int& cur_target, kinova_arm_moveit_demo::targetState& curTargetPoint,const int& n)
{
	int num=targets.size();		//检测到的物品的个数
	ROS_INFO("size of goal = %d",num);
	if(cur_target>=targetsTag.size() || num==0 )		//全部抓完或者一个目标物都没有返回3
	{
		ROS_INFO("have goal 3");
		return 3;
	}
  
  	if(n>=n_MAX)   		//抓取次数达到上限
  	{
		ROS_INFO("have goal 2");
     	return 2;
  	}

	for(int i=0;i<num;i++)
	{
		if(targets[i].tag==targetsTag[cur_target] && n<n_MAX)
		{
			//目标物在相机坐标系下的坐标转机器人坐标系下的坐标
			Eigen::Vector3d cam_center3d, base_center3d;
			cam_center3d(0)=targets[i].x;
			cam_center3d(1)=targets[i].y;
			cam_center3d(2)=targets[i].z;
			base_center3d=base2eye_r*cam_center3d+base2eye_t;
			//获取当前抓取物品的位置
			curTargetPoint.x=base_center3d(0);
            curTargetPoint.y=base_center3d(1);
			curTargetPoint.z=base_center3d(2);		
			ROS_INFO("have goal 1");
			ROS_INFO("%d",targets[i].tag);
            //ROS_INFO("%f %f %f",cam_center3d(0),cam_center3d(1),cam_center3d(2));
			ROS_INFO("%f %f %f",curTargetPoint.x,curTargetPoint.y,curTargetPoint.z);
			return 1;
		}
	}
	ROS_INFO("have goal");
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

void pickAndPlace(kinova_arm_moveit_demo::targetState curTargetPoint)
{
//流程介绍
//1--获得目标点并对路径进行插值
//2--执行插值后的路径
//3--到达目标点抓取物体
//4--从目标点到放置点进行插值
//5--执行插值后的路径
//6--放置物体
//7--等待下一个目标点
#ifdef UR5
    moveit::planning_interface::MoveGroup arm_group("manipulator");	//manipulator
#else
    moveit::planning_interface::MoveGroup arm_group("arm");	//manipulator
    moveit::planning_interface::MoveGroup *finger_group;
    finger_group = new moveit::planning_interface::MoveGroup("gripper");
#endif
    geometry_msgs::Pose targetPose;	//定义抓取位姿
    geometry_msgs::Point point;
    geometry_msgs::Quaternion orientation;
 
#ifdef UR5
	//ur5手爪的动作
	sendGripperMsg(0);//open

#else
    //抓取动作
    if(Simulation)
    {
        finger_group->setNamedTarget("Open");   //仿真使用
		finger_group->move();
    }
    else if(!Simulation)
    {
        fingerControl(0.1);               //实物，Simulation宏改为0
    }
    //抓取完毕
#endif
                

    point.x = curTargetPoint.x;//获取抓取位姿
    point.y = curTargetPoint.y;
    //point.z = curTargetPoint.z;//这里等待实验测量结果－－－－－－－－－－－－－－－－－－修改为固定值－－－－－－周佩
	point.z =0.19;

    moveit::planning_interface::MoveGroup::Plan pick_plan;
    moveit::planning_interface::MoveGroup::Plan place_plan;

    orientation.x = 1;//方向由视觉节点给定－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－Petori
    orientation.y = 0;
    orientation.z = 0;
    orientation.w = 0;

    targetPose.position = point;// 设置好目标位姿为可用的格式
    targetPose.orientation = orientation;

    // if we use ur for experiment-----------------------------------------------------------------------------------------for ur
#ifdef UR5
    geometry_msgs::Pose pose;
    pose = targetPose;
    targetPose = changePoseForUR(pose);
#endif

    //抓取插值
    std::vector<geometry_msgs::Pose> pickWayPoints;
    pickWayPoints = pickInterpolate(placePose, targetPose);

    //前往抓取点
    moveit_msgs::RobotTrajectory trajectory1;
    arm_group.computeCartesianPath(pickWayPoints,
                                   0.02,  // eef_step
                                   0.0,   // jump_threshold
                                   trajectory1);

    pick_plan.trajectory_ = trajectory1;
    arm_group.execute(pick_plan);

    double tPlan1 = arm_group.getPlanningTime();
    ROS_INFO("Planning time is [%lf]s.", tPlan1);
    ROS_INFO("Go to the goal and prepare for picking .");

#ifdef UR5
    //ur5手抓的动作
	sendGripperMsg(155);//close
#else
    //抓取动作
    if(Simulation)
    {
        finger_group->setNamedTarget("Close");   //仿真使用
		finger_group->move();
    }
    else if(!Simulation)
    {
        fingerControl(0.9);               //实物，Simulation宏改为0
    }
    //抓取完毕
#endif

    //放置插值
    std::vector<geometry_msgs::Pose> placeWayPoints;
    placeWayPoints = placeInterpolate(targetPose, placePose);

    //前往放置点
    moveit_msgs::RobotTrajectory trajectory2;
    arm_group.computeCartesianPath(placeWayPoints,
                                   0.02,  // eef_step
                                   0.0,   // jump_threshold
                                   trajectory2);
    place_plan.trajectory_ = trajectory2;
    arm_group.execute(place_plan);

    double tPlan2 = arm_group.getPlanningTime();
    ROS_INFO("Planning time is [%lf]s.", tPlan2);
    ROS_INFO("Go to the goal and prepare for placing . ");

#ifdef UR5
    //ur5手抓的动作
	sendGripperMsg(0);//open
#else

    //松开爪子
    if(Simulation)
    {
    	finger_group->setNamedTarget("Open");   //仿真使用
		finger_group->move();
    }
    else if(!Simulation)
    {
        fingerControl(0.1);              //实物，Simulation宏改为0
    }
    //松开完毕
#endif

    ROS_INFO("Waiting for the next goal.");
}
//抓取插值函数
std::vector<geometry_msgs::Pose> pickInterpolate(geometry_msgs::Pose startPose,geometry_msgs::Pose targetPose)
{
    //从放置位置前往抓取位置
    //插值后路径为＂----|＂形（先平移，后下落）
    std::vector<geometry_msgs::Pose> pickWayPoints;

    geometry_msgs::Pose midPose4;

    geometry_msgs::Point startPoint;
    geometry_msgs::Point targetPoint;
    geometry_msgs::Point midPoint;

    startPoint = startPose.position;
    targetPoint = targetPose.position;

    // midPose4
    midPoint.x = targetPoint.x;
    midPoint.y = targetPoint.y;
    midPoint.z = startPoint.z;

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

    // Give targetPose
    placeWayPoints.push_back(targetPose);

    return placeWayPoints;
}
void setPlacePose()
{
    placePose.position.x = -0.56;
    placePose.position.y = -0.52;
    placePose.position.z = 0.3;
    placePose.orientation.x = 1;
    placePose.orientation.y = 0;
    placePose.orientation.z = 0;
    placePose.orientation.w = 0;

// 如果使用UR实物，请解除下面三行的注释
#ifdef UR5
    geometry_msgs::Pose pose;
    pose = placePose;
    placePose = changePoseForUR(pose);
#endif
}

//前往放置位置
void goPlacePose(geometry_msgs::Pose placePose)
{
#ifdef UR5
    moveit::planning_interface::MoveGroup arm_group("manipulator");	//manipulator
    std::vector< double > jointValues;
    jointValues.push_back(0.6053);
    jointValues.push_back(-0.8751);
    jointValues.push_back(0.9604);
    jointValues.push_back(-1.6561);
    jointValues.push_back(-1.5708);
    jointValues.push_back(2.1761);
    arm_group.setJointValueTarget(jointValues);
#else
    moveit::planning_interface::MoveGroup arm_group("arm");	//manipulator
    arm_group.setPoseTarget(placePose);
#endif
    arm_group.move();
}

// 把位姿态转换一下，给UR使用
geometry_msgs::Pose changePoseForUR(geometry_msgs::Pose pose)
{
    double x_init,y_init,z_init,w_init;
    double x_mid,y_mid,z_mid,w_mid;
    double x_trans,y_trans,z_trans,w_trans;
    double x_x180,y_x180,z_x180,w_x180;
    double x_y270,y_y270,z_y270,w_y270;

    pose.position.x = -pose.position.x;
    pose.position.y = -pose.position.y;
    pose.position.z = pose.position.z;

    x_init = pose.orientation.x;
    y_init = pose.orientation.y;
    z_init = pose.orientation.z;
    w_init = pose.orientation.w;

    x_x180 = 1;
    y_x180 = 0;
    z_x180 = 0;
    w_x180 = 0;

    x_y270 = 0;
    y_y270 = -0.707;
    z_y270 = 0;
    w_y270 = 0.707;

    // init and y270
    w_mid = z_init*z_y270 - x_init*x_y270 - y_init*y_y270 - z_init*z_y270;
    x_mid = w_init*x_y270 + x_init*w_y270 + z_init*y_y270 - y_init*z_y270;
    y_mid = w_init*y_y270 + y_init*w_y270 + x_init*z_y270 - z_init*x_y270;
    z_mid = w_init*z_y270 + z_init*w_y270 + y_init*x_y270 - x_init*y_y270;

    // mid and x180
    w_trans = z_mid*z_x180 - x_mid*x_x180 - y_mid*y_x180 - z_mid*z_x180;
    x_trans = w_mid*x_x180 + x_mid*w_x180 + z_mid*y_x180 - y_mid*z_x180;
    y_trans = w_mid*y_x180 + y_mid*w_x180 + x_mid*z_x180 - z_mid*x_x180;
    z_trans = w_mid*z_x180 + z_mid*w_x180 + y_mid*x_x180 - x_mid*y_x180;

    // so...
    pose.orientation.x = x_trans;
    pose.orientation.y = y_trans;
    pose.orientation.z = z_trans;
    pose.orientation.w = w_trans;

    return pose;
}
