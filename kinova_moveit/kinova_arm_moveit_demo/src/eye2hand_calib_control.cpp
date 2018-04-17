#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <kinova_driver/kinova_ros_types.h>

#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>

#include <iostream>
#include <vector>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

const double FINGER_MAX = 6400;
actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>* finger_client_=NULL;
moveit::planning_interface::MoveGroup* gripper_group_=NULL;

geometry_msgs::Pose target_pose1;
bool flag = true;//　用于判断是否有需要执行的位姿信息
int grip = 0; // 用于给出手爪的执行动作，０抓紧，１不动作，２松开，３不动作

void chatterCallback(const geometry_msgs::Pose& pose1)
{
  target_pose1 = pose1;
  flag = false;
}

//获取当前位姿
geometry_msgs::Pose get_current_pose(moveit::planning_interface::MoveGroup &group)
{
	geometry_msgs::PoseStamped getPoseMsg;
  getPoseMsg = group.getCurrentPose();

  geometry_msgs::Pose current_pose;
  current_pose = getPoseMsg.pose;

	ROS_INFO("Current position :\n x[%f], y[%f], z[%f]", current_pose.position.x, current_pose.position.y, current_pose.position.z);
	ROS_INFO("Current orientation :\n x[%f], y[%f], z[%f], w[%f]", current_pose.orientation.x, current_pose.orientation.y,
																															 current_pose.orientation.z, current_pose.orientation.w);
  return current_pose;
}

//获取old位姿坐标系下，从old到new的位置和姿态增量
MatrixXd getPoseIncreasement(const geometry_msgs::Pose &current_pose, const geometry_msgs::Pose &old_pose)
{
	VectorXd poseCur_t= MatrixXd::Zero(3, 1);		//当前位姿位置
	Matrix3d poseCur_r;	//当前位姿旋转矩阵
	VectorXd poseOld_t= MatrixXd::Zero(3, 1);	//之前位姿位置
	Matrix3d poseOld_r;	//之前位姿旋转矩阵
	//当前位姿的四元数转旋转矩阵
	Quaterniond poseCur_r_q(current_pose.orientation.w,
													current_pose.orientation.x,
													current_pose.orientation.y,
													current_pose.orientation.z);
	poseCur_r=poseCur_r_q.matrix();
	//当前位姿的位置向量
	poseCur_t<<current_pose.position.x, current_pose.position.y, current_pose.position.z;
	//之前位姿的四元数转旋转矩阵
	Quaterniond poseOld_r_q(old_pose.orientation.w,
													old_pose.orientation.x,
													old_pose.orientation.y,
													old_pose.orientation.z);
	poseOld_r=poseOld_r_q.matrix();
	//当前位姿的位置向量
	poseOld_t<<old_pose.position.x, old_pose.position.y, old_pose.position.z;

	//位姿增量计算
	Matrix3d R_old_new;
  VectorXd t_old_new(3);
  R_old_new = poseOld_r.transpose()*poseCur_r;
  t_old_new = poseOld_r.transpose()*poseCur_t-poseOld_r.transpose()*poseOld_t;

	//转换为4X4矩阵
  MatrixXd old_new = MatrixXd::Zero(4, 4);
  for(int j=0; j<3; j++)
  {
    for(int k=0; k<3; k++)	
	{
	  old_new(j,k)=R_old_new(j,k);
	}		
  }
	for(int j=0; j<3; j++)
	{
	  old_new(j,3)=t_old_new(j);
	}
	old_new(3,3)=1.0;

	cout<<"old_new :"<<endl<<old_new<<endl;

  return old_new;
}

//手指抓取
bool gripper_action(double finger_turn)
{
		/*
    if(robot_connected_ == false)
    {
        if (finger_turn>0.5*FINGER_MAX)
        {
          gripper_group_->setNamedTarget("Close");
        }
        else
        {
          gripper_group_->setNamedTarget("Open");
        }
        gripper_group_->move();
        return true;
    }*/

    if (finger_turn < 0)
    {
        finger_turn = 0.0;
    }
    else
    {
        finger_turn = std::min(finger_turn, FINGER_MAX);
    }

    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = finger_turn;
    goal.fingers.finger2 = goal.fingers.finger1;
    goal.fingers.finger3 = goal.fingers.finger1;
    finger_client_->sendGoal(goal);

    if (finger_client_->waitForResult(ros::Duration(5.0)))
    {
        finger_client_->getResult();
        return true;
    }
    else
    {
        finger_client_->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "eye2hand_calib_control");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("arm");		//这句话的作用？
	gripper_group_ = new moveit::planning_interface::MoveGroup("gripper");

  //　实物控制的话，得删掉这两句－－删掉是为了减少Rviz的使用所占用的时间
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ros::Subscriber sub = node_handle.subscribe("hub", 10, chatterCallback);
  sleep(1.0);
  
  // Add by Petori in 2018/4/8
  group.setEndEffectorLink( "j2s7s300_end_effector");	//这句话的作用？

	//抓紧手指
/*
	finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
            ("/j2s7s300_driver/fingers_action/finger_positions", false);
  while(ros::ok() && !finger_client_->waitForServer(ros::Duration(5.0)))
	{
     ROS_INFO("Waiting for the finger action server to come up");
  }
*/
	sleep(3.0);
//	gripper_action(3200.0);
	gripper_group_->setNamedTarget("Close");
	gripper_group_->move();
	sleep(3.0);

	//位置0 初始位置
	ROS_INFO("STATE 0");
	tf::Pose random_pose;
  random_pose.setOrigin(tf::Vector3(0.54882, -0.30854,  0.65841));
  random_pose.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));

  geometry_msgs::Pose target_pose;
	tf::poseTFToMsg(random_pose, target_pose);
	group.setPoseTarget(target_pose);
	group.move();
	int k=0;
	while(ros::ok() && k<5)
	{
			sleep(1.0);
			k++;
	}

	geometry_msgs::Pose current_pose, old_pose;	//获取当前末端位姿
	current_pose=get_current_pose(group);
	old_pose=current_pose;
	
	//位置1
	ROS_INFO("STATE 1");
	random_pose.setOrigin(tf::Vector3(0.64882, -0.30854,  0.65841));
  random_pose.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));
	tf::poseTFToMsg(random_pose, target_pose);
	group.setPoseTarget(target_pose);
	group.move();
	k=0;
	while(ros::ok() && k<5)
	{
			sleep(1.0);
			k++;
	}
	current_pose=get_current_pose(group);	//获取当前末端位姿
	MatrixXd d_pose=MatrixXd::Zero(4, 4);	//位姿增量
	d_pose=getPoseIncreasement(current_pose,old_pose);
	old_pose=current_pose;

	//位置2
	ROS_INFO("STATE 2");
	random_pose.setOrigin(tf::Vector3(0.64882, -0.30854,  0.55841));
  random_pose.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));
	tf::poseTFToMsg(random_pose, target_pose);
	group.setPoseTarget(target_pose);
	group.move();
	k=0;
	while(ros::ok() && k<5)
	{
			sleep(1.0);
			k++;
	}

	current_pose=get_current_pose(group);	//获取当前末端位姿
	old_pose=current_pose;

	//位置3
	ROS_INFO("STATE 3");
	random_pose.setOrigin(tf::Vector3(0.64882, -0.30854,  0.60841));
  random_pose.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));
	tf::poseTFToMsg(random_pose, target_pose);
	group.setPoseTarget(target_pose);
	group.move();
	k=0;
	while(ros::ok() && k<5)
	{
			sleep(1.0);
			k++;
	}
	current_pose=get_current_pose(group);
	old_pose=current_pose;

	//位置4
	ROS_INFO("STATE 4");
	random_pose.setOrigin(tf::Vector3(0.54882, -0.30854,  0.65841));
  random_pose.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));
	tf::poseTFToMsg(random_pose, target_pose);
	group.setPoseTarget(target_pose);
	group.move();
	k=0;
	while(ros::ok() && k<5)
	{
			sleep(1.0);
			k++;
	}

	current_pose=get_current_pose(group);	//获取当前末端位姿
	old_pose=current_pose;

	sleep(10.0);
	gripper_group_->setNamedTarget("Open");
	gripper_group_->move();

/*
  while(ros::ok())
  {
    if (flag == false)
    {
      ros::spinOnce();
      group.setStartState(*group.getCurrentState());

      // 下面这几句只是为了展示规划的起始点
      geometry_msgs::PoseStamped msg;
      msg = group.getCurrentPose();

      geometry_msgs::Pose current_pose;
      current_pose = msg.pose;

      geometry_msgs::Point position = current_pose.position;
      //geometry_msgs::Orientation orientation = current_pose.orientation;

      float aa, bb, cc;
      aa = position.x;
      bb = position.y;
      cc = position.z;
      ROS_INFO("Current end-effector State : x[%f], y[%f], z[%f]", aa, bb, cc);
      //group.getCurrentRPY(const std::string &end_effector_link="j2s7s300_end_effector");
      group.setPoseTarget(target_pose1);
      
      //　默认从当前位置规划至目标位置，因此我不需要额外添加语句去获取机器人当前姿态．
      moveit::planning_interface::MoveGroup::Plan my_plan;

      bool success = group.plan(my_plan);
      ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
      // 实物控制的话删掉上面两句，并增加下面这句－－－－删掉是为了减少Rviz的使用所占用的时间
      //group.move();

      sleep(2.0);
      if(grip == 0);
        //手爪收紧---手爪的控制是否考虑受力的问题
        // 给一个收紧时间
      else if(grip == 1);
        //手爪不动作
      else if(grip == 2);
        //手爪松开
        // 给一个松开时间
      else if(grip == 3);
      //手爪不动作
      else ;
        
      grip ++;
      grip = grip%4;
      flag = true;
    }
  }
*/
  ros::shutdown();  
  return 0;
}

  /*　测试是否收到了pose消息
  geometry_msgs::Pose pose = msg->pose;
  geometry_msgs::Point position = target_pose1.position;
  geometry_msgs::Quaternion orientation = target_pose1.orientation;
  float aaaa;
  aaaa = orientation.x;
  ROS_INFO("I got a date: [%f]", aaaa);*/  

// 测试－创建一个话题hub，并发布geometry/Pose类型的数据
// while(true){
// ros::Publisher hub = node_handle.advertise<geometry_msgs::Pose>("hub", 20, true);
// hub.publish(target_pose1);}
// 结束
