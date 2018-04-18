#include <ros/ros.h>
 
#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"
#include <actionlib/client/simple_action_client.h>
 
typedef actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> ArmJoint_actionlibClient;
typedef actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ArmPose_actionlibClient;
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> Finger_actionlibClient;
 
std::string kinova_robot_type = "j2s7s300";
std::string Finger_action_address = "/" + kinova_robot_type + "_driver/fingers_action/finger_positions";    //用于连接的手指控制服务器的名称
std::string finger_position_sub_address = "/" + kinova_robot_type +"_driver/out/finger_position";           //手指位置信息的话题
 
int finger_maxTurn = 6400;              // max thread rotation for one finger
int setFingerPos[3] = {0,0,0};          //设置手指闭合百分比
 
bool currcent_flag = false;
bool sendflag = true;
 
typedef struct _fingers
{
  float finger1;
  float finger2;
  float finger3;
 
} FINGERS;
 
FINGERS fingers;
 
void sendFingerGoal(int *p)
{
    Finger_actionlibClient client(Finger_action_address, true);
    kinova_msgs::SetFingersPositionGoal goal;
 
    goal.fingers.finger1 = (float)p[0] / 100 * finger_maxTurn;
    goal.fingers.finger2 = (float)p[1] / 100 * finger_maxTurn;
    goal.fingers.finger3 = (float)p[2] / 100 * finger_maxTurn;
 
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");
 
    //sendFingerGoal();
    client.sendGoal(goal);
 
    bool finish_before_timeout = client.waitForResult(ros::Duration(5.0));
     
    if(finish_before_timeout)
    {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO("action finish : %s",state.toString().c_str());
    }
    else
    {
        ROS_INFO("TIMEOUT");
    }
}
 
void currentFingerPoseFeedback(const kinova_msgs::FingerPosition  finger_pose_command)
{
    fingers.finger1= finger_pose_command.finger1;
    fingers.finger2= finger_pose_command.finger2;
    fingers.finger3= finger_pose_command.finger3;
    currcent_flag = true;
}

int main(int argc, char** argv)
{
    if(argc != 4)
    {
        printf("arg error !!! \n");
        return -1;
    }
    else
    {
        setFingerPos[0] = atoi(argv[1]);               
        setFingerPos[1] = atoi(argv[2]);
        setFingerPos[2] = atoi(argv[3]);
    }
 
    ros::init(argc, argv, "finger_control");
    ros::NodeHandle nh;
    ros::Subscriber finger_sub = nh.subscribe(finger_position_sub_address, 10, currentFingerPoseFeedback);
 
     
   ros::Rate rate(10);
 
    while(ros::ok())
    {
        if(sendflag == true)
        {
            sendFingerGoal(setFingerPos);
            sendflag = false;
        }
        if(currcent_flag == true)
        {  
            ROS_INFO("\nFinger values in turn are:\n \tFinger1 = %f \n \tFinger2 = %f  \n \tFinger3 = %f",fingers.finger1,fingers.finger2,fingers.finger3);
            ros::shutdown();
        }
        ros::spinOnce();
        rate.sleep();
    }
     
    return 0;
}


