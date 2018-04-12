#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <kinova_driver/kinova_ros_types.h>

#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Pose>("hub", 10);
  sleep(2.0);

  tf::Pose random_pose1;
  random_pose1.setOrigin(tf::Vector3(0.54882, -0.30854,  0.45841));
  random_pose1.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));

  tf::Pose random_pose2;
  random_pose2.setOrigin(tf::Vector3(0.54882, -0.40854,  0.5841));
  random_pose2.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));

  tf::Pose random_pose3;
  random_pose3.setOrigin(tf::Vector3(0.24882, -0.4854,  0.45841));
  random_pose3.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));

  geometry_msgs::Pose target_pose1;
  tf::poseTFToMsg(random_pose1, target_pose1);

  geometry_msgs::Pose target_pose2;
  tf::poseTFToMsg(random_pose2, target_pose2);

  geometry_msgs::Pose target_pose3;
  tf::poseTFToMsg(random_pose3, target_pose3);

  chatter_pub.publish(target_pose1);
  sleep(1);//注意这里间接发布位姿的时间要比留给机械臂运动的时间稍长

  chatter_pub.publish(target_pose2);
  sleep(1);

  chatter_pub.publish(target_pose3);
  sleep(1);

  while(ros::ok())
  {

  }
  ros::spinOnce();

  return 0;
}
