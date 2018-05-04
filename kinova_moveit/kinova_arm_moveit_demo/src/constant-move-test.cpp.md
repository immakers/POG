#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <kinova_driver/kinova_ros_types.h>

// Written by Petori in 2018/4/20
//　用于连续运动测试，使用时需备份原来的simple_move.cpp，并把此文件改名为simple_move.cpp

int main(int argc, char **argv)
{

  ros::init(argc, argv, "simple_move");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("arm");
  moveit::planning_interface::MoveGroup::Plan plan;

  std::vector<geometry_msgs::Pose> waypoints;

  tf::Pose random_pose1;
  random_pose1.setOrigin(tf::Vector3(0.54882, -0.30854,  0.45841));
  random_pose1.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));

  tf::Pose random_pose2;
  random_pose2.setOrigin(tf::Vector3(0.54882, -0.40854,  0.5841));
  random_pose2.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));

  tf::Pose random_pose3;
  random_pose3.setOrigin(tf::Vector3(0.24882, 0.2854,  0.45841));
  random_pose3.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));

  geometry_msgs::Pose target_pose1;
  tf::poseTFToMsg(random_pose1, target_pose1);

  geometry_msgs::Pose target_pose2;
  tf::poseTFToMsg(random_pose2, target_pose2);

  geometry_msgs::Pose target_pose3;
  tf::poseTFToMsg(random_pose3, target_pose3);

  waypoints.push_back(target_pose1); 
  waypoints.push_back(target_pose2); 
  waypoints.push_back(target_pose3); 

  //　实物控制的话，得删掉这两句－－删掉是为了减少Rviz的使用所占用的时间
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Add by Petori in 2018/4/8
  group.setEndEffectorLink( "j2s7s300_end_effector");

  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);
  plan.trajectory_ = trajectory; 
  group.execute(plan);

  ROS_INFO("Visualizing plan over 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);   

      //bool success = group.move();// group.plan(my_plan)
      //double tPlan;
      //tPlan = group.getPlanningTime();
      //ROS_INFO("Planning time is [%lf]s.", tPlan);
      //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");


  ros::shutdown();  
  return 0;
}

