#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <kinova_driver/kinova_ros_types.h>

geometry_msgs::Pose target_pose1;
bool flag = true;//　用于判断是否有需要执行的位姿信息
int grip = 0; // 用于给出手爪的执行动作，０抓紧，１不动作，２松开，３不动作

void chatterCallback(const geometry_msgs::Pose& pose1)
{
  target_pose1 = pose1;
  flag = false;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_plan");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("arm");

  //　实物控制的话，得删掉这两句－－删掉是为了减少Rviz的使用所占用的时间
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ros::Subscriber sub = node_handle.subscribe("hub", 10, chatterCallback);
  sleep(1.0);
  
  // Add by Petori in 2018/4/8
  group.setEndEffectorLink( "j2s7s300_end_effector");

  while(ros::ok())
  {
    if (flag == false)
    {
      //ros::spinOnce();
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
      //moveit::planning_interface::MoveGroup::Plan my_plan;

      bool success = group.move();// group.plan(my_plan)
      double tPlan;
      tPlan = group.getPlanningTime();
      ROS_INFO("Planning time is [%lf]s.", tPlan);
      ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
      /* 实物控制的话删掉上面两句，并增加下面这句－－－－删掉是为了减少Rviz的使用所占用的时间
      group.move();*/
      //sleep(2.0);
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
