//
// Simple demo program that calls the youBot ROS wrapper with a trajectory
//
 
#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
 
int main(int argc, char **argv) {
	ros::init(argc, argv, "j2s7300_finger");
	ros::NodeHandle n;
 
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/j2s7s300/effort_joint_trajectory_controller/follow_joint_trajectory", true);
 
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();
 
	ROS_INFO("Action server started, sending trajectory.");
 
	// setup trajectory message
	control_msgs::FollowJointTrajectoryGoal msg;
 
	// some arbitrary points of a not too useful trajectory
	const int nPoints = 1;
	double values[nPoints][10] = {
//		{0.11, 0.11, -0.10, 0.11, 0.13},// Home 
		{0.0, 2.9, 0.389, 1.297, 10.477, 1.400, -18.849, 0.5, 0.5, 0.5}// Candle
		 };
 
	// set values for all points of trajectory
	for (int p = 0; p < nPoints; p++)
	{ // iterate over all points
		trajectory_msgs::JointTrajectoryPoint point;
 
		for (int i = 0; i < 10; i++)
		{ // 5 DOF
			point.positions.push_back(values[p][i]);
            point.velocities.push_back(0.1); // 0.1
			point.accelerations.push_back(0);
		}
        point.time_from_start = ros::Duration((p+1)*3.0);
		msg.trajectory.points.push_back(point);
	}
 
	// set joint names
	for (int i = 1; i < 8; i++) {
		std::stringstream jointName;
		jointName << "j2s7s300_joint_" << (i);
		msg.trajectory.joint_names.push_back(jointName.str());
	}
	for (int i = 1; i < 4; i++) {
		std::stringstream jointName;
		jointName << "j2s7s300_joint_finger_" << (i);
		msg.trajectory.joint_names.push_back(jointName.str());
	}
 
	// fill message header and sent it out
	msg.trajectory.header.frame_id = "root";
	msg.trajectory.header.stamp = ros::Time::now();
	ac.sendGoal(msg);
 
	// wait for reply that action was completed (or cancel after 10 sec)
	ac.waitForResult(ros::Duration(10));
	ROS_INFO("end");
 
	return 0;
}
