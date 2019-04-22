#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>


#include <actionlib/client/simple_action_client.h>
#include <piper/ConstrainedManipulationAction.h>

typedef actionlib::SimpleActionClient<piper::ConstrainedManipulationAction> Client;



int main(int argc, char** argv)
{
	ros::init(argc, argv, "constrained_manipulation_client");

	Client client("piper/execute_constrained_manipulation");
	client.waitForServer();

	piper::ConstrainedManipulationGoal goal;
	geometry_msgs::Pose temp_pose;

	temp_pose.position.x = -0.036942;
	temp_pose.position.y = 0.25664;
	temp_pose.position.z = 1.1827;
	temp_pose.orientation.x = -0.01821;
	temp_pose.orientation.y = 0.73873;
	temp_pose.orientation.z = -0.66873;
	temp_pose.orientation.w = 0.082096;

	goal.waypoints.poses.push_back(temp_pose);

    temp_pose.position.x = -0.057659;
	temp_pose.position.y = 0.11803;
	temp_pose.position.z = 1.1791;
	temp_pose.orientation.x = -0.017311;
	temp_pose.orientation.y = 0.73871;
	temp_pose.orientation.z = -0.6688;
	temp_pose.orientation.w = 0.081943;
	goal.waypoints.poses.push_back(temp_pose);


 //    temp_pose.position.x = 0.050;
	// temp_pose.position.y = -0.128;
	// temp_pose.position.z = 0.737;
	// temp_pose.orientation.x = 0.460;
	// temp_pose.orientation.y = -0.503;
	// temp_pose.orientation.z = 0.512;
	// temp_pose.orientation.w = 0.523;
	// goal.waypoints.poses.push_back(temp_pose);

	// temp_pose.position.x = 0.100;
	// temp_pose.position.y = -0.128;
	// temp_pose.position.z = 0.800;
	// temp_pose.orientation.x = 0.460;
	// temp_pose.orientation.y = -0.503;
	// temp_pose.orientation.z = 0.512;
	// temp_pose.orientation.w = 0.523;
	// goal.waypoints.poses.push_back(temp_pose);



  	client.sendGoal(goal);
  	client.waitForResult(ros::Duration(5.0));

	return 0;
}
