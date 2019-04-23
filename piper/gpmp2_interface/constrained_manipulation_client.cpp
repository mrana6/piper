#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <hlpr_trac_ik/IKHandler.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>



#include <actionlib/client/simple_action_client.h>
#include <piper/ConstrainedManipulationAction.h>

#include <string>
#include <iostream>
#include <fstream>


/* *************************************************************************** */


class FetchConstrainedManipulator {
private:
	ros::ServiceClient trac_ik_client_;
	actionlib::SimpleActionClient<piper::ConstrainedManipulationAction> manipulation_client_;
	std::vector<std::string> arm_joint_names_;

	std::vector<double> current_conf_;  //current robot configuration
	std::vector<double> goal_conf_;     // goal configuraiton 

	tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber arm_state_sub_;


    void readWaypointsFromFile(std::string filename);
    void armStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void findGoalConf();


public:
	FetchConstrainedManipulator(ros::NodeHandle nh);
	void sendGoal(std::string waypoint_file);
	geometry_msgs::PoseArray waypoints;   //cartesian waypoints
	
};

/* *************************************************************************** */

FetchConstrainedManipulator::FetchConstrainedManipulator(ros::NodeHandle nh) : 
	manipulation_client_("piper/execute_constrained_manipulation"),
	tf_listener_(tf_buffer_)
{
	trac_ik_client_ = nh.serviceClient<hlpr_trac_ik::IKHandler>("/hlpr_trac_ik");

	arm_joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint",
                        "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};

    current_conf_ = std::vector<double>(arm_joint_names_.size(),10);                 

   	arm_state_sub_ = nh.subscribe("joint_states", 1, &FetchConstrainedManipulator::armStateCallback, this);
   	ros::Duration(0.2).sleep();

   	//ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh);
    
   	//ros::spinOnce();
}

/* *************************************************************************** */

void FetchConstrainedManipulator::sendGoal(std::string waypoint_file)
{
	readWaypointsFromFile(waypoint_file);
	findGoalConf();


	piper::ConstrainedManipulationGoal goal;
	goal.waypoints = waypoints;
	goal.goal_conf = goal_conf_;
	goal.start_conf = current_conf_;

  	manipulation_client_.sendGoal(goal);
  	manipulation_client_.waitForResult(ros::Duration(5.0));

}

/* *************************************************************************** */

void FetchConstrainedManipulator::readWaypointsFromFile(std::string filename)
{
  std::ifstream file(filename);
  std::string line;

  std::vector<std::vector<double>> traj;

  while(getline(file, line)) {
    std::istringstream is(line);
    traj.push_back(std::vector<double>(std::istream_iterator<double>(is), std::istream_iterator<double>()));
  }

  geometry_msgs::Pose temp_pose;

  for (int i = 0; i < traj.size(); i++){
	temp_pose.position.x = traj[i][0];
	temp_pose.position.y = traj[i][1];
	temp_pose.position.z = traj[i][2];
	temp_pose.orientation.x = traj[i][3];
	temp_pose.orientation.y = traj[i][4];
	temp_pose.orientation.z = traj[i][5];
	temp_pose.orientation.w = traj[i][6];
	waypoints.poses.push_back(temp_pose);
	//ROS_INFO("%f, %f, %f, %f, %f, %f, %f", traj[i][0], traj[i][1], traj[i][2], traj[i][3], traj[i][4], traj[i][5], traj[i][6]);
  }
}

/* ************************************************************************** */

void FetchConstrainedManipulator::armStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  std::cout<<"Hello"<<std::endl;
  size_t index;
  for (size_t i=0; i<arm_joint_names_.size(); i++)
  {
    index = std::distance(msg->name.begin(), find(msg->name.begin(), msg->name.end(), 
      arm_joint_names_[i]));
      current_conf_[i] = msg->position[index];
  }
}

/* *********************************************************************** */

void FetchConstrainedManipulator::findGoalConf()
{	
	ROS_INFO("Finding Goal Conf using TRAC_IK");
	geometry_msgs::Pose goal_pose = waypoints.poses.back();

	std::cout<<goal_pose.position.x<<std::endl;

	// transforming pose to torso to avoid the torso link in IK
	// IMP: Set the trac_ik base_chain to torso_lift_link
	geometry_msgs::TransformStamped transform_stamped;
	ros::spinOnce();
	transform_stamped = tf_buffer_.lookupTransform("torso_lift_link", "base_link", ros::Time(0));
	geometry_msgs::Vector3 base_torso_offset = transform_stamped.transform.translation; //no quartenion offset here

	goal_pose.position.x = goal_pose.position.x + base_torso_offset.x;
	goal_pose.position.y = goal_pose.position.y + base_torso_offset.y;
	goal_pose.position.z = goal_pose.position.z + base_torso_offset.z;

	std::vector<geometry_msgs::Pose> goal_vec;
	goal_vec.push_back(goal_pose);
	

	hlpr_trac_ik::IKHandler ik_srv;

	std::vector<float> start_conf(current_conf_.begin(), current_conf_.end());

    ik_srv.request.goals = goal_vec;
  	ik_srv.request.origin = start_conf;

	ik_srv.request.tolerance = {0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001};
  	ik_srv.request.verbose = true;

  	if (trac_ik_client_.call(ik_srv))
	  {
	  	if (ik_srv.response.success){
	  		goal_conf_ = ik_srv.response.poses.back().positions;
	  		ROS_INFO("Found an IK solution for goal pose!");
	  	}
	  	else{
	  		ROS_ERROR("Failed to find IK solution!");
	  	}
	  }
	else
	  {
	    ROS_ERROR("Failed to call trac_ik service");
	  }

}

/* *********************************************************************** */

int main(int argc, char** argv)
{
	ros::init(argc, argv, "constrained_manipulation_client");
	ros::MultiThreadedSpinner spinner(0);

	ros::NodeHandle nh;

	FetchConstrainedManipulator fetch_constrained_manipulator(nh);
	fetch_constrained_manipulator.sendGoal("/home/asif/fetch_ws/src/piper/data/eef/test.txt");

	spinner.spin();

	// ros::spin();

	// ros::MultiThreadedSpinner spinner(0);
	// spinner.spin();



	// piper::ConstrainedManipulationGoal goal;
	// geometry_msgs::Pose temp_pose;

	// temp_pose.position.x = -0.036942;
	// temp_pose.position.y = 0.25664;
	// temp_pose.position.z = 1.1827;
	// temp_pose.orientation.x = -0.01821;
	// temp_pose.orientation.y = 0.73873;
	// temp_pose.orientation.z = -0.66873;
	// temp_pose.orientation.w = 0.082096;

	// goal.waypoints.poses.push_back(temp_pose);

 //    temp_pose.position.x = -0.057659;
	// temp_pose.position.y = 0.11803;
	// temp_pose.position.z = 1.1791;
	// temp_pose.orientation.x = -0.017311;
	// temp_pose.orientation.y = 0.73871;
	// temp_pose.orientation.z = -0.6688;
	// temp_pose.orientation.w = 0.081943;
	// goal.waypoints.poses.push_back(temp_pose);


 // //    temp_pose.position.x = 0.050;
	// // temp_pose.position.y = -0.128;
	// // temp_pose.position.z = 0.737;
	// // temp_pose.orientation.x = 0.460;
	// // temp_pose.orientation.y = -0.503;
	// // temp_pose.orientation.z = 0.512;
	// // temp_pose.orientation.w = 0.523;
	// // goal.waypoints.poses.push_back(temp_pose);

	// // temp_pose.position.x = 0.100;
	// // temp_pose.position.y = -0.128;
	// // temp_pose.position.z = 0.800;
	// // temp_pose.orientation.x = 0.460;
	// // temp_pose.orientation.y = -0.503;
	// // temp_pose.orientation.z = 0.512;
	// // temp_pose.orientation.w = 0.523;
	// // goal.waypoints.poses.push_back(temp_pose);



 //  	client.sendGoal(goal);
 //  	client.waitForResult(ros::Duration(5.0));

	return 0;
}
