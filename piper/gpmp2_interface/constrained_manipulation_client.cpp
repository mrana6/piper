#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// #include <hlpr_trac_ik/IKHandler.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>

// MoveIt!
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/planning_interface/planning_interface.h>
// #include <moveit/planning_scene/planning_scene.h>
// #include <moveit/kinematic_constraints/utils.h>
// #include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_msgs/PlanningScene.h>

// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>

// #include <moveit/move_group_interface/move_group.h>

#include <actionlib/client/simple_action_client.h>
#include <piper/ConstrainedManipulationAction.h>

#include <string>
#include <iostream>
#include <fstream>


/* *************************************************************************** */


class FetchConstrainedManipulator {
private:
	// ros::ServiceClient trac_ik_client_;
	actionlib::SimpleActionClient<piper::ConstrainedManipulationAction> manipulation_client_;
	std::vector<std::string> arm_joint_names_;

	std::vector<double> current_conf_;  //current robot configuration
	std::vector<double> start_conf_;	// conf to start optimization from 
	std::vector<double> goal_conf_;     // goal configuraiton 

	tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber arm_state_sub_;
    // moveit::planning_interface::MoveGroup group_;
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    ros::Publisher display_publisher_;
    sensor_msgs::JointState joint_state_;


    void readWaypointsFromFile(std::string filename);
    void armStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    std::vector<double> getIK(geometry_msgs::Pose goal_pose);
    void goToJointPose(std::vector<double> conf);


public:
	FetchConstrainedManipulator(ros::NodeHandle nh);
	void sendGoal(std::string waypoint_file);
	geometry_msgs::PoseArray waypoints;   //cartesian waypoints
	
};

/* *************************************************************************** */

FetchConstrainedManipulator::FetchConstrainedManipulator(ros::NodeHandle nh) : 
	manipulation_client_("piper/execute_constrained_manipulation"),
	tf_listener_(tf_buffer_)
	// group_("arm")
{
	// group_.startStateMonitor();
	// trac_ik_client_ = nh.serviceClient<hlpr_trac_ik::IKHandler>("/hlpr_trac_ik");
	// display_publisher_ = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

	arm_joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint",
                        "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};


    current_conf_ = std::vector<double>(arm_joint_names_.size(),10);             

   	// arm_state_sub_ = nh.subscribe("joint_states", 1, &FetchConstrainedManipulator::armStateCallback, this);
   	ros::Duration(0.5).sleep();

   	//ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh);
    
   	// ros::spinOnce();
}

/* *************************************************************************** */

void FetchConstrainedManipulator::sendGoal(std::string waypoint_file)
{


	readWaypointsFromFile(waypoint_file);

	geometry_msgs::Pose start_pose = waypoints.poses[0];
	// start_conf_ = getIK(start_pose);
	// start_conf_ = {1.480381, -0.519910, 2.330502, 0.773411, 0.576692, 2.159374, 1.160902};
	start_conf_ = {1.60517731706, -0.416938478551, 2.71645797496, 0.791172502393, 0.269659725458, 2.0819469834, 1.3784506785};//current_conf_;

	// goToJointPose(start_conf_);

	ros::Duration(2.0).sleep();

	geometry_msgs::Pose goal_pose = waypoints.poses.back();
	// goal_conf_ = getIK(goal_pose);
	// goal_conf_ = {1.364771, -0.558937, 2.354118, 1.267287, 0.581348, 1.774089, 0.915123};
	goal_conf_ = {1.57373038331, -0.377438480459, 2.69728339915, 1.3069734553, 0.250101087124, 1.63517488486, 1.23272209039};


	// goToPose(goal_pose);



	piper::ConstrainedManipulationGoal goal;
	goal.waypoints = waypoints;
	goal.goal_conf = goal_conf_;
	goal.start_conf = start_conf_;
	// goal.start_conf = {1.60517731706, -0.416938478551, 2.71645797496, 0.791172502393, 0.269659725458, 2.0819469834, 1.3784506785};//current_conf_;

  	manipulation_client_.sendGoal(goal);
  	manipulation_client_.waitForResult(ros::Duration(5.0));

}



/* *************************************************************************** */

void FetchConstrainedManipulator::goToJointPose(std::vector<double> conf)
{
	/*
	// std::vector<double> conf = {1.60517731706, -0.416938478551, 2.71645797496, 0.791172502393, 0.269659725458, 2.0819469834, 1.3784506785};

	size_t index;
	for (size_t i=0; i<arm_joint_names_.size(); i++)
	  {
	    index = std::distance(joint_state_.name.begin(), find(joint_state_.name.begin(), joint_state_.name.end(), 
	      arm_joint_names_[i]));
	    group_.setJointValueTarget(joint_state_.name[index], conf[i]);
	  }

    group_.setPlannerId("arm[RRTConnectkConfigDefault]");
    group_.setPlanningTime(6.0);
    group_.setStartStateToCurrentState();
    ROS_INFO("Planning and moving...");
    move_group_interface::MoveItErrorCode errorCode = group_.move();
    std::cout<<"Hello"<<std::endl;

    if (errorCode == moveit_msgs::MoveItErrorCodes::SUCCESS)
  	{
    ROS_INFO("Succeeded");
 	 }
  	else
  	{
    ROS_INFO("Failed with MoveIt error code: %d", errorCode.val);
  	}
	  */
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

  for (int i = 0; i < traj.size(); i+=4){
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
  joint_state_ = *msg;
  size_t index;
  for (size_t i=0; i<arm_joint_names_.size(); i++)
  {
    index = std::distance(msg->name.begin(), find(msg->name.begin(), msg->name.end(), 
      arm_joint_names_[i]));
      current_conf_[i] = msg->position[index];
  }
}

/* *********************************************************************** */

std::vector<double> FetchConstrainedManipulator::getIK(geometry_msgs::Pose goal_pose)
{	
	/*ROS_INFO("Finding Conf using TRAC_IK");

	// transforming pose to torso to avoid the torso link in IK
	// IMP: Set the trac_ik base_chain to torso_lift_link
	geometry_msgs::TransformStamped transform_stamped;
	ROS_INFO("Fixing the torso_link for IK!");
	transform_stamped = tf_buffer_.lookupTransform("torso_lift_link", "base_link", ros::Time(0));
	geometry_msgs::Vector3 base_torso_offset = transform_stamped.transform.translation; //no quartenion offset here

	goal_pose.position.x = goal_pose.position.x + base_torso_offset.x;
	goal_pose.position.y = goal_pose.position.y + base_torso_offset.y;
	goal_pose.position.z = goal_pose.position.z + base_torso_offset.z;

	std::vector<geometry_msgs::Pose> goal_vec;
	goal_vec.push_back(goal_pose);
	

	hlpr_trac_ik::IKHandler ik_srv;

	std::vector<float> ik_seed(current_conf_.begin(), current_conf_.end());

    ik_srv.request.goals = goal_vec;
  	ik_srv.request.origin = ik_seed;

	ik_srv.request.tolerance = {0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001};
  	ik_srv.request.verbose = true;

  	std::vector<double> conf;

  	if (trac_ik_client_.call(ik_srv))
	  {
	  	if (ik_srv.response.success){
	  		conf = ik_srv.response.poses.back().positions;
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

	return conf;
   */
}

/* *********************************************************************** */

int main(int argc, char** argv)
{
	ros::init(argc, argv, "constrained_manipulation_client");
	ros::MultiThreadedSpinner spinner(4);

	ros::NodeHandle nh;

	FetchConstrainedManipulator fetch_constrained_manipulator(nh);
	fetch_constrained_manipulator.sendGoal("/home/asif/fetch_ws/src/piper/data/eef/urdf/hat_reach.txt");

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
