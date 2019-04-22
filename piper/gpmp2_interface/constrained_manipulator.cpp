#include <constrained_manipulator.h>

namespace piper {

ConstrainedManipulator::ConstrainedManipulator(ros::NodeHandle nh_) :
   constrained_manipulation_server_(nh_, "execute_constrained_manipulation", boost::bind(&ConstrainedManipulator::executeCallback, this, _1),false),
   problem_(nh_),
   traj_(nh_),
   tf_listener_(tf_buffer_)
{
	if (!nh_.hasParam("start_conf"))
		use_current_start_conf = true;  // sets start conf to current conf
	else
		use_current_start_conf = false;

	if (!nh_.hasParam("goal_conf"))
		find_goal_conf = true;   // sets goal conf to IK of goal pose
	else
		find_goal_conf = false;

	if (!nh_.hasParam("start_conf"))
		nh_.getParam("robot/arm_base/position", base_shoulder_offset_);
	else
		ROS_ERROR("Base to shoulder offset not found");

	nh_.getParam("fix_cartpose_sigma", fix_cartpose_sigma_);
	nh_.getParam("fix_cartorient_sigma", fix_cartorient_sigma_);

	  // robot state subscriber (used to initialize start state if not passed as param)
	  if (nh_.hasParam("robot/arm_state_topic"))
	  {
	    nh_.getParam("robot/arm_state_topic", arm_state_topic_);
	    arm_state_sub_ = nh_.subscribe(arm_state_topic_, 1, &ConstrainedManipulator::armStateCallback, this);
	    arm_pos_ = gtsam::Vector::Zero(problem_.robot.getDOFarm());
	    arm_pos_time_ = ros::Time::now();
	  }
	  ros::Duration(1.0).sleep();

	trac_ik_client_ = nh_.serviceClient<hlpr_trac_ik::IKHandler>("/hlpr_trac_ik");

	constrained_manipulation_server_.start();



	// problem_ = Problem(nh_);
	// traj_ = Traj(nh_);
	// Server server(nh_, "execute_constrained_manipulation", boost::bind(&ConstrainedManipulator::execute, this, _1), false);
	// Server(nh_, "execute_constrained_manipulation", boost::bind(&ConstrainedManipulator::execute, this, _1), false);

}

// ConstrainedManipulator::ConstrainedManipulator()
//  : 
// 	nh_("~"),
// 	problem_(nh_),
// 	traj_(nh_),
// 	constrained_manipulation_server_(nh_, "execute_constrained_manipulation", boost::bind(&ConstrainedManipulator::execute, this, _1), false)
// {
// 	// nh_.getParam("fix_cartpose_sigma", fix_cartpose_sigma_);
//  //    nh_.getParam("fix_cartorient_sigma", fix_cartorient_sigma_);

//  //  // robot state subscriber (used to initialize start state if not passed as param)
//  //  if (nh_.hasParam("robot/arm_state_topic"))
//  //  {
//  //    nh_.getParam("robot/arm_state_topic", arm_state_topic_);
//  //    arm_state_sub_ = nh_.subscribe(arm_state_topic_, 1, &ConstrainedManipulator::armStateCallback, this);
//  //    arm_pos_ = gtsam::Vector::Zero(problem_.robot.getDOFarm());
//  //    arm_pos_time_ = ros::Time::now();
//  //  }
//  //  ros::Duration(1.0).sleep();

//  //  constrained_manipulation_server_.start();

// }

/* ************************************************************************** */

void ConstrainedManipulator::executeCallback(const piper::ConstrainedManipulationGoalConstPtr &goal)
{	
	 // vector<geometry_msgs::Pose> pose_vec =  goal->waypoints.poses;
	 problem_.copyMsgToTrajectory(goal->waypoints);

	  // get start from measurement if not passed as param
	  if (use_current_start_conf)
	  {
	    problem_.start_conf = arm_pos_;
	    if (problem_.robot.isThetaNeg())
	      problem_.robot.negateTheta(problem_.start_conf);
	  }
	  // get start from measurement if not passed as param
	  if (find_goal_conf)
	  {
	  	hlpr_trac_ik::IKHandler ik_srv;

		geometry_msgs::TransformStamped transform_stamped;

		// Make sure trac_ik is set to the base chain as torso_lift_link
      	transform_stamped = tf_buffer_.lookupTransform("torso_lift_link", "base_link", ros::Time(0));
      	geometry_msgs::Vector3 base_torso_offset = transform_stamped.transform.translation; //no quartenion offset here

	  	geometry_msgs::Pose goal_pose = goal->waypoints.poses.back();
	  	

	  	//////////////////////////////////////////////////////
	  	//NOTE: HACK FOR FETCH ONLY. SUBTRACTING THE LINEAR LINK OFFSET BEFORE PASSING TO IK
        //////////////////////////////////////////////////////
        goal_pose.position.x = goal_pose.position.x + base_torso_offset.x;
        goal_pose.position.y = goal_pose.position.y + base_torso_offset.y;
        goal_pose.position.z = goal_pose.position.z + base_torso_offset.z;
 		//////////////////////////////////////////////////////

	  	vector<geometry_msgs::Pose> goal_vec;
	  	goal_vec.push_back(goal_pose);
	  	ik_srv.request.goals = goal_vec;
	  	
	  	vector<double> temp_conf_vec = getStdVector(problem_.start_conf);
	  	vector<float> start_conf_vec(temp_conf_vec.begin(), temp_conf_vec.end());
	  	// for (int i = 0; i < start_conf_vec.size(); i++)
   	// 		ROS_INFO("%f ", start_conf_vec[i]);

	  	ik_srv.request.origin = start_conf_vec;

		ik_srv.request.tolerance = {0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001};
	  	ik_srv.request.verbose = true;

	  	if (trac_ik_client_.call(ik_srv))
		  {
		  	if (ik_srv.response.success){
		  		std::vector<double> goal_conf_vec = ik_srv.response.poses.back().positions;
		  		ROS_INFO("Found an IK solution for goal pose!");
		  		problem_.goal_conf = getVector(goal_conf_vec);

		  		if (problem_.robot.isThetaNeg())
	      			problem_.robot.negateTheta(problem_.goal_conf);
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


	// initialize trajectory
	traj_.initializeTrajectory(init_values_, problem_);


	    // solve with batch gpmp2
	  ROS_INFO("Optimizing...");
	  int DOF = problem_.robot.getDOF();

	  int total_time_step = problem_.traj.size() - 1;
	  ROS_INFO("Total time step is %i", total_time_step);

	  gtsam::noiseModel::Gaussian::shared_ptr cartpose_model = gtsam::noiseModel::Isotropic::Sigma(3, fix_cartpose_sigma_);
	  gtsam::noiseModel::Gaussian::shared_ptr cartorient_model = gtsam::noiseModel::Isotropic::Sigma(3, fix_cartorient_sigma_);

	  gtsam::NonlinearFactorGraph graph;
	  Vector start_vel = gtsam::Vector::Zero(DOF);
	  Vector end_vel = gtsam::Vector::Zero(DOF);

	  for (int i = 0; i < problem_.traj.size(); i++) {
	    gtsam::Key key_pos = gtsam::Symbol('x', i);
	    gtsam::Key key_vel = gtsam::Symbol('v', i);
	    if (i == 0)
	      graph.add(gtsam::PriorFactor<gtsam::Vector>(key_vel, start_vel, problem_.opt_setting.vel_prior_model));
		else if (i == total_time_step)
	      graph.add(gtsam::PriorFactor<gtsam::Vector>(key_vel, end_vel, problem_.opt_setting.vel_prior_model)); 
	    gtsam::Point3 pose(problem_.traj[i][0],problem_.traj[i][1],problem_.traj[i][2]);
	    graph.add(gpmp2::GaussianPriorWorkspacePositionArm(key_pos, problem_.robot.arm, DOF-1, pose, cartpose_model));
	    gtsam::Rot3 orient = gtsam::Rot3::Quaternion(problem_.traj[i][6],problem_.traj[i][3],problem_.traj[i][4],problem_.traj[i][5]);
	    //gtsam::Rot3 orient_corrected = orient * gtsam::Rot3::Quaternion(0.5, -0.5, 0.5, -0.5);
	    graph.add(gpmp2::GaussianPriorWorkspaceOrientationArm(key_pos, problem_.robot.arm, DOF-1, orient, cartorient_model));

	    problem_.delta_t = problem_.total_time / double(total_time_step);
	    ROS_INFO("Delta t is %f", problem_.delta_t);
	    // GP priors and cost factor
	    if (i > 0) {
	      gtsam::Key key_pos1 = gtsam::Symbol('x', i-1);
	      gtsam::Key key_pos2 = gtsam::Symbol('x', i);
	      gtsam::Key key_vel1 = gtsam::Symbol('v', i-1);
	      gtsam::Key key_vel2 = gtsam::Symbol('v', i);
	      graph.add(gpmp2::GaussianProcessPriorLinear(key_pos1, key_vel1, key_pos2, key_vel2, problem_.delta_t, problem_.opt_setting.Qc_model));

	      // Cost factor
	      graph.add(gpmp2::ObstacleSDFFactorArm(key_pos, problem_.robot.arm, problem_.sdf, problem_.cost_sigma, problem_.epsilon));

	      /*if (problem_.obs_check_inter > 0) {
	        double total_check_step = (problem_.obs_check_inter + 1.0) * double(total_time_step);
	        for (int j = 0; j < problem_.obs_check_inter; j++) {
	          double tau = j * (problem_.total_time / total_check_step);
	          graph.add(gpmp2::ObstacleSDFFactorGPArm(key_pos1, key_vel1, key_pos2, key_vel2, problem_.robot.arm, problem_.sdf, problem_.cost_sigma, problem_.epsilon, problem_.opt_setting.Qc_model, problem_.delta_t, tau));
	        }
	      }*/
	    }
	  }
	  gtsam::Values init_values = gpmp2::initArmTrajStraightLine(problem_.start_conf, problem_.goal_conf, total_time_step);
	// init_values.insert(qkey, qinit);

	  gtsam::LevenbergMarquardtParams parameters;
	  parameters.setVerbosity("ERROR");
	  parameters.setVerbosityLM("LAMBDA");
	  parameters.setlambdaInitial(1200.0);
	  parameters.setlambdaUpperBound(1.0e10);
	  parameters.setMaxIterations(900);
	  gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, parameters);
	  optimizer.optimize();
	  batch_values_ = optimizer.values();

	   ROS_INFO("Batch GPMP2 optimization complete.");

	  // publish trajectory for visualization or other use
	  if (traj_.plan_traj_pub)
	    traj_.publishPlannedTrajectory(batch_values_, problem_, 0);

	  ConstrainedManipulator::execute();


    piper::ConstrainedManipulationResult result;
	result.plan_success = true;
	result.execution_success = true;
	constrained_manipulation_server_.setSucceeded(result);
}


// void ConstrainedManipulator::executeCallback(const piper::ConstrainedManipulationGoalConstPtr &goal)
// {

// }

/* ************************************************************************** */
void ConstrainedManipulator::execute()
{
  size_t exec_step;
  double coll_cost;

  // interpolate batch solution to a desired resolution for control and check for collision
  ROS_INFO("Checking for collision.");

    exec_values_ = gpmp2::interpolateArmTraj(batch_values_, problem_.opt_setting.Qc_model, problem_.delta_t, 
      problem_.control_inter, 0, problem_.total_step-1);
    coll_cost = gpmp2::CollisionCost3DArm(problem_.robot.arm, problem_.sdf, exec_values_, problem_.opt_setting);
  if (coll_cost != 0)
  {
    ROS_FATAL("Plan is not collision free! Collision cost = %.3f", coll_cost);
    sigintHandler(0);
  }

  //  execute trajectory
  ROS_INFO("Executing GPMP2 planned trajectory open-loop...");
  exec_step = problem_.total_step+problem_.control_inter*(problem_.total_step-1);
  if (write_traj_) 
    traj_.writeTrajectory(exec_values_, problem_, exec_step);
  if (execute_traj_)
    traj_.executeTrajectory(exec_values_, problem_, exec_step);
}


/* ************************************************************************** */
void ConstrainedManipulator::armStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  size_t index;
  for (size_t i=0; i<problem_.robot.getDOFarm(); i++)
  {
    index = std::distance(msg->name.begin(), find(msg->name.begin(), msg->name.end(), 
      traj_.arm_joint_names[i]));
    arm_pos_[i] = msg->position[index];
  }
  arm_pos_time_ = ros::Time::now();
}

}

/* ************************************************************************** */

int main(int argc, char** argv)
{
	ros::init(argc, argv, "constrained_manipulation_server");
    signal(SIGINT, piper::sigintHandler);

	ros::NodeHandle nh("piper");


	piper::ConstrainedManipulator manipulator(nh);

	ros::spin();

	return EXIT_SUCCESS;
}
