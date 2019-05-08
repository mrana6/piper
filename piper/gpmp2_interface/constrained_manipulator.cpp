#include <constrained_manipulator.h>

namespace piper {

ConstrainedManipulator::ConstrainedManipulator(ros::NodeHandle nh) :
   constrained_manipulation_server_(nh, "execute_constrained_manipulation", boost::bind(&ConstrainedManipulator::executeCallback, this, _1),false),
   problem_(nh),
   traj_(nh)
{

	nh.getParam("fix_cartpose_sigma", fix_cartpose_sigma_);
	nh.getParam("fix_cartorient_sigma", fix_cartorient_sigma_);
	nh.getParam("robot/DOF", DOF_);

	if (nh.hasParam("robot/arm_state_topic"))
	  {
	    nh.getParam("robot/arm_state_topic", arm_state_topic_);
	    arm_state_sub_ = nh.subscribe(arm_state_topic_, 1, &ConstrainedManipulator::armStateCallback, this);
	    arm_pos_ = gtsam::Vector::Zero(problem_.robot.getDOFarm());
	    arm_pos_time_ = ros::Time::now();
	  }
	  ros::Duration(1.0).sleep();
	
	constrained_manipulation_server_.start();
}

/* ********************************************************************* */

void ConstrainedManipulator::executeCallback(const piper::ConstrainedManipulationGoalConstPtr &manipulation_goal)
{	
	gtsam::Vector start_conf;
	gtsam::Vector goal_conf;

	if (!manipulation_goal->start_conf.empty()) {
		ROS_INFO("Using Pre-defined start_conf");
		start_conf = getVector(manipulation_goal->start_conf);
	}
	else {
		ROS_INFO("Setting start_conf to current conf");
		start_conf = arm_pos_;
	}
		

	if (!manipulation_goal->goal_conf.empty()) {
		ROS_INFO("Using Pre-defined goal_conf");
		goal_conf = getVector(manipulation_goal->goal_conf);
		std::cout<<goal_conf[0]<<std::endl;
	}
	else {
		ROS_ERROR("No goal_conf supplied. Setting goal_conf = start_conf!");
		goal_conf = start_conf;
	}

	ConstrainedManipulator::plan(start_conf, goal_conf, manipulation_goal->waypoints);

	ConstrainedManipulator::execute();

	piper::ConstrainedManipulationResult result;
	result.plan_success = true;
	result.execution_success = true;
	constrained_manipulation_server_.setSucceeded(result);
}


/* ********************************************************************* */

void ConstrainedManipulator::plan(gtsam::Vector start_conf, gtsam::Vector goal_conf, geometry_msgs::PoseArray waypoints)
{
	problem_.start_conf = start_conf;
	if (problem_.robot.isThetaNeg())
		problem_.robot.negateTheta(problem_.start_conf);

	problem_.goal_conf = goal_conf;
	if (problem_.robot.isThetaNeg())
		problem_.robot.negateTheta(problem_.goal_conf);

	problem_.copyMsgToTrajectory(waypoints);

	// ADD FACTORS BETWEEN CARTESIAN WAYPOINTS!!??

	// solve with batch gpmp2
	ROS_INFO("Optimizing...");
	ROS_INFO("Total time step is %i", static_cast<int>(problem_.opt_setting.total_step));
	ROS_INFO("Delta t is %f", problem_.delta_t);
	ROS_INFO("Arm DOF: %i", DOF_);
	int DOF = problem_.robot.getDOF();

	gtsam::noiseModel::Gaussian::shared_ptr cartpose_model = gtsam::noiseModel::Isotropic::Sigma(3, fix_cartpose_sigma_);
	gtsam::noiseModel::Gaussian::shared_ptr cartorient_model = gtsam::noiseModel::Isotropic::Sigma(3, fix_cartorient_sigma_);


	std::vector<double> quat;
	std::vector<double> quat_offset = {0, 0.7071, 0, 0.7071};

	ROS_INFO("start_conf: %f, %f, %f, %f, %f, %f, %f", problem_.start_conf[0], problem_.start_conf[1], problem_.start_conf[2],
																problem_.start_conf[3], problem_.start_conf[4],problem_.start_conf[5], problem_.start_conf[6]);

	
	ROS_INFO("goal_conf: %f, %f, %f, %f, %f, %f, %f", problem_.goal_conf[0], problem_.goal_conf[1], problem_.goal_conf[2],
																problem_.goal_conf[3], problem_.goal_conf[4],problem_.goal_conf[5], problem_.goal_conf[6]);


	std::cout<<problem_.robot.arm.fk_model().forwardKinematicsPose(problem_.start_conf)<<std::endl;



	gtsam::NonlinearFactorGraph graph;


	init_values_.clear(); // TODO: CHECK IF GTSAM IS CLEARED

	// initialize trajectory
	traj_.initializeTrajectory(init_values_, problem_);
	init_values_ = gpmp2::initArmTrajStraightLine(problem_.start_conf, problem_.goal_conf, problem_.opt_setting.total_step);


	for (size_t i = 0; i <= problem_.opt_setting.total_step; i++) {
	    gtsam::Key key_pos = gtsam::Symbol('x', i);
	    gtsam::Key key_vel = gtsam::Symbol('v', i);
	    
	    // Adding init conf constraint is conflicting. I dont know why!!!
	    if (i == 0){
	    	graph.add(gtsam::PriorFactor<gtsam::Vector>(key_pos, problem_.start_conf, problem_.opt_setting.conf_prior_model));
	    	graph.add(gtsam::PriorFactor<gtsam::Vector>(key_vel, gtsam::Vector::Zero(DOF), problem_.opt_setting.vel_prior_model));
	  	}
		else if (i == problem_.opt_setting.total_step){
				// graph.add(gtsam::PriorFactor<gtsam::Vector>(key_pos, problem_.goal_conf, problem_.opt_setting.conf_prior_model));
	      graph.add(gtsam::PriorFactor<gtsam::Vector>(key_vel, gtsam::Vector::Zero(DOF), problem_.opt_setting.vel_prior_model)); 
	    }

    	      // Cost factor
      // graph.add(gpmp2::ObstacleSDFFactorArm(key_pos, problem_.robot.arm, problem_.sdf, problem_.cost_sigma, problem_.epsilon));

      //workspace factor
			// TODO: check arm model with matlab
    gtsam::Point3 pose(problem_.traj[i][0],problem_.traj[i][1],problem_.traj[i][2]);
	  graph.add(gpmp2::GaussianPriorWorkspacePositionArm(key_pos, problem_.robot.arm, DOF-1, pose, cartpose_model));

	  quat = {problem_.traj[i][6],problem_.traj[i][3],problem_.traj[i][4],problem_.traj[i][5]};
	  quat = quatmultiply(quat, quat_offset);
	  gtsam::Rot3 orient = gtsam::Rot3::Quaternion(quat[0], quat[1], quat[2], quat[3]);
	  graph.add(gpmp2::GaussianPriorWorkspaceOrientationArm(key_pos, problem_.robot.arm, DOF-1, orient, cartorient_model));

	  // if (i == 0 || i==problem_.opt_setting.total_step || i == 50){
	  // 	gpmp2::GaussianPriorWorkspaceOrientationArm factor = 
	  // 		gpmp2::GaussianPriorWorkspaceOrientationArm(key_pos, problem_.robot.arm, DOF-1, orient, cartorient_model);
	  // 	// std::cout<<orient<<std::endl;
	  // 	factor.print();
	  // 	// std::copy(begin(factor), end(factor), std::ostream_iterator<char>(std::cout, " "));

	  // }



	  // ROS_INFO("position constraint: %f, %f, %f", problem_.traj[i][0], problem_.traj[i][1], problem_.traj[i][2]);
	  // ROS_INFO("quaternion constraint: %f, %f, %f, %f", quat[0], quat[1], quat[2], quat[3]);
	    
	  // gtsam::Rot3 orient = gtsam::Rot3::Quaternion(problem_.traj[i][6],problem_.traj[i][3],problem_.traj[i][4],problem_.traj[i][5]);
	 //  if (problem_.robot.isOrientationOffset()){
		// 	problem_.robot.offsetOrientation(orient);
		// }
	  // orient = orient * gtsam::Rot3::Quaternion(0, 0.7071, 0, 0.7071);


	    // GP priors and cost factor
	    if (i > 0) {
	      gtsam::Key key_pos1 = gtsam::Symbol('x', i-1);
	      gtsam::Key key_pos2 = gtsam::Symbol('x', i);
	      gtsam::Key key_vel1 = gtsam::Symbol('v', i-1);
	      gtsam::Key key_vel2 = gtsam::Symbol('v', i);

				// TODO: May conflict if waypoints too far apart
	      graph.add(gpmp2::GaussianProcessPriorLinear(key_pos1, key_vel1, key_pos2, key_vel2, problem_.delta_t, problem_.opt_setting.Qc_model));


	      /*if (problem_.obs_check_inter > 0) {
	        double total_check_step = (problem_.obs_check_inter + 1.0) * double(total_time_step);
	        for (int j = 0; j < problem_.obs_check_inter; j++) {
	          double tau = j * (problem_.total_time / total_check_step);
	          graph.add(gpmp2::ObstacleSDFFactorGPArm(key_pos1, key_vel1, key_pos2, key_vel2, problem_.robot.arm, problem_.sdf, problem_.cost_sigma, problem_.epsilon, problem_.opt_setting.Qc_model, problem_.delta_t, tau));
	        }
	      }*/
	    }
	  }


	ROS_INFO("Optimizing...");
	gtsam::LevenbergMarquardtParams parameters;
	parameters.setVerbosity("ERROR");
	parameters.setVerbosityLM("LAMBDA");
	parameters.setlambdaInitial(1200.0);
	// parameters.setlambdaUpperBound(1.0e10);
	parameters.setMaxIterations(500);
	gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values_, parameters);



	// gtsam::DoglegParams parameters;
	// parameters.setVerbosity("ERROR");
	// // parameters.setlambdaInitial(1200.0);
	// // parameters.setlambdaUpperBound(1.0e10);
	// parameters.setMaxIterations(1000);
	// gtsam::DoglegOptimizer optimizer(graph, init_values_, parameters);

	// graph.print();
	// init_values_.print();
	// parameters.print();


	optimizer.optimize();
	batch_values_ = optimizer.values();

	ROS_INFO("Batch GPMP2 optimization complete.");

	  // publish trajectory for visualization or other use
  	if (traj_.plan_traj_pub)
    	traj_.publishPlannedTrajectory(batch_values_, problem_, 0);
}


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
  exec_step = problem_.total_step + problem_.control_inter * (problem_.total_step - 1);

	// write_traj_ = true;
	execute_traj_ = true;
  if (write_traj_) 
    traj_.writeTrajectory(exec_values_, problem_, exec_step);
  if (execute_traj_)
    traj_.executeTrajectory(exec_values_, problem_, exec_step);
}

/* ************************************************************************** */





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



/* ************************************************************************** */

int main(int argc, char** argv)
{
	ros::init(argc, argv, "constrained_manipulation_server",  ros::init_options::NoSigintHandler);
    signal(SIGINT, piper::sigintHandler);

	ros::NodeHandle nh("piper");


	piper::ConstrainedManipulator manipulator(nh);

	ros::spin();

	ros::shutdown();

	return EXIT_SUCCESS;
}
