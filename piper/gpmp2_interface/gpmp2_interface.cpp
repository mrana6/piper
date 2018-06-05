/**
 *  @file   gpmp2_interface.cpp
 *  @brief  ROS interface between GPMP2 and a real/sim robot
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 **/

#include <gpmp2_interface.h>


namespace piper {

/* ************************************************************************** */
GPMP2Interface::GPMP2Interface(ros::NodeHandle nh)
{
  // first load problem and setup trajectory client
  problem_ = Problem(nh);
  traj_ = Traj(nh);

  // robot state subscriber (used to initialize start state if not passed as param)
  if (nh.hasParam("robot/arm_state_topic"))
  {
    nh.getParam("robot/arm_state_topic", arm_state_topic_);
    arm_state_sub_ = nh.subscribe(arm_state_topic_, 1, &GPMP2Interface::armStateCallback, this);
    arm_pos_ = gtsam::Vector::Zero(problem_.robot.getDOFarm());
    arm_pos_time_ = ros::Time::now();
  }
  if (problem_.robot.isMobileBase() && nh.hasParam("robot/base_state_topic"))
  {
    nh.getParam("robot/base_state_topic", base_state_topic_);
    base_state_sub_ = nh.subscribe(base_state_topic_, 1, &GPMP2Interface::baseStateCallback, this);
    base_pos_ = gtsam::Pose2();
    base_pos_time_ = ros::Time::now();
  }
  ros::Duration(1.0).sleep();

  // get start from measurement if not passed as param
  if (!nh.hasParam("start_conf"))
  {
    problem_.start_conf = arm_pos_;
    if (problem_.robot.isThetaNeg())
      problem_.robot.negateTheta(problem_.start_conf);
  }
  if (problem_.robot.isMobileBase())
  {
    if (!nh.hasParam("start_pose"))
      problem_.start_pose = base_pos_;
    problem_.pstart = gpmp2::Pose2Vector(problem_.start_pose, problem_.start_conf);
  }
  if (nh.hasParam("traj_file")) {
    std::string traj_file_;
    nh.getParam("traj_file", traj_file_);
    bool read_conf_ = false;
    if (nh.hasParam("read_conf")) 
      nh.getParam("read_conf", read_conf_);
    // get traj from file
    ROS_INFO("Loading trajectory from file...");
    problem_.readTrajectory(traj_file_,read_conf_);
  }
  
  // initialize trajectory
  traj_.initializeTrajectory(init_values_, problem_);

  // solve with batch gpmp2
  ROS_INFO("Optimizing...");
  int DOF = problem_.robot.getDOF();
  if (nh.hasParam("traj_file")) {
    /*std::string traj_file_;
    nh.getParam("traj_file", traj_file_);
    bool read_conf_ = false;
    if (nh.hasParam("read_conf")) {
      std::cout<<"has param"<<std::endl;
      nh.getParam("read_conf", read_conf_);
    } else
      std::cout<<"no param"<<std::endl;
    nh.getParam("fix_cartpose_sigma", fix_cartpose_sigma_);
    nh.getParam("fix_cartorient_sigma", fix_cartorient_sigma_);
    // get traj from file
    ROS_INFO("Loading trajectory from file...");
    problem_.readTrajectory(traj_file_,read_conf_);*/
    nh.getParam("fix_cartpose_sigma", fix_cartpose_sigma_);
    nh.getParam("fix_cartorient_sigma", fix_cartorient_sigma_);
    int total_time_step = problem_.traj.size() - 1;
    ROS_INFO("Total time step is %i", total_time_step);
    if (!problem_.robot.isMobileBase()) {
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
        gtsam::Rot3 orient_corrected = orient * gtsam::Rot3::Quaternion(0.5, -0.5, 0.5, -0.5);
        graph.add(gpmp2::GaussianPriorWorkspaceOrientationArm(key_pos, problem_.robot.arm, DOF-1, orient_corrected, cartorient_model));

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

    } else {

    }
  } else {
    if (!problem_.robot.isMobileBase())
      batch_values_ = gpmp2::BatchTrajOptimize3DArm(problem_.robot.arm, problem_.sdf, problem_.start_conf, 
        gtsam::Vector::Zero(DOF), problem_.goal_conf, gtsam::Vector::Zero(DOF), init_values_, problem_.opt_setting);
    else
      batch_values_ = gpmp2::BatchTrajOptimizePose2MobileArm(problem_.robot.marm, problem_.sdf, problem_.pstart, 
        gtsam::Vector::Zero(DOF), problem_.pgoal, gtsam::Vector::Zero(DOF), init_values_, problem_.opt_setting);
  }
  ROS_INFO("Batch GPMP2 optimization complete.");

  // publish trajectory for visualization or other use
  if (traj_.plan_traj_pub)
    traj_.publishPlannedTrajectory(batch_values_, problem_, 0);
}

/* ************************************************************************** */
void GPMP2Interface::execute()
{
  size_t exec_step;
  double coll_cost;

  // interpolate batch solution to a desired resolution for control and check for collision
  ROS_INFO("Checking for collision.");
  if (!problem_.robot.isMobileBase())
  {
    exec_values_ = gpmp2::interpolateArmTraj(batch_values_, problem_.opt_setting.Qc_model, problem_.delta_t, 
      problem_.control_inter, 0, problem_.total_step-1);
    coll_cost = gpmp2::CollisionCost3DArm(problem_.robot.arm, problem_.sdf, exec_values_, problem_.opt_setting);
  }
  else
  {
    exec_values_ = gpmp2::interpolatePose2MobileArmTraj(batch_values_, problem_.opt_setting.Qc_model, 
      problem_.delta_t, problem_.control_inter, 0, problem_.total_step-1);
    coll_cost = gpmp2::CollisionCostPose2MobileArm(problem_.robot.marm, problem_.sdf, exec_values_, problem_.opt_setting);
  }
  if (coll_cost != 0)
  {
    ROS_FATAL("Plan is not collision free! Collision cost = %.3f", coll_cost);
    sigintHandler(0);
  }

  //  execute trajectory
  ROS_INFO("Executing GPMP2 planned trajectory open-loop...");
  exec_step = problem_.total_step+problem_.control_inter*(problem_.total_step-1);
  traj_.executeTrajectory(exec_values_, problem_, exec_step);
}

/* ************************************************************************** */
void GPMP2Interface::armStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
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

/* ************************************************************************** */
void GPMP2Interface::baseStateCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  base_pos_ = gtsam::Pose2(msg->position.x, msg->position.y, gtsam::Rot3::Quaternion(msg->orientation.w, 
    msg->orientation.x, msg->orientation.y, msg->orientation.z).yaw());
  base_pos_time_ = ros::Time::now();
}

} // piper namespace


/* ************************************************************************** */
/* main callback */
void mainCallback(const std_msgs::Bool::ConstPtr& msg)
{
  ros::NodeHandle nh("piper");
  piper::GPMP2Interface gpmp2(nh);
  gpmp2.execute();
  ROS_INFO("Done.");
  ros::shutdown();
}

/* ************************************************************************** */
/* main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "gpmp2_interface");
  signal(SIGINT, piper::sigintHandler);
  ros::MultiThreadedSpinner spinner(0);

  ros::NodeHandle n;
  ros::Publisher main_pub = n.advertise<std_msgs::Bool>("/piper/run_main", 1);
  ros::Subscriber main_sub = n.subscribe("/piper/run_main", 1, mainCallback);
  main_pub.publish(std_msgs::Bool());

  spinner.spin();
}
