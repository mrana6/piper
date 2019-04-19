#ifndef CONSTRAINED_MANIPULATOR_H
#define CONSTRAINED_MANIPULATOR_H

#include <actionlib/server/simple_action_server.h>
#include <piper/ConstrainedManipulationAction.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <gpmp2/planner/BatchTrajOptimizer.h>
#include <gpmp2/planner/TrajUtils.h>
#include <gpmp2/kinematics/GaussianPriorWorkspacePositionArm.h>
#include <gpmp2/kinematics/GaussianPriorWorkspaceOrientationArm.h>
#include <gpmp2/gp/GaussianProcessPriorLinear.h>
#include <gpmp2/obstacle/ObstacleSDFFactorArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPArm.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <problem.h>
#include <traj.h>
#include <misc.h>

class ConstrainedManipulator
{
public:
	ConstrainedManipulator();

private:
    Problem problem_;
    Traj traj_;
    gtsam::Values init_values_, batch_values_, exec_values_;

    std::string arm_state_topic_, base_state_topic_, traj_file_;
    ros::Subscriber arm_state_sub_, base_state_sub_;
    gtsam::Vector arm_pos_;
    gtsam::Pose2 base_pos_;
    ros::Time arm_pos_time_, base_pos_time_;
    double fix_cartpose_sigma_, fix_cartorient_sigma_;
    bool execute_traj_, write_traj_;

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<piper::ConstrainedManipulationAction> constrained_manipulation_server_;
	void armStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
	void execute(const piper::ConstrainedManipulationGoalConstPtr &goal);
};

#endif  // CONSTRAINED_MANIPULATOR_H