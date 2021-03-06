#ifndef CONSTRAINED_MANIPULATOR_H
#define CONSTRAINED_MANIPULATOR_H
#include <bits/stdc++.h> 

#include <actionlib/server/simple_action_server.h>
#include <piper/ConstrainedManipulationAction.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>


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

#include <hlpr_trac_ik/IKHandler.h>


#include <problem.h>
#include <traj.h>
#include <misc.h>

typedef actionlib::SimpleActionServer<piper::ConstrainedManipulationAction> Server;


namespace piper {

class ConstrainedManipulator
{
public:
	// ConstrainedManipulator() {}
    ConstrainedManipulator(ros::NodeHandle nh_);

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
    bool execute_traj_, write_traj_, use_current_start_conf, find_goal_conf; 
    
    //NOTE: HACK FOR FETCH ONLY!
    std::vector<double> base_shoulder_offset_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    // 



	// ros::NodeHandle nh_;
	void armStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void executeCallback(const piper::ConstrainedManipulationGoalConstPtr &goal);
    void execute();


    actionlib::SimpleActionServer<piper::ConstrainedManipulationAction> constrained_manipulation_server_;
    ros::ServiceClient trac_ik_client_;

};

}

#endif  // CONSTRAINED_MANIPULATOR_H