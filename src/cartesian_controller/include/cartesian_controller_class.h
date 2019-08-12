#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
//#include <geometry_msgs/Point.h>

// KDL INCLUDE
//#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
//#include <kdl/jntarrayvel.hpp>


#include <kdl/chainiksolverpos_lma.hpp>
//#include <kdl/chainiksolvervel_pinv_givens.hpp>

/*
// EIGEN INCLUDES
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>
//using namespace Eigen;
*/
#include <Eigen/QR>    

class CartesianController
{
private:
	ros::NodeHandle nh_;

	ros::Subscriber sub_curr_joint_pos, sub_goal_cart_pos;
	ros::Publisher pub_control_msg, pub_curr_cart_pos_vel;


	// position member
	KDL::JntArrayVel curr_joint_pos_vel;
	KDL::FrameVel curr_cart_pos_vel;
	
	KDL::JntArray goal_joint_pos, goal_joint_vel, local_goal_joint_pos, local_goal_joint_vel;
	
	KDL::Frame goal_cart_pos; 
	KDL::Twist goal_cart_vel;
	
	KDL::Rotation orientation;
	Eigen::MatrixXd A, A_pinv;  // peseudo inverse of jacobian matrix;
	

	bool curr_coming, goal_coming;
	bool goal_updated;



	trajectory_msgs::JointTrajectory control_msg, curr_pos_vel_msg;
	trajectory_msgs::JointTrajectoryPoint control_point, pub_curr_point;

	//forward kinematic and inverse kinematic solver
	KDL::ChainIkSolverPos_LMA IK_Solver;
	KDL::ChainFkSolverVel_recursive FK_Solver_Vel;
	
	
	void UpdateCurrJointPos(const sensor_msgs::JointState::ConstPtr &joint_states);
	void UpdateGoalJointPos(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &goal);
	//void UpdateGoalJointPos(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	
	//ros::Duration loc_time_from_start;
public:
	CartesianController(ros::NodeHandle nh, KDL::Chain chain);
	void move_to_goal();
	void publish_curr_cart_pos_vel();
};
