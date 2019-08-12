/*--------------------------------------------------*/
/*
message tf conversion node
Verision: 0.5
to show the effect of stiffness control at "offer_gripper" position, "GOAL_INITIALIZED" and "CURRENT_STATES_INITIALIZED"
should be initialized to true
*/
/*--------------------------------------------------*/

#include <ros/ros.h>
#include <tf/transform_listener.h>

//#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <stiffness_control_msgs/WristDataUpdate.h>
//#include <gripper_stiffness_controller/gripper_control.h>
//#include <std_srvs/Empty.h>

#include <math.h> 

#define OFFER_GRIPPER_POS_X 0.735093
#define OFFER_GRIPPER_POS_Y 0.0520262
#define OFFER_GRIPPER_POS_Z -0.175099

//#define THR_APPR_DIST 0.1
//#define THR_EQI_VEL 0.001 
//#define THR_EQI_PACC 0.01 
#define THR_RANGE 1.1

class MessageTransform{
private:
	ros::Subscriber sub_CurrentState;
	ros::Subscriber sub_GoalPose;
	ros::Subscriber sub_wrist_ft;
	ros::Subscriber sub_GoalOffset;
	ros::Subscriber sub_AccOffset;
	ros::ServiceServer srv_SendData;
	
	//ros::ServiceClient client_Approach;
	//ros::ServiceClient client_Reach;
	
	tf::TransformListener listener;
	
	geometry_msgs::Vector3Stamped vel_EndEff;
	geometry_msgs::PointStamped pos_EndEff;
	geometry_msgs::PointStamped desired_goal_pose;
	geometry_msgs::Vector3Stamped ft_force;
	geometry_msgs::Vector3Stamped acc_offset;
	geometry_msgs::Vector3Stamped goal_offset;
	
	/*-----     Flags     -----*/
	bool GOAL_INITIALIZED;
	bool CURRENT_STATES_INITIALIZED;
	//bool APPR_CALLED,EQI_CALLED;
	
	/*-----   Callbacks   -----*/
	void GetCurrentState(const trajectory_msgs::JointTrajectory::ConstPtr& msg_curr_state);
	void GetGoalPose(const geometry_msgs::PointStamped::ConstPtr& msg_goal_pose);
	void GetFTData(const geometry_msgs::WrenchStamped::ConstPtr& msg_ft_data);
	void GetAccelerationOffset(const geometry_msgs::Vector3Stamped::ConstPtr& msg_acc_offset);
	void GetGoalOffset(const geometry_msgs::Vector3Stamped::ConstPtr& msg_goal_offset);
	bool SendData(stiffness_control_msgs::WristDataUpdate::Request &req, stiffness_control_msgs::WristDataUpdate::Response &res);
	
public:

	MessageTransform(ros::NodeHandle nh){
		GOAL_INITIALIZED = false;
		CURRENT_STATES_INITIALIZED = false;
		//APPR_CALLED,EQI_CALLED = false;
		//desired_goal_pose.pose.position.x = 0.735093;
		//desired_goal_pose.pose.position.y = 0.0520262;
		//desired_goal_pose.pose.position.z = -0.175099;
		
		//pos_EndEff.point.x = 0.735093;
		//pos_EndEff.point.y = 0.0520262;
		//pos_EndEff.point.z = -0.175099;
		
		sub_CurrentState = nh.subscribe<trajectory_msgs::JointTrajectory>("/cartesian_controller/curr_cart_pos_vel", 10, &MessageTransform::GetCurrentState, this);
		sub_GoalPose = nh.subscribe<geometry_msgs::PointStamped>("/control/goal_pose", 10, &MessageTransform::GetGoalPose, this);
		sub_wrist_ft = nh.subscribe<geometry_msgs::WrenchStamped>("/wrist_ft", 1, &MessageTransform::GetFTData, this);
		sub_AccOffset = nh.subscribe<geometry_msgs::Vector3Stamped>("/control/acc_offset", 10, &MessageTransform::GetAccelerationOffset, this);
		sub_GoalOffset = nh.subscribe<geometry_msgs::Vector3Stamped>("/control/goal_offset", 10, &MessageTransform::GetGoalOffset, this);
		srv_SendData = nh.advertiseService("/control/wrist_stiffness/update", &MessageTransform::SendData, this);
		
		//client_Approach = nh.serviceClient<std_srvs::Empty>("/position_correction",10);
		//client_Reach = nh.serviceClient<gripper_stiffness_controller::gripper_control>("/gripper_control",10);
	}
};

void MessageTransform::GetCurrentState(const trajectory_msgs::JointTrajectory::ConstPtr& msg_curr_state){
	CURRENT_STATES_INITIALIZED = true;
	//float tmp_pacc; //pseudo-acceleartion
	//float tmp_pvel;
	geometry_msgs::Vector3Stamped tmp_vel;
	//geometry_msgs::Vector3Stamped vel_old;
	geometry_msgs::PointStamped tmp_pos;
	
	//float distance;

	tmp_pos.header = tmp_vel.header=  msg_curr_state->header;
	
	//vel_old = vel_EndEff;	
	
	tmp_pos.point.x = (msg_curr_state->points[0]).positions[0];
	tmp_pos.point.y = (msg_curr_state->points[0]).positions[1];
	tmp_pos.point.z = (msg_curr_state->points[0]).positions[2];

	tmp_vel.vector.x = (msg_curr_state->points[0]).velocities[0];
	tmp_vel.vector.y = (msg_curr_state->points[0]).velocities[1];
	tmp_vel.vector.z = (msg_curr_state->points[0]).velocities[2];
	
	//no conversion in this version
	try{
		//listener.waitForTransform("torso_lift_link","torso_lift_link",tmp_pos.header.stamp,ros::Duration(1));
    	listener.transformPoint("torso_lift_link", tmp_pos, pos_EndEff);
    	listener.transformVector("torso_lift_link", tmp_vel, vel_EndEff);
  	}catch(tf::TransformException& ex){
    	ROS_ERROR("Received an exception trying to transform a point from \"torso_lift_link\" to \"torso_lift_link\": %s", ex.what());
    }
    /*
    tmp_pacc = 1/3*(abs(tmp_vel.vector.x - vel_old.vector.x)+abs(tmp_vel.vector.y - vel_old.vector.y)+abs(tmp_vel.vector.y - vel_old.vector.y));
	tmp_pvel = 1/3*(abs(tmp_vel.vector.x)+abs(tmp_vel.vector.y)+abs(tmp_vel.vector.z));
    
    distance = sqrt(pow(tmp_pos.point.x-desired_goal_pose.pose.position.x,2)+pow(tmp_pos.point.y-desired_goal_pose.pose.position.y,2)+pow(tmp_pos.point.z-desired_goal_pose.pose.position.z,2));
    
    if(distance < THR_APPR_DIST && !APPR_CALLED){
    	std_srvs::Empty msg_Empty;
    	client_Approach.call(msg_Empty);
    	APPR_CALLED = true;
    }
    
    if(tmp_pacc < THR_EQI_PACC && tmp_pvel < THR_EQI_VEL && !EQI_CALLED){//reached the goal
    	gripper_stiffness_controller::gripper_control msg_grasp;
    	msg_grasp.request.command = "grip";
    	client_Reach.call(msg_grasp);
    	EQI_CALLED = true;
    }
    */
}

void MessageTransform::GetGoalPose(const geometry_msgs::PointStamped::ConstPtr& msg_goal_pose){
	GOAL_INITIALIZED = true;
	geometry_msgs::PointStamped tmp_goal;
	try{
		listener.waitForTransform("torso_lift_link","torso_lift_link",msg_goal_pose->header.stamp,ros::Duration(1));
    	listener.transformPoint("torso_lift_link", *msg_goal_pose, tmp_goal);
  	}catch(tf::TransformException& ex){
    	ROS_ERROR("Received an exception trying to transform a point from \"torso_lift_link\" to \"torso_lift_link\": %s", ex.what());
    }
    
    if( sqrt(pow(tmp_goal.point.x,2)+pow(tmp_goal.point.y,2)+pow(tmp_goal.point.z,2)) >= THR_RANGE){
    	ROS_ERROR("Out of range!");
    }else{
    	desired_goal_pose = tmp_goal;
    }
    
}

void MessageTransform::GetFTData(const geometry_msgs::WrenchStamped::ConstPtr& msg_ft_data){
	geometry_msgs::Vector3Stamped tmp_force;
	tmp_force.header = msg_ft_data->header;
	//tmp_force.header.stamp = ros::Time::now();
	
	//ft sensor inner stress compensation
	tmp_force.vector.x = msg_ft_data->wrench.force.x + 2.45;
	tmp_force.vector.y = msg_ft_data->wrench.force.y + 4.35;
	tmp_force.vector.z = msg_ft_data->wrench.force.z - 3.30;
	
	//tf::StampedTransform transform;
	
	try{
		listener.waitForTransform("wrist_ft_link","torso_lift_link",tmp_force.header.stamp,ros::Duration(1));
    	listener.transformVector("torso_lift_link", tmp_force, ft_force);
  	}catch(tf::TransformException& ex){
    	ROS_ERROR("Received an exception trying to transform a point from \"wrist_ft_link\" to \"torso_lift_link\": %s", ex.what());
    }
}

void MessageTransform::GetAccelerationOffset(const geometry_msgs::Vector3Stamped::ConstPtr& msg_acc_offset){
	std::string source_link = msg_acc_offset->header.frame_id;
	try{
		listener.waitForTransform(source_link,"torso_lift_link",msg_acc_offset->header.stamp,ros::Duration(1));
    	listener.transformVector("torso_lift_link", *msg_acc_offset, acc_offset);
  	}catch(tf::TransformException& ex){
    	ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"torso_lift_link\": %s", source_link.c_str(), ex.what());
    }
}

void MessageTransform::GetGoalOffset(const geometry_msgs::Vector3Stamped::ConstPtr& msg_goal_offset){
	std::string source_link = msg_goal_offset->header.frame_id;
	try{
		listener.waitForTransform(source_link,"torso_lift_link",msg_goal_offset->header.stamp,ros::Duration(1));
    	listener.transformVector("torso_lift_link", *msg_goal_offset, goal_offset);
  	}catch(tf::TransformException& ex){
    	ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"torso_lift_link\": %s", source_link.c_str(), ex.what());
    }
}

bool MessageTransform::SendData(stiffness_control_msgs::WristDataUpdate::Request &req, stiffness_control_msgs::WristDataUpdate::Response &res){
	
	if(!CURRENT_STATES_INITIALIZED){
		ROS_WARN("Current states have not been initialized! The program will try again later.");
		return false;
	}else{
		res.vel_EndEff = vel_EndEff;
		res.pos_EndEff = pos_EndEff;
	}
	
	if(!GOAL_INITIALIZED){
	
		/*
		ROS_WARN("There is no goal received! Try to maintain the pre-defined pose \"offer_gripper\".");
		
		//res.desired_goal_pose.header = pos_EndEff.header;
		res.desired_goal_pose.pose.position.x = OFFER_GRIPPER_POS_X;
		res.desired_goal_pose.pose.position.y = OFFER_GRIPPER_POS_Y;
		res.desired_goal_pose.pose.position.z = OFFER_GRIPPER_POS_Z;
		*/
		
		ROS_WARN("There is no goal received! Do nothing.");
		return false;
		
	}else{
		res.desired_goal_pose.pose.position = desired_goal_pose.point;
	}
	
	res.ft_force = ft_force;
	res.acc_offset = acc_offset;
	res.goal_offset = goal_offset;
	
	return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "msg_tf_node");
    ros::NodeHandle nh;
    MessageTransform n(nh);
    ros::MultiThreadedSpinner spinner(6); 
	spinner.spin(); // spin() will not return until the node has been shutdown
    return 0;
}
