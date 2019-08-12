#include <ros/ros.h>
#include <perception_msgs/SystGetCentroid.h>
//#include <central_control/SystSendGoal.h>
#include <perception_msgs/SystYOLO_SetState.h>
//#include <moveit/move_group_interface/move_group_interface.h>
//#include <map>
//#include <vector>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <play_motion_msgs/PlayMotionAction.h>
#include <play_motion_msgs/PlayMotionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <gripper_stiffness_controller/gripper_control.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

#define THR_APPR_DIST 0.005
#define THR_EQI_VEL 0.01 
#define THR_EQI_PACC 0.1 

class CentralControl{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  //bool APPR_CALLED,EQI_CALLED;

  double torso_joint, arm_1_joint, arm_2_joint, arm_3_joint, arm_4_joint, arm_5_joint, arm_6_joint, arm_7_joint;
  
  void GetCurrentState(const trajectory_msgs::JointTrajectory::ConstPtr& msg_curr_state);

public:
	ros::ServiceClient client_GetGoal; 
	ros::ServiceClient client_SetState;
	ros::Publisher torso_cmd;
	ros::Publisher goal_cmd;
	
	ros::Subscriber sub_CurrentState;
	ros::ServiceClient client_Approach_Start;
	ros::ServiceClient client_Approach_Stop;
	ros::ServiceClient client_Reach;
	
	geometry_msgs::PointStamped goal;
	geometry_msgs::Point centroid; 
	geometry_msgs::Vector3 vel_EndEff;
	geometry_msgs::Point pos_EndEff;
	bool GOAL_RECEIVED;
	bool GOAL_REACHED;
	
	ros::Publisher pub_debug_centroid;
  //bool pre_grasp();

  CentralControl(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
  	GOAL_RECEIVED = false;
  	//APPR_CALLED,EQI_CALLED = false;
  	GOAL_REACHED = false;
    client_GetGoal = nh_.serviceClient<perception_msgs::SystGetCentroid>("/perception/GetCentroid");
    client_SetState = nh_.serviceClient<perception_msgs::SystYOLO_SetState>("/perception/yolo_enable");
    torso_cmd = nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1);
    goal_cmd = nh_.advertise<geometry_msgs::PointStamped>("/control/goal_pose", 1);
    
    sub_CurrentState = nh.subscribe<trajectory_msgs::JointTrajectory>("/cartesian_controller/curr_cart_pos_vel", 10, &CentralControl::GetCurrentState, this);
	client_Approach_Start = nh.serviceClient<std_srvs::Empty>("/pos_correction_start",10);
	client_Approach_Stop = nh.serviceClient<std_srvs::Empty>("/pos_correction_stop",10);
	client_Reach = nh.serviceClient<gripper_stiffness_controller::gripper_control>("/gripper_control",10);
	
	pub_debug_centroid = nh.advertise<geometry_msgs::PointStamped>("/debug_main/centroid",100);
	
	goal.header.frame_id = "torso_lift_link";
    /*
    priv_nh_.getParam("/torso_joint", torso_joint);
    priv_nh_.getParam("/arm_1_joint", arm_1_joint);
    priv_nh_.getParam("/arm_2_joint", arm_2_joint);
    priv_nh_.getParam("/arm_3_joint", arm_3_joint);
    priv_nh_.getParam("/arm_4_joint", arm_4_joint);
    priv_nh_.getParam("/arm_5_joint", arm_5_joint);
    priv_nh_.getParam("/arm_6_joint", arm_6_joint);
    priv_nh_.getParam("/arm_7_joint", arm_7_joint);
    */
   }
};

/*
bool CentralControl::pre_grasp(){


  std::map<std::string, double> target_position;

  target_position["arm_1_joint"] = arm_1_joint;
  target_position["arm_2_joint"] = arm_2_joint;
  target_position["arm_3_joint"] = arm_3_joint;
  target_position["arm_4_joint"] = arm_4_joint;
  target_position["arm_5_joint"] = arm_5_joint;
  target_position["arm_6_joint"] = arm_6_joint;
  target_position["arm_7_joint"] = arm_7_joint;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> torso_arm_joint_names;

  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");

  group_arm_torso.setPlannerId("SBLkConfigDefault");

  torso_arm_joint_names = group_arm_torso.getJoints();

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
  {
    if( target_position.count(torso_arm_joint_names[i]) > 0 )
    {
      ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << "goal position: " << target_position[torso_arm_joint_names[i]]);
      group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
    }
  }

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group_arm_torso.setPlanningTime(15.0);

  bool success = (group_arm_torso.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if( !success )
      throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  ros::Time start = ros::Time::now();

  group_arm_torso.move();
  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

  spinner.stop();

  return true;

}
*/

void CentralControl::GetCurrentState(const trajectory_msgs::JointTrajectory::ConstPtr& msg_curr_state){
	if(GOAL_RECEIVED){
		float tmp_pacc; //pseudo-acceleartion
		float tmp_pvel;
		float distance;
		
		geometry_msgs::Vector3 vel_EndEff_old;
	
		vel_EndEff_old = vel_EndEff;
	
		pos_EndEff.x = (msg_curr_state->points[0]).positions[0];
		pos_EndEff.y = (msg_curr_state->points[0]).positions[1];
		pos_EndEff.z = (msg_curr_state->points[0]).positions[2];

		vel_EndEff.x = (msg_curr_state->points[0]).velocities[0];
		vel_EndEff.y = (msg_curr_state->points[0]).velocities[1];
		vel_EndEff.z = (msg_curr_state->points[0]).velocities[2];
	
		tmp_pacc = 1/3*(abs(vel_EndEff.x - vel_EndEff_old.x)+abs(vel_EndEff.y - vel_EndEff_old.y)+abs(vel_EndEff.z - vel_EndEff_old.z));
		tmp_pvel = 1/3*(abs(vel_EndEff.x)+abs(vel_EndEff.y)+abs(vel_EndEff.z));
	
		distance = sqrt(pow(pos_EndEff.x-centroid.x,2)+pow(pos_EndEff.y-centroid.y,2)+pow(pos_EndEff.z-centroid.z,2));
    
    	//if(distance < THR_APPR_DIST){
    		//std_srvs::Empty msg_Empty;
    		//client_Approach_Start.call(msg_Empty);
    		//APPR_CALLED = true;
    	//}
    
    	if(tmp_pacc < THR_EQI_PACC && tmp_pvel < THR_EQI_VEL){//reached the goal
    		GOAL_REACHED = true;
    		//gripper_stiffness_controller::gripper_control msg_grasp;
    		//msg_grasp.request.command = "grip";
    		//client_Reach.call(msg_grasp);
    		//EQI_CALLED = true;
    	}
    }else{
    	//ROS_WARN("No Goal Received! Skip Approach Estimation.");
    }
}



int main(int argc, char** argv)
{
   ros::init(argc, argv, "central_control");
   ros::NodeHandle nh;
   CentralControl node(nh);
   perception_msgs::SystGetCentroid srvMsg_GetCentroid;
   perception_msgs::SystYOLO_SetState srvMsg_SetState;
   //central_control::SystSendGoal srvMsg_SendGoal;
   
   actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> ac("/play_motion", true);
   ROS_INFO("Waiting for action server to start.");
   ac.waitForServer(); //will wait for infinite time
   ROS_INFO("Action server started.");
   play_motion_msgs::PlayMotionGoal pmg;
    
   bool result;
   
   ros::AsyncSpinner spinner(2);
   spinner.start();
   
   // 1. LIFT TORSO
   ROS_INFO("Moving up torso.");
   trajectory_msgs::JointTrajectory jt;
   jt.joint_names = {"torso_lift_joint"};
   trajectory_msgs::JointTrajectoryPoint jtp;
   jtp.positions = {0.34};
   jtp.time_from_start = ros::Duration(2.5);
   jt.points.push_back(jtp);
   node.torso_cmd.publish(jt);
   
   ros::Duration(3.0).sleep();
   
   
   // 2. CALL OPEN DETECTION SERVICE
   srvMsg_GetCentroid.request.Name = "orange";
   srvMsg_SetState.request.state = 1;
   while(true)
   {
     result = node.client_SetState.call(srvMsg_SetState);
     if(result){
         break;
         }
     else{
         ROS_INFO("call open detection service failed");
         }
   }
   ROS_INFO("open yolo detection done");
   
   /*
   //3.LOOK AROUND
   pmg.motion_name = "head_look_around";
   pmg.skip_planning = false;
   ac.sendGoalAndWait(pmg);
   ROS_INFO("head_look_around done.");
   */
   ros::Duration(5.0).sleep();
   srvMsg_GetCentroid.request.Name = "orange";
   //4. CALL GET GOAL SERVICE TO GET THE GOAL, LOOP UNTIL CALL SUCCEEDS
   while(true)
   {
   	 result = node.client_GetGoal.call(srvMsg_GetCentroid);
   	 //srvMsg_GetCentroid.request.Name = "orange";
     if(result){
       node.centroid = srvMsg_GetCentroid.response.centroid.point;
       //node.pub_debug_centroid.publish(srvMsg_GetCentroid.response.centroid);
       node.GOAL_RECEIVED = true;
       break;
     }else{
       ROS_INFO("there is no orange! try to find apple");
       srvMsg_GetCentroid.request.Name = "apple";
     }
     ros::Duration(0.5).sleep();
   }
   //ROS_INFO("goal position got");
   ROS_INFO_STREAM("goal position got: ["<<node.centroid.x<<", "<<node.centroid.y<<", "<<node.centroid.z<<"]");

   //5.PRE_GRASP
   gripper_stiffness_controller::gripper_control msg_grasp;
   ROS_INFO("open the gripper.");
   msg_grasp.request.command = "release";
   node.client_Reach.call(msg_grasp);
   
   pmg.motion_name = "pregrasp";
   pmg.skip_planning = true;
   ac.sendGoalAndWait(pmg);
   
   ROS_INFO("move arm to pregrasp done.");
   
   //6.CLOSE YOLO
   srvMsg_SetState.request.state = 0;
   while(true)
   {
     result = node.client_SetState.call(srvMsg_SetState);
     if(result)
         break;
     else
         ROS_INFO("call close detection service failed");
   }
   ROS_INFO("close yolo detection done");
   
   //7. SEND APPRO GOAL TO CONTROLLER

   node.goal.point.x = node.centroid.x - 0.3;
   node.goal.point.y = node.centroid.y -0.05;
   node.goal.point.z = node.centroid.z -0.025;

   node.goal.header.stamp = ros::Time::now();
   
   node.goal_cmd.publish(node.goal);
   
   while(!node.GOAL_REACHED);
   
   
   ROS_INFO("approach the target");
   
   std_srvs::Empty msg_Empty;
   node.client_Approach_Start.call(msg_Empty);
   node.GOAL_REACHED = false;
   
   //8. SEND GOAL TO CONTROLLER
   node.goal.point.x = node.centroid.x - 0.1;
   node.goal.point.y = node.centroid.y - 0.05;
   node.goal.point.z = node.centroid.z -0.05;
   //node.goal.point.x = node.centroid.x;
   node.goal.header.stamp = ros::Time::now();
   
   node.goal_cmd.publish(node.goal);
   
   while(!node.GOAL_REACHED);
   
   ROS_INFO("reach the target");
   ros::Duration(8.0).sleep();
   //node.client_Approach_Stop.call(msg_Empty);
   
   ROS_INFO("begin to grasp.");
   msg_grasp.request.command = "grip";
   node.client_Reach.call(msg_grasp);
   
   ros::Duration(2).sleep();
   ROS_INFO("grasp done. Move the object to another position");
   
   node.goal.point.x = node.goal.point.x - 0.1;
   node.goal.point.y = node.goal.point.y;
   node.goal.point.z = node.goal.point.z + 0.1;
   node.goal.header.stamp = ros::Time::now();
   node.goal_cmd.publish(node.goal);
   
   ros::waitForShutdown();
  
   return 0;
}
