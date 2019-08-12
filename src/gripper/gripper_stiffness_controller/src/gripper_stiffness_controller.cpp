#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
//#include <tum_ics_skin_msgs/SkinCellDataArray.h>
#include <gripper_stiffness_controller/gripper_control.h>
#include <pal_interaction_msgs/TtsActionGoal.h>


class GripperStiffnessController
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle priv_nh_;

	ros::Subscriber patch_3;
	ros::Subscriber patch_5;
	ros::Subscriber sub_CurrentState; //pos. vel. eff.
	ros::Publisher pub_CurrentGoalPose;
	ros::Publisher Text_to_Speech;
	ros::ServiceServer service;

	double pitch_3_temp, pitch_5_temp;
	bool flag_pitch_3_hot_pub, flag_pitch_5_hot_pub;
	bool flag_pitch_3_cold_pub, flag_pitch_5_cold_pub;
	bool flag_sensor;
    const float length_of_object = 0.06;


	float M_hard, K_hard, B_hard, M_soft, K_soft, B_soft, M_current, K_current, B_current;
	float pos_left, pos_right, vel_left, vel_right, acc_left, acc_right, eff_left, eff_right;
	trajectory_msgs::JointTrajectory goal;
	pal_interaction_msgs::TtsActionGoal speech;

	void GetCurrentState(const sensor_msgs::JointState::ConstPtr& msg_curr_state);
	bool ControlGripper(gripper_stiffness_controller::gripper_control::Request &req, gripper_stiffness_controller::gripper_control::Response &res);
	void SetGoal(float pos_left, float pos_right, float vel_left, float vel_right, ros::Duration dt);
	//void Get_prox_pathch_3(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch);
	//void Get_prox_pathch_5(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch);

public:
	float t;
	float pos_target_left, pos_target_right;
	float pos_gripper_left, pos_gripper_right, eff_gripper_left, eff_gripper_right, vel_gripper_left, vel_gripper_right;

	void ControlProcess();

	GripperStiffnessController(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
	{
		sub_CurrentState = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &GripperStiffnessController::GetCurrentState, this);
    	Text_to_Speech = nh_.advertise<pal_interaction_msgs::TtsActionGoal>("/tts/goal", 10);
		pub_CurrentGoalPose = nh_.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command",10);
    	service = nh_.advertiseService("gripper_control", &GripperStiffnessController::ControlGripper, this);
    	//patch_3 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch3", 10, &GripperStiffnessController::Get_prox_pathch_3, this);
    	//patch_5 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch5", 10, &GripperStiffnessController::Get_prox_pathch_5, this);

		priv_nh_.getParam("/gain/M_hard", M_hard);
		priv_nh_.getParam("/gain/K_hard", K_hard);
		priv_nh_.getParam("/gain/B_hard", B_hard);
		priv_nh_.getParam("/gain/M_soft", M_soft);
		priv_nh_.getParam("/gain/K_soft", K_soft);
		priv_nh_.getParam("/gain/B_soft", B_soft);

		speech.goal.rawtext.lang_id = "en_GB";
    	speech.goal.wait_before_speaking = 0.0;
		
		pitch_3_temp = 25;
		pitch_5_temp = 25;
		
		flag_pitch_3_hot_pub = false;
		flag_pitch_5_hot_pub = false;
		flag_pitch_3_cold_pub = false;
		flag_pitch_5_cold_pub = false;
		flag_sensor = false;

		M_current = M_hard;
		K_current = K_hard;
		B_current = B_hard;

		vel_left = 0;
		vel_right = 0;

		goal.joint_names.push_back("gripper_left_finger_joint");
		goal.joint_names.push_back("gripper_right_finger_joint");
		goal.points.resize(1);
		goal.points[0].positions.resize(2);
		goal.points[0].velocities.resize(2);

		t = 0.02;

		pos_target_left = 0.04;
		pos_target_right = 0.04;

		ROS_INFO("Gripper Controller initialization done!");
	}

	~GripperStiffnessController()
	{

	}
};

/*
void GripperStiffnessController::Get_prox_pathch_3(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  pitch_3_temp = data_patch->data.data()->temp[0];

  if(pitch_3_temp > 35 && flag_pitch_3_hot_pub == false && flag_pitch_5_hot_pub == false)
  {
    ROS_INFO_STREAM("TEMP IS HIGHER THAN 20!!!!!");
    speech.goal.rawtext.text = "The water is too hot";
    Text_to_Speech.publish(speech);
    flag_pitch_3_hot_pub = true;
    flag_pitch_3_cold_pub = false;
  }

  if(pitch_3_temp < 20 && flag_pitch_3_cold_pub == false && flag_pitch_5_cold_pub == false)
  {
    ROS_INFO_STREAM("TEMP IS HIGHER THAN 20!!!!!");
    speech.goal.rawtext.text = "This water is too cold";
    Text_to_Speech.publish(speech);
    flag_pitch_3_cold_pub = true;
    flag_pitch_3_hot_pub = false;
  }
}

void GripperStiffnessController::Get_prox_pathch_5(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  pitch_5_temp = data_patch->data.data()->temp[0];
  if(pitch_5_temp > 35 && flag_pitch_3_hot_pub == false && flag_pitch_5_hot_pub == false)
  {
    speech.goal.rawtext.text = " Hot Hot Hot";
    Text_to_Speech.publish(speech);
    flag_pitch_5_hot_pub = true;
    flag_pitch_5_cold_pub = false;
  }

  if(pitch_5_temp < 20 && flag_pitch_3_cold_pub == false && flag_pitch_5_cold_pub == false)
  {
    speech.goal.rawtext.text = " Cold Cold Cold";
    Text_to_Speech.publish(speech);
    flag_pitch_5_cold_pub = true;
    flag_pitch_5_hot_pub = false;
  }
}
*/

void GripperStiffnessController::GetCurrentState(const sensor_msgs::JointState::ConstPtr& msg_curr_state)
{
	pos_gripper_left = (msg_curr_state->position)[7];
	pos_gripper_right = (msg_curr_state->position)[8];
	vel_gripper_left = (msg_curr_state->velocity)[7];
	vel_gripper_right = (msg_curr_state->velocity)[8];
	eff_gripper_left = (msg_curr_state->effort)[7];
	eff_gripper_right = (msg_curr_state->effort)[8];
	//ROS_INFO_STREAM("K="<<(fabs(eff_gripper_left) + fabs(eff_gripper_right)) / (length_of_object - pos_gripper_left -pos_gripper_right));
	if((fabs(eff_gripper_left) + fabs(eff_gripper_right)) / (length_of_object - pos_gripper_left -pos_gripper_right) > 20)
	{
		//ROS_INFO("Control Mode: Hard Control");
		M_current = M_hard;
		K_current = K_hard;
		B_current = B_hard;
	}
	else
	{
		//ROS_INFO("Control Mode: Soft Control");
		M_current = M_soft;
		K_current = K_soft;
		B_current = B_soft;
	}
	flag_sensor = true;
}

void GripperStiffnessController::SetGoal(float pos_left, float pos_right, float vel_left, float vel_right, ros::Duration dt)
{
	goal.points[0].positions[0] = pos_left;
	goal.points[0].positions[1] = pos_right;
	goal.points[0].velocities[0] = vel_left;
	goal.points[0].velocities[1] = vel_right;
	goal.points[0].time_from_start = dt;
  //goal.header.stamp = ros::Time::now();
}


bool GripperStiffnessController::ControlGripper(gripper_stiffness_controller::gripper_control::Request &req, gripper_stiffness_controller::gripper_control::Response &res)
{
	ros::Duration dt;
	if(req.command == "grip")
	{
		pos_target_left = 0.0;
		pos_target_right = 0.0;
		res.reply = true;
	}
	else if(req.command == "release")
	{
		pos_target_left = 0.04;
		pos_target_right = 0.04;
		res.reply = true;
	}
	else
	{
		ROS_INFO("command should be \"grip\" or \"release\"!");
		res.reply = false;
	}
	return true;
}

void GripperStiffnessController::ControlProcess()
{
	if(flag_sensor)
	{
		//ROS_INFO_STREAM("\npos_left= "<<pos_gripper_left<<"\npos_right= "<<pos_gripper_right);
		pos_left = pos_gripper_left;//open 0.04; closed 0;
		pos_right = pos_gripper_right;
		eff_left = -eff_gripper_left-K_current*(pos_gripper_left-pos_target_left)-B_current*vel_left;
		eff_right = -eff_gripper_right-K_current*(pos_gripper_right-pos_target_right)-B_current*vel_right;
		acc_left = 1/M_current * eff_left;
		acc_right = 1/M_current * eff_right;
		vel_left = vel_left + acc_left * t;
		vel_right = vel_right + acc_right * t;
		pos_left = pos_left + vel_left * t;
		pos_right = pos_right + vel_right * t;
		if((eff_left + eff_gripper_left) * (eff_left + eff_gripper_left) + (eff_right + eff_gripper_right)* (eff_right + eff_gripper_right) > 0.0003)
		{
			//ROS_INFO_STREAM("\npos_d_left= "<<pos_left<<"\npos_d_right= "<<pos_right);
			SetGoal(pos_left, pos_right, vel_left, vel_right, ros::Duration(t));
			pub_CurrentGoalPose.publish(goal);
		}
	}
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "gripper_controller");

	ros::NodeHandle nh;
	ros::Rate r(50);
	GripperStiffnessController node(nh);
	while (ros::ok())
	{
		node.ControlProcess();
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
