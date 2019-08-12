#include <cartesian_controller_class.h>

CartesianController::CartesianController(ros::NodeHandle nh, KDL::Chain chain) : nh_(nh), IK_Solver(chain), FK_Solver_Vel(chain)//, FK_Solver(chain), IK_Solver_Vel(chain)
{
		
	sub_curr_joint_pos = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &CartesianController::UpdateCurrJointPos, this);
	//sub_goal_cart_pos = nh_.subscribe<visualization_msgs::InteractiveMarkerFeedback>("/simple_marker/feedback", 1, &CartesianController::UpdateGoalJointPos, this);
	sub_goal_cart_pos = nh_.subscribe<trajectory_msgs::JointTrajectoryPoint>("/admittance_controller/goal", 1, &CartesianController::UpdateGoalJointPos, this);
	pub_control_msg = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);  //arm_controller/command
	pub_curr_cart_pos_vel = nh_.advertise<trajectory_msgs::JointTrajectory>("/cartesian_controller/curr_cart_pos_vel", 1);
	
	
	
	

	curr_coming = false;
	goal_coming = false;
	goal_updated = false;
	// fill necessary invariant parto of the control msg;
	control_msg.joint_names.push_back("arm_1_joint");
	control_msg.joint_names.push_back("arm_2_joint");
	control_msg.joint_names.push_back("arm_3_joint");
	control_msg.joint_names.push_back("arm_4_joint");
	control_msg.joint_names.push_back("arm_5_joint");
	control_msg.joint_names.push_back("arm_6_joint");
	control_msg.joint_names.push_back("arm_7_joint");

	control_point.time_from_start = ros::Duration(1);
	//loc_time_from_start = ros::Duration(0.1);
	control_point.positions.clear();
	control_point.velocities.clear();
	control_point.accelerations.clear();
	control_point.effort.clear();

    curr_joint_pos_vel.resize(chain.getNrOfJoints());
    SetToZero(curr_joint_pos_vel);
    
    local_goal_joint_pos = curr_joint_pos_vel.q;
    goal_joint_vel = curr_joint_pos_vel.qdot;
    local_goal_joint_vel = curr_joint_pos_vel.qdot;
    
    
	orientation = orientation.Quaternion(0.707872, 0.0492599, -0.0454885, 0.703152);
	//-0.00300049, -0.136514, 0.0175532, 0.990478  offer gripper orientation
	//0.69, 0.02, 0.72, 0.02                       home			 orientation
	//0.707872, 0.0492599, -0.0454885, 0.703152    defined pregrasp orientation

	goal_cart_pos.M = orientation;
	
	goal_cart_vel.rot = goal_cart_vel.rot.Zero();
	goal_cart_vel.vel = goal_cart_vel.vel.Zero();


	ROS_INFO("cartesian controller initialized.\n");
	
}

void CartesianController::UpdateCurrJointPos(const sensor_msgs::JointState::ConstPtr &joint_states)
{
	for(int i=0; i < 7; i++)
	{
		curr_joint_pos_vel.q.data[i] = joint_states->position[i];
		curr_joint_pos_vel.qdot.data[i] = joint_states->velocity[i];
	}
	curr_coming = true;
	//ROS_INFO("current joint value updated.");
}


void CartesianController::UpdateGoalJointPos(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &goal)
//void CartesianController::UpdateGoalJointPos(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	
	// position goal, keep orientation position unchanged in original orientation
	goal_cart_pos.p[0] = goal->positions[0];
	goal_cart_pos.p[1] = goal->positions[1];
	goal_cart_pos.p[2] = goal->positions[2];

	//std::cout<<"goal pos.: ["<<goal_cart_pos.p[0]<<", "<<goal_cart_pos.p[1]<<", "<<goal_cart_pos.p[2]<<"]"<<std::endl;
	
	// velocity goal, keep orientation velocity unchanged in zero
	goal_cart_vel.vel[0] = goal->velocities[0];
	goal_cart_vel.vel[1] = goal->velocities[1];
	goal_cart_vel.vel[2] = goal->velocities[2];
	std::cout<<"goal vel.: ["<<goal_cart_vel.vel[0]<<", "<<goal_cart_vel.vel[1]<<", "<<goal_cart_vel.vel[2]<<"]"<<std::endl;
	
	//loc_time_from_start = goal->time_from_start;
	//std::cout<<"goal acc.: ["<<goal->accelerations[0]<<", "<<goal->accelerations[1]<<", "<<goal->accelerations[2]<<"]"<<std::endl;
	
	/*
	goal_cart_pos.p[0] = feedback->pose.position.x;
	goal_cart_pos.p[1] = feedback->pose.position.y;
	goal_cart_pos.p[2] = feedback->pose.position.z;
	
	goal_cart_pos.M = goal_cart_pos.M.Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w ); 
	ROS_INFO_STREAM(feedback->pose.orientation.x<< " " << feedback->pose.orientation.y << " " << feedback->pose.orientation.z << " " << feedback->pose.orientation.w);
	*/
	//ROS_INFO("goal received.");
	
	goal_updated = true;
	goal_coming = true;
	move_to_goal();
}

void CartesianController::publish_curr_cart_pos_vel()
{
	if(!curr_coming)
	{
		ROS_INFO("Wait for joint data coming in.");
		return;
	}

	
	if(FK_Solver_Vel.JntToCart(curr_joint_pos_vel, curr_cart_pos_vel) < 0)
		ROS_ERROR("Jnt pos to Cartesian vel pos transformation failed. ");
		 
	else
	{ 	
		//ROS_INFO("Jnt pos to Cartesian vel pos transformation succeeded. ");
		pub_curr_point.positions.clear();
   		pub_curr_point.positions.push_back(curr_cart_pos_vel.p.p[0]);
   		pub_curr_point.positions.push_back(curr_cart_pos_vel.p.p[1]);
   		pub_curr_point.positions.push_back(curr_cart_pos_vel.p.p[2]);
   		
   		pub_curr_point.velocities.clear();
   		pub_curr_point.velocities.push_back(curr_cart_pos_vel.p.v[0]);
   		pub_curr_point.velocities.push_back(curr_cart_pos_vel.p.v[1]);
   		pub_curr_point.velocities.push_back(curr_cart_pos_vel.p.v[2]);
    	
    	curr_pos_vel_msg.points.clear();
    	curr_pos_vel_msg.points.push_back(pub_curr_point);
    	curr_pos_vel_msg.header.frame_id = "torso_lift_link";
    	curr_pos_vel_msg.header.stamp = ros::Time::now();
    	pub_curr_cart_pos_vel.publish(curr_pos_vel_msg);
    	

    	
    	std::cout 	<< "curr cart position: ["
    				<< curr_cart_pos_vel.p.p[0] << ", "
    				<< curr_cart_pos_vel.p.p[1] << ", "
    				<< curr_cart_pos_vel.p.p[2] << "]" << std::endl;
    	
    	double x, y, z, w;
   		curr_cart_pos_vel.M.R.GetQuaternion(x, y, z, w);
   		
   		std::cout 	<< "curr cart orientation in quaternion: ["
   				    << x << ", " << y << ", " << z << ", " << w << "]" << std::endl;
   		
   		
	}
}

void CartesianController::move_to_goal()
{
	if(!goal_coming)
	{
		ROS_INFO("goal has not come.");
		return;
	}
	if(goal_updated && curr_coming)
	{

		//int result = IK_Solver.CartToJnt(curr_joint_pos_vel.q, goal_cart_pos, goal_joint_pos);
		if(IK_Solver.CartToJnt(curr_joint_pos_vel.q, goal_cart_pos, goal_joint_pos) == IK_Solver.E_NOERROR)
			//&& IK_Solver_Vel.CartToJnt(curr_joint_pos_vel.q, goal_cart_vel, goal_joint_vel) == IK_Solver_Vel.E_NOERROR)
		{	
		
			
			using namespace Eigen;
			VectorXd error = goal_joint_pos.data - curr_joint_pos_vel.q.data;
			error = error.array().abs();
	
			std::cout << error << std::endl;
			
			VectorXd threshold_of_change;
			//threshold_of_change << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
			if((error.array() > 0.03).any())
			{
				ROS_ERROR("Change too much.Lets go die.");
				ros::shutdown();
			}
			 
			
			A = IK_Solver.jac;
			JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
			VectorXd singular_v; 
			singular_v = svd.singularValues();
			double pinvtoler = 1e-6;
			
			VectorXd singular_v_inv = singular_v;
			
			for(int i = 0; i < 6; i++)
			{
				if( singular_v(i) > pinvtoler)
					singular_v_inv(i) = 1.0/singular_v(i);
				else
					singular_v_inv(i) = 0;	
			}
				
			A_pinv = svd.matrixV() * singular_v_inv.asDiagonal() * svd.matrixU().transpose();
			//double r, p , y;
			//curr_cart_pos_vel.M.R.GetRPY(r, p, y);
			ROS_INFO("Calculating goal postion");
			VectorXd goal(6);
			goal[0] = goal_cart_vel.vel[0];
			goal[1] = goal_cart_vel.vel[1];
			goal[2] = goal_cart_vel.vel[2];
			goal[3] = goal_cart_vel.rot[0];
			goal[4] = goal_cart_vel.rot[1];
			goal[5] = goal_cart_vel.rot[2];
			
			//std::cout << goal << std::endl;

			VectorXd joint_goal = A_pinv * goal;
			//std::cout << joint_goal << std::endl;
			
			for(int i = 0; i < joint_goal.size(); i++)
				goal_joint_vel.data[i] = joint_goal[i];
			
			
			if( 
				goal_joint_pos.data[0] < 2.73  && goal_joint_pos.data[0] > 0.02 &&
				goal_joint_pos.data[1] < 1.07  && goal_joint_pos.data[1] > -1.55 &&
				goal_joint_pos.data[2] < 1.55  && goal_joint_pos.data[2] > -3.51 &&
				goal_joint_pos.data[3] < 2.34  && goal_joint_pos.data[3] > -0.37 &&
				goal_joint_pos.data[4] < 2.07  && goal_joint_pos.data[4] > -2.07 &&
				goal_joint_pos.data[5] < 1.41  && goal_joint_pos.data[5] > -1.41 &&
				goal_joint_pos.data[6] < 2.07  && goal_joint_pos.data[6] > -2.07 
			){
				ROS_INFO("goal position transformation succeeded. ");
				local_goal_joint_pos = goal_joint_pos;
				local_goal_joint_vel = goal_joint_vel;
				goal_updated = false;
			}
			else{
				ROS_ERROR("can not reach this goal.Discard this goal.");
				//ros::shutdown();
				
				std::cout << "goal joint pose: ";
				for(int i = 0; i < 7; i++)
					std::cout << goal_joint_pos.data[i] << " ";
				std::cout << std::endl;
				/*
				std::cout << "goal cart pose: ";
				for(int i = 0; i < 3; i++)
					std::cout << goal_cart_pos.p[i] << " ";
				std::cout << std::endl;
				*/
			}
		}
		else
		{
			ROS_ERROR("cart to joint failed. Discard this goal.");
		}
	}

	
	control_point.positions.clear();
	control_point.velocities.clear();
	for(int i = 0; i < 7; i++)
	{
		control_point.positions.push_back(local_goal_joint_pos.data[i]);
		control_point.velocities.push_back(local_goal_joint_vel.data[i]);
		//control_point.velocities.push_back(0);
	}

	//control_point.time_from_start = loc_time_from_start;
	control_msg.points.clear();
	control_msg.points.push_back(control_point);
	pub_control_msg.publish(control_msg);	
}






