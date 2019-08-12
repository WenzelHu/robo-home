#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include <filters/mean.h>

#include <math.h> 

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <stiffness_control_msgs/WristDataUpdate.h>
//#include <Eigen/Dense>

class WristStiffnessController{
private:
	ros::ServiceClient client_Update;
	
	//tf::StampedTransform transform;
	tf::TransformListener listener;

	/*-----   Callbacks   -----*/

public:

	ros::Publisher pub_CurrentGoalPose;

	/*-----   Locals   -----*/
	//trajectory_msgs::JointTrajectoryPoint current_EndEff_state;
	geometry_msgs::Vector3Stamped vel_EndEff;
	geometry_msgs::PointStamped pos_EndEff;
	geometry_msgs::PoseStamped desired_goal_pose;
	geometry_msgs::Vector3Stamped ft_force;
	geometry_msgs::Vector3Stamped acc_offset;
	
	bool Update();
	
	WristStiffnessController(ros::NodeHandle nh) 
	{	
		client_Update = nh.serviceClient<stiffness_control_msgs::WristDataUpdate>("/control/wrist_stiffness/update",10);
		pub_CurrentGoalPose = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/admittance_controller/goal", 10);
		Update();
	}

	~WristStiffnessController() {}
};

bool WristStiffnessController::Update(){
	stiffness_control_msgs::WristDataUpdate msg_update;
	
	if(client_Update.call(msg_update)){
	
		vel_EndEff = msg_update.response.vel_EndEff;
		pos_EndEff = msg_update.response.pos_EndEff;
		desired_goal_pose = msg_update.response.desired_goal_pose;
		desired_goal_pose.pose.position.x += msg_update.response.goal_offset.vector.x;
		desired_goal_pose.pose.position.y += msg_update.response.goal_offset.vector.y;
		desired_goal_pose.pose.position.z += msg_update.response.goal_offset.vector.z;
		acc_offset = msg_update.response.acc_offset;
		
		ft_force = msg_update.response.ft_force;
		ROS_INFO_STREAM("received position: "<<pos_EndEff.point);
		return true;
	}else{
		ROS_INFO("update failed");
		return false;
	}
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "wrist_stiffness_controller");
    ros::NodeHandle nh;
    WristStiffnessController controller(nh);

    ros::Rate r(100);

    ros::Time ti, tf;
    ti=ros::Time::now();
    tf = ti;

    double x,y,z,x_d,y_d,z_d,x_dd,y_dd,z_dd;
	double M_x,M_y,M_z,K_x,K_y,K_z,D_x,D_y,D_z;
	double dt = 0;
	bool result;
	geometry_msgs::Vector3Stamped vel_EndEff_old;
	geometry_msgs::PointStamped pos_EndEff_old;

	//param init
	nh.param<double>("/wrist/M_x",M_x,1); nh.param<double>("/wrist/M_y",M_y,1); nh.param<double>("/wrist/M_z",M_z,1);
	nh.param<double>("/wrist/K_x",K_x,10); nh.param<double>("/wrist/K_y",K_y,10); nh.param<double>("/wrist/K_z",K_z,10);

	//in order to make the system critically damped
	D_x = 2*sqrt(M_x*K_x);
	D_y = 2*sqrt(M_y*K_y);
	D_z = 2*sqrt(M_z*K_z);

	trajectory_msgs::JointTrajectoryPoint goal;

	while(ros::ok())
	{
		vel_EndEff_old = controller.vel_EndEff;
		pos_EndEff_old = controller.pos_EndEff;
		
		result = controller.Update();
		
		if(result){
		
			tf = ros::Time::now();   
        	dt = tf.toSec()-ti.toSec();

			ROS_INFO_STREAM("dt = "<<dt);
			
			if(vel_EndEff_old.vector.x != controller.vel_EndEff.vector.x || pos_EndEff_old.point.x != controller.pos_EndEff.point.x){
				//use sensor reading 
        		x = controller.pos_EndEff.point.x; 
        		y = controller.pos_EndEff.point.y; 
        		z = controller.pos_EndEff.point.z;

        		ROS_INFO_STREAM("current position: ["<<x<<", "<<y<<", "<<z<<"]");

        		x_d = controller.vel_EndEff.vector.x; 
        		y_d = controller.vel_EndEff.vector.y; 
        		z_d = controller.vel_EndEff.vector.z;
        
        		ROS_INFO_STREAM("current velocity: ["<<x_d << ", "<<y_d <<", "<<z_d<<"]");
        	}else{
        		//use internal vel. and pos.
        		//ROS_INFO_STREAM("sensor readings haven't updated, use internal data instead."); 
        		x = controller.pos_EndEff.point.x; 
        		y = controller.pos_EndEff.point.y; 
        		z = controller.pos_EndEff.point.z;
        		x_d = controller.vel_EndEff.vector.x; 
        		y_d = controller.vel_EndEff.vector.y; 
        		z_d = controller.vel_EndEff.vector.z;
        		//
        		ROS_INFO_STREAM("current position: ["<<x<<", "<<y<<", "<<z<<"]");
        		ROS_INFO_STREAM("current velocity: ["<<x_d << ", "<<y_d <<", "<<z_d<<"]");
        		
        	}

    		x_dd = 1/M_x*(K_x*(controller.desired_goal_pose.pose.position.x-x)-D_x*x_d+controller.ft_force.vector.x + 0.0) + controller.acc_offset.vector.x;
        	x_d = x_d + x_dd * dt;
    		x = x + x_d*dt;

    		y_dd = 1/M_y*(K_y*(controller.desired_goal_pose.pose.position.y-y)-D_y*y_d+controller.ft_force.vector.y + 0.0) + controller.acc_offset.vector.y;
    		y_d = y_d + y_dd * dt;
    		y = y + y_d*dt;

    		z_dd = 1/M_z*(K_z*(controller.desired_goal_pose.pose.position.z-z)-D_z*z_d+controller.ft_force.vector.z + 13.0) + controller.acc_offset.vector.z; //+13
    		z_d = z_d + z_dd * dt;
    		z = z + z_d*dt;


			ROS_INFO_STREAM("current force: ["<<controller.ft_force.vector.x << ", "<<controller.ft_force.vector.y <<", "<<controller.ft_force.vector.z+13.0<<"]");
    		ROS_INFO_STREAM("current goal position: ["<<x<<", "<<y<<", "<<z<<"]");
    		ROS_INFO_STREAM("current goal vel: ["<<x_d<<", "<<y_d<<", "<<z_d<<"]");
    		ROS_INFO_STREAM("current goal acc: ["<<x_dd<<", "<<y_dd<<", "<<z_dd<<"]");
    		ROS_INFO_STREAM("current acceleartion offset: ["<<controller.acc_offset.vector.x<<", "<<controller.acc_offset.vector.x<<", "<<controller.acc_offset.vector.x<<"]");
    	
    		goal.positions = {x, y, z};
    		goal.velocities = {-x_d, -y_d, -z_d};//?????
    		goal.accelerations = {x_dd, y_dd, z_dd};
    		goal.time_from_start = tf-ti;
		
    		controller.pub_CurrentGoalPose.publish(goal);
    		//ti=tf;
    		//dt = 0;
		}else{
			tf = ros::Time::now();
		}
		
    	ti=tf;

    	ros::spinOnce();
    	r.sleep();
    }
    return 0;
}
