
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>

#include <cartesian_controller_class.h>

int main(int argc, char** argv)
{
  using namespace std;

  ros::init(argc, argv, "cartesian_control");

  // get chain of the robot arm
  KDL::Tree my_tree;
  ros::NodeHandle node;
  string robot_desc_string;
  node.param("robot_description", robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
  	ROS_ERROR("Failed to construct kdl tree");
  	return false;
  }

  string chain_root = "torso_lift_link";
  string chain_tip =  "arm_tool_link";
  KDL::Chain my_chain;
  my_tree.getChain(chain_root, chain_tip, my_chain);

  //build controller class
  CartesianController CartController(node, my_chain);
  
  ros::AsyncSpinner spinner(3); // Use 3 threads
  spinner.start();

  while(ros::ok())
  {
	CartController.publish_curr_cart_pos_vel();
    //CartController.move_to_goal();
    
    ros::Duration(0.01).sleep();
	//ros::spinOnce();
  }
  ros::waitForShutdown();
}

