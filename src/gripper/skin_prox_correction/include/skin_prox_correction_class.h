#include <math.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tum_ics_skin_msgs/SkinCellDataArray.h>

class SkinProxCorrection
{

private:

  ros::NodeHandle nh_;

  ros::Publisher pub_goal_position_correction;
  ros::Publisher pub_acc_correction;
  ros::Subscriber patch_1;
  ros::Subscriber patch_2;
  //ros::Subscriber patch_3;
  ros::Subscriber patch_4;
  //ros::Subscriber patch_5;
  ros::Subscriber patch_6;
  ros::Subscriber patch_7;
  ros::Subscriber patch_8;
  ros::Subscriber patch_9;
  ros::Subscriber patch_10;
  ros::ServiceServer pos_correction_start;
  ros::ServiceServer pos_correction_stop;

  int cellId_patch[10];
  double prox_patch[10];
  double weight_x, weight_y, weight_z;

  double threshold_1, threshold_2;  //set the threshold of prox
  double max_delta_x, delta_x;
  int direction;

  geometry_msgs::Vector3Stamped goal_position_correction;
  geometry_msgs::Vector3Stamped acc_correction;

  //void Get_position_from_percertion(const geometry_msgs::PoseStamped::ConstPtr &position_from_perception); //will be used for the subscribe of position of perception
  void Get_prox_pathch_1(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch); //get the prox of left  finger from skin
  void Get_prox_pathch_2(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch); //get the prox of right finger from skin
  void Get_prox_pathch_3(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch);
  void Get_prox_pathch_4(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch);
  void Get_prox_pathch_5(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch);
  void Get_prox_pathch_6(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch);
  void Get_prox_pathch_7(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch);
  void Get_prox_pathch_8(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch);
  void Get_prox_pathch_9(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch);
  void Get_prox_pathch_10(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch);
  bool Correction_start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool Correction_stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  void Movement_correction();


public:

  SkinProxCorrection(ros::NodeHandle nh);
  void Position_correction();
  bool flag; //flag == True: stop position correction;flag == False: start position correction;

};
