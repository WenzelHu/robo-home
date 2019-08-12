#include <skin_prox_correction_class.h>

SkinProxCorrection::SkinProxCorrection(ros::NodeHandle nh) : nh_(nh)
{

  for(int i=0;i<10;i++)
  {
    cellId_patch[i] = 0;
    prox_patch[i] = 0;
  }

  weight_x = 30;
  weight_y = 30;
  weight_z = 30;

  threshold_1 = 0.1;
  threshold_2 = 0.1;
  max_delta_x = 0.1;
  delta_x = 0.0;

  direction = 1;
  flag = false;

  goal_position_correction.header.frame_id = "gripper_link";
  goal_position_correction.vector.x = 0;
  goal_position_correction.vector.y = 0;
  goal_position_correction.vector.z = 0;
  acc_correction.header.frame_id = "gripper_link";
  acc_correction.vector.x = 0;
  acc_correction.vector.y = 0;
  acc_correction.vector.z = 0;

  pub_goal_position_correction = nh_.advertise<geometry_msgs::Vector3Stamped>("/control/goal_offset", 10);
  pub_acc_correction  = nh_.advertise<geometry_msgs::Vector3Stamped>("/control/acc_offset", 10);
  patch_1 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch1", 10, &SkinProxCorrection::Get_prox_pathch_1, this);
  patch_2 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch2", 10, &SkinProxCorrection::Get_prox_pathch_2, this);
  //patch_3 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch3", 10, &SkinProxCorrection::Get_prox_pathch_3, this);
  patch_4 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch4", 10, &SkinProxCorrection::Get_prox_pathch_4, this);
  //patch_5 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch5", 10, &SkinProxCorrection::Get_prox_pathch_5, this);
  patch_6 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch6", 10, &SkinProxCorrection::Get_prox_pathch_6, this);
  patch_7 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch7", 10, &SkinProxCorrection::Get_prox_pathch_7, this);
  patch_8 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch8", 10, &SkinProxCorrection::Get_prox_pathch_8, this);
  patch_9 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch9", 10, &SkinProxCorrection::Get_prox_pathch_9, this);
  patch_10 = nh_.subscribe<tum_ics_skin_msgs::SkinCellDataArray>("/tiago/patch10", 10, &SkinProxCorrection::Get_prox_pathch_10, this);
  pos_correction_start = nh_.advertiseService("/pos_correction_start", &SkinProxCorrection::Correction_start, this);
  pos_correction_stop = nh_.advertiseService("/pos_correction_stop", &SkinProxCorrection::Correction_stop, this);

  ROS_INFO("Initialization done!");
}

/*void SkinProxCorrection::Get_position_from_percertion(const geometry_msgs::PoseStamped::ConstPtr &position_from_perception) //how to judge the postion which was changed ???
{
  goal_position_from_perception.x = position_from_perception->pose.position.x;
  goal_position_from_perception.y = position_from_perception->pose.position.y;
  goal_position_from_perception.z = position_from_perception->pose.position.z;
}*/

void SkinProxCorrection::Get_prox_pathch_1(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of left finger
{
  prox_patch[0] = data_patch->data.data()->prox[0];
  Movement_correction();
}

void SkinProxCorrection::Get_prox_pathch_2(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  prox_patch[1] = data_patch->data.data()->prox[0];
  Movement_correction();
}

/*void SkinProxCorrection::Get_prox_pathch_3(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  if(data_patch->data.data()->prox[0] > prox_patch[2])
  {
    prox_patch[2] = data_patch->data.data()->prox[0];
    cellId_patch[2] = data_patch->data.data()->cellId;
    Movement_correction();
  }
  else if(cellId_patch[2] == data_patch->data.data()->cellId)
  {
    prox_patch[2] = data_patch->data.data()->prox[0];
    Movement_correction();
  }
}*/

void SkinProxCorrection::Get_prox_pathch_4(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  if(data_patch->data.data()->prox[0] > prox_patch[3])
  {
    prox_patch[3] = data_patch->data.data()->prox[0];
    cellId_patch[3] = data_patch->data.data()->cellId;
    Movement_correction();
  }
  else if(cellId_patch[3] == data_patch->data.data()->cellId)
  {
    prox_patch[3] = data_patch->data.data()->prox[0];
    Movement_correction();
  }
}

/*void SkinProxCorrection::Get_prox_pathch_5(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  if(data_patch->data.data()->prox[0] > prox_patch[4])
  {
    prox_patch[4] = data_patch->data.data()->prox[0];
    cellId_patch[4] = data_patch->data.data()->cellId;
    Movement_correction();
  }
  else if(cellId_patch[4] == data_patch->data.data()->cellId)
  {
    prox_patch[4] = data_patch->data.data()->prox[0];
    Movement_correction();
  }
}*/

void SkinProxCorrection::Get_prox_pathch_6(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  if(data_patch->data.data()->prox[0] > prox_patch[5])
  {
    prox_patch[5] = data_patch->data.data()->prox[0];
    cellId_patch[5] = data_patch->data.data()->cellId;
    Movement_correction();
  }
  else if(cellId_patch[5] == data_patch->data.data()->cellId)
  {
    prox_patch[5] = data_patch->data.data()->prox[0];
    Movement_correction();
  }
}

void SkinProxCorrection::Get_prox_pathch_7(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  if(data_patch->data.data()->prox[0] > prox_patch[6])
  {
    prox_patch[6] = data_patch->data.data()->prox[0];
    cellId_patch[6] = data_patch->data.data()->cellId;
    Movement_correction();
  }
  else if(cellId_patch[6] == data_patch->data.data()->cellId)
  {
    prox_patch[6] = data_patch->data.data()->prox[0];
    Movement_correction();
  }
}

void SkinProxCorrection::Get_prox_pathch_8(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  if(data_patch->data.data()->prox[0] > prox_patch[7])
  {
    prox_patch[7] = data_patch->data.data()->prox[0];
    cellId_patch[7] = data_patch->data.data()->cellId;
    Movement_correction();
  }
  else if(cellId_patch[7] == data_patch->data.data()->cellId)
  {
    prox_patch[7] = data_patch->data.data()->prox[0];
    Movement_correction();
  }
}

void SkinProxCorrection::Get_prox_pathch_9(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  if(data_patch->data.data()->prox[0] > prox_patch[8])
  {
    prox_patch[8] = data_patch->data.data()->prox[0];
    cellId_patch[8] = data_patch->data.data()->cellId;
    Movement_correction();
  }
  else if(cellId_patch[8] == data_patch->data.data()->cellId)
  {
    prox_patch[8] = data_patch->data.data()->prox[0];
    Movement_correction();
  }
}

void SkinProxCorrection::Get_prox_pathch_10(const tum_ics_skin_msgs::SkinCellDataArray::ConstPtr &data_patch) //Get the prox of right finger
{
  if(data_patch->data.data()->prox[0] > prox_patch[9])
  {
    prox_patch[9] = data_patch->data.data()->prox[0];
    cellId_patch[9] = data_patch->data.data()->cellId;
    Movement_correction();
  }
  else if(cellId_patch[9] == data_patch->data.data()->cellId)
  {
    prox_patch[9] = data_patch->data.data()->prox[0];
    Movement_correction();
  }
}

bool SkinProxCorrection::Correction_start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  flag = true;
  return true;
}

bool SkinProxCorrection::Correction_stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  flag = false;
  return true;
}

void SkinProxCorrection::Movement_correction()
{
  double prox_xL, prox_xR, prox_x, prox_yL, prox_yR, prox_y, prox_zL, prox_zR, prox_z, da_x, da_y, da_z;
  prox_xL = (prox_patch[3]>prox_patch[9])?prox_patch[3]:prox_patch[9];
  prox_xR = (prox_patch[5]>prox_patch[7])?prox_patch[5]:prox_patch[7];
  prox_yL = prox_patch[8];
  prox_yR = prox_patch[6];
  prox_zL = (prox_patch[0]>prox_patch[1])?prox_patch[0]:prox_patch[1];
  prox_zR = 0;
  prox_x = prox_xL * prox_xL - prox_xR * prox_xR;
  prox_y = prox_yL * prox_yL - prox_yR * prox_yR;
  prox_z = prox_zL * prox_zL - prox_zR * prox_zR;
  if(prox_x < 0.4 && prox_y < 0.4 && prox_z < 0.4)
  {
  	da_x = weight_x * (prox_xL * prox_xL - prox_xR * prox_xR);
    da_y = weight_y * (prox_yL * prox_yL - prox_yR * prox_yR);
    da_z = weight_z * (prox_zL * prox_zL - prox_zR * prox_zR);
  }
  else
  {
    da_x = (weight_x-15) * (prox_xL * prox_xL - prox_xR * prox_xR);
    da_y = (weight_y-15) * (prox_yL * prox_yL - prox_yR * prox_yR);
    da_z = (weight_z-15) * (prox_zL * prox_zL - prox_zR * prox_zR);
  }

  acc_correction.vector.x = (fabs(da_x)>weight_x*0.00075)?da_x:0; //sensor will have disturbance
  acc_correction.vector.y = (fabs(da_y)>weight_y*0.00075)?da_y:0;
  acc_correction.vector.z = (fabs(da_z)>weight_z*0.00075)?da_z:0;
  //acc_correction.header.stamp = ros::Time::now();
  pub_acc_correction.publish(acc_correction);
  //ROS_INFO_STREAM("dax="<<acc_correction.vector.x<<"\nday="<<acc_correction.vector.y<<"\ndaz="<<acc_correction.vector.z<<"\n");
}

void SkinProxCorrection::Position_correction()
{
  if( prox_patch[0] < threshold_1 && prox_patch[1] < threshold_2)
  {
    ROS_INFO_STREAM("OK, I can grip !!!!!!!!!!");
  }
  else
  {
    if(prox_patch[0] > threshold_1 && prox_patch[1] < threshold_2)
    {
      direction = -1;
      ROS_INFO_STREAM("Move left !!!!!!!!!!");
    }
    if(prox_patch[0] < threshold_1 && prox_patch[1] > threshold_2)
    {
      direction = 1;
      ROS_INFO_STREAM("Move right !!!!!!!!!!");
    }

    ROS_INFO_STREAM("X= "<<delta_x);

    delta_x = delta_x + 0.015;//move distance
    
    goal_position_correction.vector.x = delta_x * direction;
    //goal_position_correction.header.stamp = ros::Time::now();
    //goal_position_correction.vector.y = goal_position_from_perception.y;
    //goal_position_correction.pose.position.z = goal_position_from_perception.z;
    pub_goal_position_correction.publish(goal_position_correction);

    if(delta_x > max_delta_x - 0.001)
    {
      delta_x = 0;
      direction = -direction;
    }
  }
  ROS_INFO_STREAM("delta_x = "<< delta_x);
}
