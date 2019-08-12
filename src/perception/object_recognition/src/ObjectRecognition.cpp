#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/PointCloud2.h>
#include <actionlib/client/simple_action_client.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
// boost
#include <boost/thread.hpp>

#include <darknet_ros_msgs/CheckForObjectsAction.h>

#include <perception_msgs/Job.h>
#include <perception_msgs/ClassCentroids.h>

#include <perception_msgs/SystGetCentroid.h>
#include <perception_msgs/SystYOLO_SetState.h>

#include <tf/transform_listener.h>

#include <limits>
/*********************************************************************
* EIGEN INCLUDES
********************************************************************/
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <vector>
#include <math.h> 

//using namespace Eigen;

#define YOLO_ENABLE 1
#define YOLO_DISABLE 0
#define IMAGE_BUF_SIZE_ros 50

typedef actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction> Client;
typedef std::shared_ptr<Client> ClientPtr;
//typedef actionlib::SimpleActionClient< darknet_ros_msgs::CheckForObjectsAction >::ResultConstPtr AcResultConstPtr;

class ObjectRecogenition{
private:
	/*-----   Interface   -----*/
	ros::Subscriber sub_image;
	ros::Subscriber sub_pc2;
	ros::ServiceServer srv_ext; //receive external call for object centroid

	/*-----   YOLO relevant   -----*/
	ros::ServiceServer srv_yolo_enable;
	ClientPtr ac; //action client used to send image YOLO
	// Subscribers to the bounding box data from YOLO
	//ros::Subscriber sub_bbox;
	//ros::Subscriber sub_obj_num;

	/*-----   Job Publisher   -----*/
	ros::Publisher pub_job;

	/*-----   Subscribe results of job   -----*/
	ros::Subscriber sub_Centroids;

	/*-----   Synchronizer   -----*/
	std::vector<sensor_msgs::Image> buf_image;
	ros::Time image_time_stamp;
	ros::Subscriber sub_JointStates;

	/*-----   Local member variables   -----*/
	std::vector<std::string> class_names;
	std::vector<geometry_msgs::PointStamped> class_centroids;
	int state;
	sensor_msgs::PointCloud2ConstPtr loc_pc2_ptr;

	bool FLAG_CMP_DONE,OVERSPEED;
	//NEW_FRAME;


	/*-----   Callbacks   -----*/
	void GetCloud(const sensor_msgs::PointCloud2ConstPtr& pc);
	void PushImageToBuffer(const sensor_msgs::ImageConstPtr& img);

	//void ProcessBoundingBox(const darknet_ros_msgs::BoundingBoxesConstPtr& msg_bbox);
	void ProcessBoundingBox(const darknet_ros_msgs::BoundingBoxes& msg_bbox);
	
	void ProcessClassCentroids(const perception_msgs::ClassCentroids msg_ClassCentroids);
	//void GetBoundingBoxNumber(const std_msgs::Int8 num);
	bool SendCentroidToSystem(perception_msgs::SystGetCentroid::Request& req,perception_msgs::SystGetCentroid::Response& res);
	bool YOLO_SetState(perception_msgs::SystYOLO_SetState::Request& req,perception_msgs::SystYOLO_SetState::Response& res);
	
	//void Get3DCentroid(const geometry_msgs::PointConstPtr& centroid);
	//perception_msgs::ClassCent ObjectRecogenition::GetClass3DCentroid(const perception_msgs::ClassCent::ConstPtr& srv);

	tf::TransformListener listener_;

	geometry_msgs::PointStamped transformPoint(const tf::TransformListener& listener, const geometry_msgs::PointStamped& mov_point);
	
	void VelocityMonitor(const sensor_msgs::JointState::ConstPtr& msg_joint_states);
	////////////////////debug
	ros::Publisher pub_debug_centroid;
	//geometry_msgs::PointStamped debug_centroid;
	/////////////////////////
public:	
	
	ObjectRecogenition(ros::NodeHandle nh):listener_(ros::Duration(60),true) 
	{
		state = 0;
		FLAG_CMP_DONE = true;
		OVERSPEED = false;
		//NEW_FRAME = false;
		/*-----     Action Client init.     -----*/
		std::string checkForObjectsActionName;
		nh.param("/darknet_ros/camera_action", checkForObjectsActionName, std::string("/darknet_ros/check_for_objects"));
  		ac.reset(new Client(nh, checkForObjectsActionName,true));

		/*-----     init. for all other stuff     -----*/
		sub_image = nh.subscribe<sensor_msgs::Image>("/xtion/rgb/image_rect_color",10,&ObjectRecogenition::PushImageToBuffer,this);

		//sub_obj_num = nh.subscribe<std_msgs::Int8>("/darknet_ros/found_object",100,&ObjectRecogenition::GetBoundingBoxNumber,this);
		sub_pc2 = nh.subscribe<sensor_msgs::PointCloud2>("/xtion/depth_registered/points", 10, &ObjectRecogenition::GetCloud,this);

		//sub_bbox = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes",100,&ObjectRecogenition::ProcessBoundingBox,this);

		pub_job = nh.advertise<perception_msgs::Job>("/perception/job_plane_segmentation",10);
		sub_Centroids = nh.subscribe<perception_msgs::ClassCentroids>("/perception/result",10,&ObjectRecogenition::ProcessClassCentroids,this);

		srv_ext = nh.advertiseService("/perception/GetCentroid",&ObjectRecogenition::SendCentroidToSystem,this);
		srv_yolo_enable = nh.advertiseService("/perception/yolo_enable",&ObjectRecogenition::YOLO_SetState,this);
		
		sub_JointStates = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &ObjectRecogenition::VelocityMonitor, this);
		////////////////////debug
		pub_debug_centroid = nh.advertise<geometry_msgs::PointStamped>("/debug/centroid",10);
		//pub_debug_centroid.publish(debug_centroid);
		/////////////////////////
	}

	~ObjectRecogenition(){}

};

	bool ObjectRecogenition::YOLO_SetState(perception_msgs::SystYOLO_SetState::Request& req,perception_msgs::SystYOLO_SetState::Response& res){
		state = req.state;
		return true;
	}

	void ObjectRecogenition::GetCloud(const sensor_msgs::PointCloud2ConstPtr& pc){
		//NEW_FRAME = true;
		loc_pc2_ptr = pc;
		if(buf_image.empty() || state == YOLO_DISABLE || OVERSPEED){
			return;
			}
		ros::Time pc_time_stamp = (pc->header).stamp;
		double pc_time = pc_time_stamp.toSec();
		FLAG_CMP_DONE = false;
		double time_diff;
		double min_time_diff = 1000.0;
		int valid_idx = 0;
		
		for(int i = 0;i<buf_image.size();i++){
			
			time_diff = std::abs(buf_image[i].header.stamp.toSec() - pc_time);
			if(time_diff<=min_time_diff){
				min_time_diff = time_diff;
				valid_idx = i;
			}
		}
		
		FLAG_CMP_DONE = true;
		
		if(min_time_diff<=30){
		std::cout<<"min_diff = "<<min_time_diff<<std::endl;
		darknet_ros_msgs::CheckForObjectsGoal goal;
		goal.image = buf_image[valid_idx];
		image_time_stamp = (buf_image[valid_idx]).header.stamp;
		ros::Time beginYolo = ros::Time::now();
		ac->sendGoal(goal);
		
		ac->waitForResult(ros::Duration(2.0));
		if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			 //darknet_ros_msgs::CheckForObjectsActionResultConstPtr result;
			 ProcessBoundingBox((ac->getResult())->bounding_boxes);
			 //ProcessBoundingBox(result);
		}else{
			ROS_ERROR("yolo failed");
		}
		}
	}

	void ObjectRecogenition::PushImageToBuffer(const sensor_msgs::ImageConstPtr& img){
		if(state==YOLO_ENABLE && !OVERSPEED){
			if(!FLAG_CMP_DONE)
				return;
			while(buf_image.size()>=IMAGE_BUF_SIZE_ros)
				buf_image.erase(buf_image.begin());
			buf_image.push_back(*img);
		}else{
			//ROS_INFO("YOLO deactivated");
		}
		/*
		if(!ac->waitForResult(ros::Duration(30.0))) {
			std::cout << "[ObjectDetectionTest] sendImageToYolo(): checkForObjects action server took to long to send back result." << std::endl;
  		}else{
  			ros::Time endYolo = ros::Time::now();
  			std::cout << "[ObjectDetectionTest] Object detection for one image took " << endYolo-beginYolo << " seconds." << std::endl;
  		}
  		*/
	}

	//void ObjectRecogenition::ProcessBoundingBox(const darknet_ros_msgs::BoundingBoxesConstPtr& msg_bbox){
	void ObjectRecogenition::ProcessBoundingBox(const darknet_ros_msgs::BoundingBoxes& msg_bbox){
		//if(NEW_FRAME){
			//NEW_FRAME = false;
			int BoundingBoxNumber = msg_bbox.bounding_boxes.size();
			//std::cout<<"BoundingBoxNumber = "<<BoundingBoxNumber<<std::endl;
			std::string curr_class_name;
			darknet_ros_msgs::BoundingBox curr_bbox;
			darknet_ros_msgs::BoundingBoxes bboxes_NewObjects;

			bboxes_NewObjects.header = msg_bbox.header;
			bboxes_NewObjects.image_header = msg_bbox.image_header;
			
			//ROS_INFO("New frame received");

			for(int i = 0; i < BoundingBoxNumber; i++){
				curr_bbox = msg_bbox.bounding_boxes[i];
				curr_class_name = curr_bbox.Class;

				//determin whether the new object already exists in local variable
				if(curr_bbox.probability>0.30&&(std::find(class_names.begin(),class_names.end(),curr_class_name)==class_names.end())){
					//does not exist ---> store the bounding box
					class_names.push_back(curr_class_name);
					std::cout<<"New object found: '"<< curr_class_name <<"' now generate a new job."<<std::endl;
					bboxes_NewObjects.bounding_boxes.push_back(curr_bbox);
				}
			}

			if(bboxes_NewObjects.bounding_boxes.size()>0){ //new object(s) recognized ---> publish new job
				perception_msgs::Job job;
				job.time_stamp = loc_pc2_ptr->header.stamp;
				job.pc2 = *loc_pc2_ptr;
				job.bboxes_NewObjects = bboxes_NewObjects;
				pub_job.publish(job);
			}	
		//}
	}


	void ObjectRecogenition::ProcessClassCentroids(const perception_msgs::ClassCentroids msg_ClassCentroids){
		int new_size = msg_ClassCentroids.class_names.size();
		geometry_msgs::PointStamped tmp_centroid;
		
		for(int i = 0; i<new_size; i++){
			//class_names.push_back(msg_ClassCentroids.class_names[i]);
			//tmp_centroid = ObjectRecogenition::transformPoint(listener_,msg_ClassCentroids.class_centroids[i]);
			//tmp_centroid = msg_ClassCentroids.class_centroids[i];
			
			std::string source_link = msg_ClassCentroids.class_centroids[i].header.frame_id;
			try{
				ROS_INFO_STREAM("tf time: "<<msg_ClassCentroids.class_centroids[i].header.stamp.sec);
				listener_.waitForTransform(source_link,"torso_lift_link",msg_ClassCentroids.class_centroids[i].header.stamp,ros::Duration(1));
    			listener_.transformPoint("torso_lift_link", msg_ClassCentroids.class_centroids[i], tmp_centroid);
  			}catch(tf::TransformException& ex){
    			ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"torso_lift_link\": %s", source_link.c_str(), ex.what());
    		}
    		
			class_centroids.push_back(tmp_centroid);
			std::cout<<msg_ClassCentroids.class_names[i]<<" is found at ["<<tmp_centroid.point.x<<", "<<tmp_centroid.point.y<<", "<<tmp_centroid.point.z<<"]"<<std::endl;
		}
		
	}

  	bool ObjectRecogenition::SendCentroidToSystem(perception_msgs::SystGetCentroid::Request& req,perception_msgs::SystGetCentroid::Response& res){
  		std::vector<std::string>::iterator it;
  		ROS_INFO_STREAM(class_names.size());
  		ptrdiff_t  pos = std::find(class_names.begin(),class_names.end(),req.Name)-class_names.begin();
  		std::cout<<(int)pos<<std::endl;
  		if((int)pos >= class_names.size()) {
    		ROS_INFO("Required object does not exists!");
    		return false;
		}
		/////////debug
		pub_debug_centroid.publish(class_centroids.at((int)pos));
		//////////////
		res.centroid = class_centroids.at((int)pos);
  		return true;
  	}
  	
  	void ObjectRecogenition::VelocityMonitor(const sensor_msgs::JointState::ConstPtr& msg_joint_states){
  		if(msg_joint_states->velocity[9]>0.01 || msg_joint_states->velocity[10]>0.01 || msg_joint_states->velocity[9]<-0.01 || msg_joint_states->velocity[10]<-0.01 ){
  			OVERSPEED = true;	
  			//ROS_INFO("too fast!");	
  			}
  		else{
  			OVERSPEED = false;
  			//ROS_INFO("speed ok");	
  		}
  	}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_recognition");
    ros::NodeHandle nh;
    ObjectRecogenition node(nh);
    ros::AsyncSpinner spinner(5); // Use 4 threads
	spinner.start();
	ros::waitForShutdown();
    //ros::spin();
    return 0;
}
