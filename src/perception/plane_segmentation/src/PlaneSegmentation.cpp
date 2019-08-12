#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Char.h>


//#include <message_filters/subscriber.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/exact_time.h>
//#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <perception_msgs/Job.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// PCL specific includes
#include <pcl_ros/point_cloud.h> // enable pcl publishing
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <image_geometry/pinhole_camera_model.h>


// Visualization
//#include <visualization_msgs/MarkerArray.h>
//#include <visualization_msgs/Marker.h>


//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>
#include <limits>
using namespace std;
using namespace cv;


//! Plane segmentation class
//! computes and split the big planes from the rest of the point cloud clusters
class PlaneSegmentation
{

private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    ros::Subscriber sub_job;

    ros::Publisher pub_job;
    
    ros::Publisher pub_clusters_pc_;

    //sensor_msgs::PointCloud2ConstPtr curr_point_cloudPtr;
    

	//bool FLAG_VALID;
    //------------------ Callbacks -------------------
    void plane_segmentation(const perception_msgs::Job::ConstPtr& job);

    //! Callback for service calls
	

    //! Callback for subscribers
    //! Complete processing of new point cloud
    //void GetCloud(const sensor_msgs::PointCloud2ConstPtr &pc2); // for multiple data topics (const sensor_msgs::TypeConstPtr &var, const sensor_msgs::TypeConstPtr &var, ...)
    //void processCloud(perception_msgs::GetCentroid::Request& req,perception_msgs::GetCentroid::Response& res);

public:
    //! Subscribes to and advertises topics
    PlaneSegmentation(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
        //sub(nh, "topic", 5) // constructor initialization form for the subscriber if needed
    {
        sub_job = nh_.subscribe<perception_msgs::Job>("/perception/job_plane_segmentation",10,&PlaneSegmentation::plane_segmentation,this);
		pub_job = nh_.advertise<perception_msgs::Job>("/perception/job_calculate_centroids",10);
        //FLAG_VALID = false;
        //sub_pc2 = nh_.subscribe<sensor_msgs::PointCloud2>("/xtion/depth_registered/points",10,&PlaneSegmentation::GetCloud,this);
        //pub_plane_pc_ = nh_.advertise< sensor_msgs::PointCloud2 >("/segmentation/plane_points", 10);
        pub_clusters_pc_ = nh_.advertise< sensor_msgs::PointCloud2 >("/perception/segmentation/clusters_points", 10);
        //pub_clusters_array_pc_ = nh_.advertise< perception_msgs::PointCloudArray >("/segmentation/clusters_array_points", 10);

        //sub_GetCentroid = nh.subscribe<>("/perception/ClassCentroid",10,&PlaneSegmentation::processCloud,this);
        // Callback function register

        //initialize params
    }

    ~PlaneSegmentation() {}
};

//! Callback for processing the Point Cloud data
void PlaneSegmentation::plane_segmentation(const perception_msgs::Job::ConstPtr& job)
{
	
	ros::Time ti, tf;
    ti=ros::Time::now();
    tf = ti;	
	
	ROS_INFO("plane seg.");
    pcl::PointCloud< pcl::PointXYZ > pc,pc_valid; // internal data
    
	// Convert the data to the internal var (pc) using pcl function: fromROSMsg
	pcl::fromROSMsg(job->pc2,pc);
	
	std::vector< int > index;
	
	pcl::removeNaNFromPointCloud(pc,pc_valid,index);
	(void)index;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pc_valid.makeShared(); // cloud to operate
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZ> ); // cloud to store the filter the data

    std::cout << "PointCloud before filtering has: " << pc_valid.points.size() << " data points." << std::endl; //*
    std::cout << "width: " << pc_valid.width << "height: " << pc_valid.height << std::endl;

    // Down sample the pointcloud using VoxelGrid
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	filter.setLeafSize (0.01f, 0.01f, 0.01f);
	filter.filter(*cloud_filtered);
	
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size()  << " data points." << std::endl;

    // Create the segmentation object for the plane model and set all the parameters using pcl::SACSegmentation<pcl::PointXYZ>
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //seg.setOptimizeCoefficients (true);
    pcl::PointIndices::Ptr inliers( new pcl::PointIndices );
    pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster( new pcl::PointCloud<pcl::PointXYZ>() );
    // set parameters of the SACS segmentation
	
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);
    seg.setMaxIterations (1000);
    //----

    int nr_points = (int) cloud_filtered->points.size();
    int itr_cntr = 0;
    while (cloud_filtered->points.size () > 0.1 * nr_points && itr_cntr<=15){
    itr_cntr++;
    // Segment the planes using pcl::SACSegmentation::segment() function and pcl::ExtractIndices::filter() function
    seg.setInputCloud (cloud_filtered);
	seg.segment(*inliers,*coefficients);
	if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

	pcl::ExtractIndices<pcl::PointXYZ> extr;
	extr.setInputCloud(cloud_filtered);
    extr.setIndices(inliers);
    extr.setNegative (false);
	extr.filter(*cloud_plane);

	extr.setNegative (true);
    extr.filter (*cloud_cluster);
    *cloud_filtered = *cloud_cluster;
	}
	ROS_INFO_STREAM("iteration number: "<<itr_cntr);
	
	sensor_msgs::PointCloud2 curr_clusters_pc_ros;
    toROSMsg(*cloud_cluster,curr_clusters_pc_ros);

    perception_msgs::Job job_next_stage;

    job_next_stage.pc2 = curr_clusters_pc_ros;
    job_next_stage.time_stamp = job->time_stamp;
    job_next_stage.bboxes_NewObjects = job->bboxes_NewObjects;
    
	tf = ros::Time::now();
    ROS_INFO_STREAM("plane segmentation takes: "<<tf-ti<<"s");
    
    ti = tf;
    
    
	float tmp_x_hom,tmp_y_hom;
	float max_x_hom = std::numeric_limits<float>::min();
	float max_y_hom = std::numeric_limits<float>::min();
	float min_x_hom = std::numeric_limits<float>::max();
	float min_y_hom = std::numeric_limits<float>::max();
	
	for(int i = 0; i<pc_valid.size(); i++){
		tmp_x_hom = pc_valid.points[i].x/pc_valid.points[i].z;
		tmp_y_hom = pc_valid.points[i].y/pc_valid.points[i].z;
		
		if(tmp_x_hom>=max_x_hom)
			max_x_hom = tmp_x_hom;
		if(tmp_x_hom<=min_x_hom)
			min_x_hom = tmp_x_hom;
		if(tmp_y_hom>=max_y_hom)
			max_y_hom = tmp_y_hom;
		if(tmp_y_hom<=min_y_hom)
			min_y_hom = tmp_y_hom;
	}

    job_next_stage.g_xmin = min_x_hom;
    job_next_stage.g_ymin = min_y_hom;
    job_next_stage.g_xmax = max_x_hom;
    job_next_stage.g_ymax = max_y_hom;
    
    job_next_stage.width = pc.width;
    job_next_stage.height = pc.height;

    pub_job.publish(job_next_stage);

    //pub_clusters_pc_.publish(curr_clusters.makeShared());
    pub_clusters_pc_.publish(curr_clusters_pc_ros);
    
    tf = ros::Time::now();
    ROS_INFO_STREAM("processing takes: "<<tf-ti<<"s");
    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_segmentation");
    ros::NodeHandle nh;
    PlaneSegmentation node(nh);
    ros::spin();
    return 0;
}


