#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>  
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/point_cloud_conversion.h>
//#include <geometry_msgs/Point.h>
#include <std_msgs/Char.h>

//#include <message_filters/subscriber.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/exact_time.h>
//#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// PCL specific includes
#include <pcl_ros/point_cloud.h> // enable pcl publishing
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>

//#include <image_geometry/pinhole_camera_model.h>


#include <perception_msgs/Job.h>
#include <perception_msgs/ClassCentroids.h>


using namespace std;
using namespace cv;


class From2Dto3D
{

    private:
  
      //! The node handle
      ros::NodeHandle nh_;
      //! Node handle in the private namespace
      ros::NodeHandle priv_nh_;

      ros::Subscriber sub_job;
 
      ros::Publisher pub_job_result;
      
      /////////////
      //ros::Publisher pub_debug;
      /////////////
      
      //------------------ Callbacks -------------------
      void CalculateCentroids(const perception_msgs::Job::ConstPtr& job);

    public:
      //! Subscribes to and advertises topics
      From2Dto3D(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
      {
        sub_job = nh_.subscribe<perception_msgs::Job>("/perception/job_calculate_centroids", 10, &From2Dto3D::CalculateCentroids,this);
        pub_job_result = nh_.advertise<perception_msgs::ClassCentroids>("/perception/result",10);
        //pub_debug = nh_.advertise< sensor_msgs::PointCloud2 >("/perception/from2dto3d/points_in_bbox", 10);

      }

      ~From2Dto3D() {}
};


void From2Dto3D::CalculateCentroids(const perception_msgs::Job::ConstPtr& job){
  
  pcl::PointCloud< pcl::PointXYZ > pc;
  pcl::fromROSMsg(job->pc2,pc);

  int bboxes_num = job->bboxes_NewObjects.bounding_boxes.size();

  float g_xmin = job->g_xmin;
  float g_ymin = job->g_ymin;
  float g_xmax = job->g_xmax;
  float g_ymax = job->g_ymax;
  int width = job->width;
  int height = job->height;
  
  cout<<"width = "<<width<<" height = "<<height<<endl;
  cout<<"point min: ["<<g_xmin<<", "<<g_ymin<<"] point max: ["<<g_xmax<<", "<<g_ymax<<"]"<<endl;

  float scaling_x = (g_xmax-g_xmin)/width;
  float scaling_y = (g_ymax-g_ymin)/height;
  cout<<"scaling x = "<<scaling_x<<" g_xmin = "<<g_xmin<<endl;
  int pc_size = pc.points.size();

  pcl::PointXYZ curr_point;
  float x_hom,y_hom;
  float tmp_xmin,tmp_ymin,tmp_xmax,tmp_ymax;
  darknet_ros_msgs::BoundingBox curr_bounding_box;

  std::vector<std::vector<int>> class_point_container(bboxes_num);
  
   for(int j = 0; j < bboxes_num; j++){
  	curr_bounding_box = job->bboxes_NewObjects.bounding_boxes[j];
  	tmp_xmin = (curr_bounding_box.xmin+1)*scaling_x+g_xmin;
    tmp_ymin = (curr_bounding_box.ymin+1)*scaling_y+g_ymin;
    tmp_xmax = (curr_bounding_box.xmax+1)*scaling_x+g_xmin;
    tmp_ymax = (curr_bounding_box.ymax+1)*scaling_y+g_ymin; 
  	cout<<"current job: "<<curr_bounding_box.Class<<endl;
	cout<<"bbox idx: "<< j+1 <<"/"<< bboxes_num <<" currBBOX_hom: "<<tmp_xmin<<", "<<tmp_ymin<<"; "<<tmp_xmax<<", "<<tmp_ymax<<endl;
  }
  cout<<endl;
  

  for(int i = 0; i < pc_size; i++){
    curr_point = pc.points[i];
    x_hom = curr_point.x/curr_point.z;
    y_hom = curr_point.y/curr_point.z;
	//cout<<"currpoint_hom: "<<x_hom<<", "<<y_hom<<endl;
	
    for(int j = 0; j < bboxes_num; j++){
      curr_bounding_box = job->bboxes_NewObjects.bounding_boxes[j]; 
      //cout<<"bbox wtf xmin"<< curr_bounding_box.xmin<<endl;
      tmp_xmin = (curr_bounding_box.xmin+1)*scaling_x+g_xmin;
      tmp_ymin = (curr_bounding_box.ymin+1)*scaling_y+g_ymin;
      tmp_xmax = (curr_bounding_box.xmax+1)*scaling_x+g_xmin;
      tmp_ymax = (curr_bounding_box.ymax+1)*scaling_y+g_ymin;
      if(x_hom>=tmp_xmin && x_hom<=tmp_xmax && y_hom>=tmp_ymin && y_hom<=tmp_ymax){
        //store the index to a array which stands for this class 
        class_point_container[j].push_back(i);
      }

    }

  }
  cout<<"container 0 size = "<<class_point_container[0].size()<<endl;
  
  std::vector<geometry_msgs::PointStamped> class_centroids;
  Eigen::Vector4f tmp_cluster_centroid;
  std::vector<std::string> class_names;
  geometry_msgs::PointStamped tmp_centroid;
  tmp_centroid.header = (job->pc2).header;
  tmp_centroid.header.stamp = job->time_stamp;
  
  ////////////////////
  /*
  pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_debug;
  std::shared_ptr<std::vector<int>> idx_ptr = std::make_shared<std::vector<int>> (class_point_container[0]);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (pc.makeShared());
  extract.setIndices (idx_ptr);
  extract.filter (*cloud_debug);
  sensor_msgs::PointCloud2 cloud_debug_ros;
  toROSMsg(*cloud_debug,cloud_debug_ros);
  pub_debug.publish(cloud_debug_ros);
  */
  ///////////////////

  for(int i = 0; i < bboxes_num; i++){
    pcl::compute3DCentroid(pc,class_point_container[i],tmp_cluster_centroid);
    tmp_centroid.point.x = tmp_cluster_centroid(0);
    tmp_centroid.point.y = tmp_cluster_centroid(1);
    tmp_centroid.point.z = tmp_cluster_centroid(2);
    class_centroids.push_back(tmp_centroid);
    class_names.push_back(job->bboxes_NewObjects.bounding_boxes[i].Class);
    
  }

  perception_msgs::ClassCentroids msg_result;
  msg_result.class_names = class_names;
  msg_result.class_centroids = class_centroids;

  pub_job_result.publish(msg_result);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "from2Dto3D");
    ros::NodeHandle nh;
    From2Dto3D node(nh);
    ros::spin();
    return 0;
}
