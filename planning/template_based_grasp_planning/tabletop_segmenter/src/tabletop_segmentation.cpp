/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
  
// Author(s): Marius Muja, Matei Ciocarlie
// Author(s) @ MPI: Jeannette Bohg and Alexander Herzog
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
// original potentially buggy version
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/transforms.h>

#include "tabletop_segmenter/marker_generator.h"
#include "tabletop_segmenter/utilities.h"
#include "tabletop_segmenter/TabletopSegmentation.h"

// includes for projecting stereo into same frame
#include <opencv/cv.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/foreach.hpp>

float getRGB( float r, float g, float b){
  union{ int intp; float floatp; } a;
  int res = (int(r*255) << 16) | (int(g*255) << 8) | int(b*255);
  a.intp=res;
  float rgb = *(&a.floatp);
  return rgb;
}

namespace enc = sensor_msgs::image_encodings;

namespace tabletop_segmenter {

typedef pcl::PointXYZ    Point;
typedef pcl::KdTree<Point>::Ptr KdTreePtr;

class TabletopSegmentor 
{
  
private:
  //! The node handle
  ros::NodeHandle nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Publisher for markers
  ros::Publisher marker_pub_;
  //! Publisher for markers
  ros::Publisher pcd_pub_;
  //! Service server for object detection
  ros::ServiceServer segmentation_srv_;

  //! Used to remember the number of markers we publish so we can delete them later
  int num_markers_published_;
  //! The current marker being published
  int current_marker_id_;

  //! Whether to merge stereo and xtion cloud
  bool merging_;

  //! Min number of inliers for reliable plane detection
  int inlier_threshold_;
  //! Size of downsampling grid before performing plane detection
  double plane_detection_voxel_size_;
  //! Size of downsampling grid before performing clustering
  double clustering_voxel_size_;
  //! Filtering of original point cloud along the z axis
  double z_filter_min_, z_filter_max_;
  //! Filtering of point cloud in table frame after table detection
  double table_z_filter_min_, table_z_filter_max_;
  //! Min distance between two clusters
  double cluster_distance_;
  //! Min number of points for a cluster
  int min_cluster_size_;
  //! Clouds are transformed into this frame before processing; leave empty if clouds
  //! are to be processed in their original frame
  std::string processing_frame_;
  //! Positive or negative z is closer to the "up" direction in the processing frame?
  double up_direction_;

  //! A tf transform listener
  tf::TransformListener listener_;

  //! Whether or not RGB mage has been received
  bool rgb_image_;

  //------------------ Callbacks -------------------

  //! Callback for service calls
  bool serviceCallback(TabletopSegmentation::Request &request, TabletopSegmentation::Response &response);

  //------------------ Individual processing steps -------

  //! Converts raw table detection results into a Table message type
  Table getTable(std_msgs::Header cloud_header, 
		 const tf::Transform &table_plane_trans,
		 const sensor_msgs::PointCloud &table_points);

  //! Publishes rviz markers for the given tabletop clusters
  void publishClusterMarkers(const std::vector<sensor_msgs::PointCloud2> &clusters, 
			     std_msgs::Header cloud_header);

  //! Publishes rviz markers for the given tabletop clusters
  void publishTablePoints(const sensor_msgs::PointCloud2 &table, 
			  std_msgs::Header cloud_header);

  //------------------- Complete processing -----

  //! Complete processing for new style point cloud
  void processCloud(const sensor_msgs::PointCloud2 &cloud,
		    const sensor_msgs::CameraInfo &cam_info,
		    TabletopSegmentation::Response &response);
  
  //! Merge Stereo and Xtion cloud into one while keeping image-based oragnisation
  bool mergeCloud( const sensor_msgs::PointCloud2::ConstPtr &ros_cloud_stereo,
		   const sensor_msgs::PointCloud2::ConstPtr &ros_cloud_xtion,
		   const sensor_msgs::CameraInfo::ConstPtr &cam_info,
		   tf::TransformListener &listener,
		   sensor_msgs::PointCloud2::Ptr &ros_cloud_merged);
  
  //! Clears old published markers and remembers the current number of published markers
  void clearOldMarkers(std::string frame_id);


public:
  //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
  TabletopSegmentor(ros::NodeHandle nh) 
  : nh_(nh)
  , priv_nh_("~")
  , rgb_image_(false)
  {
    num_markers_published_ = 1;
    current_marker_id_ = 1;

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("markers_out"), 10);

    pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("tabletop_clusters", 10);

    segmentation_srv_ = nh_.advertiseService(nh_.resolveName("segmentation_srv"),
                                             &TabletopSegmentor::serviceCallback, this);

    //initialize operational flags
    priv_nh_.param<bool>("merging", merging_, true);
    priv_nh_.param<int>("inlier_threshold", inlier_threshold_, 300);
    priv_nh_.param<double>("plane_detection_voxel_size", plane_detection_voxel_size_, 0.01);
    priv_nh_.param<double>("clustering_voxel_size", clustering_voxel_size_, 0.003);
    priv_nh_.param<double>("z_filter_min", z_filter_min_, 0.4);
    priv_nh_.param<double>("z_filter_max", z_filter_max_, 1.25);
    priv_nh_.param<double>("table_z_filter_min", table_z_filter_min_, 0.01);
    priv_nh_.param<double>("table_z_filter_max", table_z_filter_max_, 0.40);
    priv_nh_.param<double>("cluster_distance", cluster_distance_, 0.03);
    priv_nh_.param<int>("min_cluster_size", min_cluster_size_, 300);
    priv_nh_.param<std::string>("processing_frame", processing_frame_, "");
    priv_nh_.param<double>("up_direction", up_direction_, -1.0);   
  }

  //! Empty stub
  ~TabletopSegmentor() {}
};

/*! Processes the latest point cloud and gives back the resulting array of models.
 */
bool TabletopSegmentor::serviceCallback(TabletopSegmentation::Request &request, 
                                        TabletopSegmentation::Response &response)
{

  std::string topic = nh_.resolveName("/cam_info");
  std::cout << "Trying to get camera info from topic " << topic << std::endl;
  sensor_msgs::CameraInfo::ConstPtr cam_info = 
    ros::topic::waitForMessage<sensor_msgs::CameraInfo>(topic, nh_, ros::Duration(5.0));
  
  if (!cam_info)
    {
      ROS_ERROR("No camera info has been received");
      return true;
    }
  
  
  topic = nh_.resolveName("depth_in");
  sensor_msgs::Image::ConstPtr recent_depth =
    ros::topic::waitForMessage<sensor_msgs::Image>(topic, nh_, ros::Duration(5.0));
  
  if (!recent_depth)
    {
	ROS_ERROR("Tabletop object segmenter: no depth image has been received");
	response.result = response.NO_CLOUD_RECEIVED;
	return true;
  }

  topic = nh_.resolveName("rgb_in");
  sensor_msgs::Image::ConstPtr recent_rgb = 
    ros::topic::waitForMessage<sensor_msgs::Image>(topic, nh_, ros::Duration(5.0));

  if (!recent_rgb)
  {
    ROS_WARN("Tabletop object segmenter: no rgb image has been received");
    rgb_image_ = false;
  } else {
    rgb_image_ = true;
  }
  
  topic = nh_.resolveName("cloud_in");
  ROS_INFO("Tabletop service service called; waiting for a point_cloud2 on topic %s", topic.c_str());

  sensor_msgs::PointCloud2::ConstPtr recent_cloud = 
    ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_, ros::Duration(5.0));
              
  if (!recent_cloud)
  {
    ROS_ERROR("Tabletop object segmenter: no point_cloud2 has been received");
    response.result = response.NO_CLOUD_RECEIVED;
    return true;
  }
  
  // flag whether xtion and stereo data can be merged
  bool has_stereo_cloud = false;
  // flag whether data has been merged  
  bool merged_clouds    = false;
  // memory blob for merged point cloud if stereo cloud can be grabbed
  sensor_msgs::PointCloud2::Ptr 
    recent_cloud_merged(new sensor_msgs::PointCloud2);
  sensor_msgs::CameraInfo::ConstPtr stereo_cam_info;
  sensor_msgs::PointCloud2::ConstPtr recent_stereo_cloud;

  // if merging is desired, try to get stereo cloud and stereo camera info
  if(merging_) {
    topic = nh_.resolveName("/stereo_cam_info");
    std::cout << "Trying to get stereo camera info from topic " 
	      << topic << std::endl;
    stereo_cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(topic, nh_, ros::Duration(5.0));
    
    if (!stereo_cam_info)
      ROS_WARN("No stereo camera info has been received");
    
    topic = nh_.resolveName("stereo_cloud_in");
    ROS_INFO("Waiting for a point_cloud2 on topic %s", topic.c_str());
    
    recent_stereo_cloud =  ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_, ros::Duration(5.0));
    
    if (!recent_stereo_cloud || !stereo_cam_info)
      {
	ROS_WARN("Could not grab a point cloud or camera info from topic %s\n", 
		 topic.c_str());
      } else {
      has_stereo_cloud = true;
      ROS_INFO("Grabbed a stereo point cloud of size %ld\n", 
	       (long int)recent_stereo_cloud->data.size());
    }
  }
  
  if(has_stereo_cloud){
    if(!mergeCloud(recent_stereo_cloud, 
		   recent_cloud, 
		   stereo_cam_info, 
		   listener_, 
		   recent_cloud_merged)){
      *recent_cloud_merged = *recent_cloud;
      ROS_WARN("Using pure XTION cloud\n");
    } else {
      ROS_INFO("Using point cloud merged from XTION and stereo camera\n");
      merged_clouds=true;
    }
      
  } else {
    *recent_cloud_merged = *recent_cloud;
    ROS_INFO("Using pure XTION cloud\n");
  }
  
  
  ROS_INFO("Point cloud received; processing");
  if (!processing_frame_.empty())
    {
      //convert cloud to base link frame
      sensor_msgs::PointCloud old_cloud;  
      sensor_msgs::convertPointCloud2ToPointCloud (*recent_cloud_merged, 
						   old_cloud);
      try
	{
	  listener_.transformPointCloud(processing_frame_, 
					old_cloud, 
					old_cloud);    
	}
      catch (tf::TransformException& ex)
	{
	  ROS_ERROR("Failed to transform cloud from frame %s into frame %s",
		    old_cloud.header.frame_id.c_str(), 
		    processing_frame_.c_str());
	  response.result = response.OTHER_ERROR;
	  return true;
	}
    sensor_msgs::PointCloud2 converted_cloud;
    sensor_msgs::convertPointCloudToPointCloud2 (old_cloud, 
						 converted_cloud);
    ROS_INFO("Input cloud converted to %s frame", 
	     processing_frame_.c_str());
    if(merged_clouds)
      processCloud(converted_cloud, *stereo_cam_info, response);
    else 
      processCloud(converted_cloud, *cam_info, response);
    clearOldMarkers(converted_cloud.header.frame_id);
  }
  else
  {
    if(merged_clouds)
      processCloud(*recent_cloud_merged, *stereo_cam_info, response);
    else {
      ROS_INFO("Processing cloud");
      processCloud(*recent_cloud_merged, *cam_info, response);
    }
    clearOldMarkers(recent_cloud_merged->header.frame_id);
  }
  
  if(recent_depth)
    response.depth = *recent_depth;

  if(rgb_image_)
    response.rgb = *recent_rgb;
  
  response.cam_info = *cam_info;

  if(rgb_image_)
    storeBag(recent_cloud_merged, response.clusters, cam_info, recent_rgb);
  else 
    storeBag(recent_cloud_merged, response.clusters, cam_info);

  return true;
}


Table TabletopSegmentor::getTable(std_msgs::Header cloud_header,
                                  const tf::Transform &table_plane_trans, 
                                  const sensor_msgs::PointCloud &table_points)
{
  Table table;
  
  //get the extents of the table
  if (!table_points.points.empty()) 
  {
    table.x_min = table_points.points[0].x;
    table.x_max = table_points.points[0].x;
    table.y_min = table_points.points[0].y;
    table.y_max = table_points.points[0].y;
  }  
  for (size_t i=1; i<table_points.points.size(); ++i) 
    {
    // if (table_points.points[i].x<table.x_min && table_points.points[i].x>-3.0) table.x_min = table_points.points[i].x;
    // if (table_points.points[i].x>table.x_max && table_points.points[i].x< 3.0) table.x_max = table_points.points[i].x;
    // if (table_points.points[i].y<table.y_min && table_points.points[i].y>-3.0) table.y_min = table_points.points[i].y;
    // if (table_points.points[i].y>table.y_max && table_points.points[i].y< 3.0) table.y_max = table_points.points[i].y;

    if (table_points.points[i].x<table.x_min ) table.x_min = table_points.points[i].x;
    if (table_points.points[i].x>table.x_max ) table.x_max = table_points.points[i].x;
    if (table_points.points[i].y<table.y_min ) table.y_min = table_points.points[i].y;
    if (table_points.points[i].y>table.y_max ) table.y_max = table_points.points[i].y;

  }

  geometry_msgs::Pose table_pose;
  tf::poseTFToMsg(table_plane_trans, table_pose);
  table.pose.pose = table_pose;
  table.pose.header = cloud_header;

  sensor_msgs::PointCloud2 table_points_2;
  convertPointCloudToPointCloud2(table_points, table_points_2);
  table.table_points = table_points_2;

  visualization_msgs::Marker tableMarker = MarkerGenerator::getTableMarker(table.x_min, table.x_max,
                                                                           table.y_min, table.y_max);

  ROS_INFO("Frame of table %s.", cloud_header.frame_id.c_str());
  tableMarker.header = cloud_header;
  tableMarker.pose = table_pose;
  tableMarker.ns = "tabletop_node";
  tableMarker.id = current_marker_id_++;
  marker_pub_.publish(tableMarker);

  return table;
}

void TabletopSegmentor::publishClusterMarkers(const std::vector<sensor_msgs::PointCloud2> &clusters, 
					      std_msgs::Header cloud_header)
{
  for (size_t i=0; i<clusters.size(); i++) 
  {
    visualization_msgs::Marker cloud_marker =  MarkerGenerator::getCloudMarker(clusters[i]);
    ROS_INFO("Frame of markers %s.", cloud_header.frame_id.c_str());
    cloud_marker.header.frame_id = cloud_header.frame_id;
    cloud_marker.header.stamp = ros::Time::now();
    cloud_marker.pose.orientation.w = 1;
    cloud_marker.ns = "tabletop_node";
    cloud_marker.id = current_marker_id_++;
    marker_pub_.publish(cloud_marker);
  }
}
    
void TabletopSegmentor::publishTablePoints(const sensor_msgs::PointCloud2 &table, 
					   std_msgs::Header cloud_header)
{
    visualization_msgs::Marker cloud_marker =  MarkerGenerator::getCloudMarker(table);
    ROS_INFO("Frame of markers %s.", cloud_header.frame_id.c_str());
    cloud_marker.header.frame_id = cloud_header.frame_id;
    cloud_marker.header.stamp = ros::Time::now();
    cloud_marker.pose.orientation.w = 1;
    cloud_marker.ns = "tabletop_node";
    cloud_marker.id = current_marker_id_++;
    marker_pub_.publish(cloud_marker);
}

void TabletopSegmentor::clearOldMarkers(std::string frame_id)
{
  for (int id=current_marker_id_; id < num_markers_published_; id++)
    {
      visualization_msgs::Marker delete_marker;
      delete_marker.header.stamp = ros::Time::now();
      delete_marker.header.frame_id = frame_id;
      delete_marker.id = id;
      delete_marker.action = visualization_msgs::Marker::DELETE;
      delete_marker.ns = "tabletop_node";
      marker_pub_.publish(delete_marker);
    }
  num_markers_published_ = current_marker_id_;
  current_marker_id_ = 0;
}

/*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
tf::Transform getPlaneTransform (pcl::ModelCoefficients coeffs, double up_direction)
{
  ROS_ASSERT(coeffs.values.size() > 3);
  double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
  //asume plane coefficients are normalized
  btVector3 position(-a*d, -b*d, -c*d);
  btVector3 z(a, b, c);
  //make sure z points "up"
  ROS_DEBUG("z.dot: %0.3f", z.dot(btVector3(0,0,1)));
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
  if ( z.dot( btVector3(0, 0, up_direction) ) < 0)
  {
    z = -1.0 * z;
    ROS_INFO("flipped z");
  }
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);

  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  btVector3 x(1, 0, 0);
  if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = btVector3(0, 1, 0);
  btVector3 y = z.cross(x).normalized();
  x = y.cross(z).normalized();

  btMatrix3x3 rotation;
  rotation[0] = x; 	// x
  rotation[1] = y; 	// y
  rotation[2] = z; 	// z
  rotation = rotation.transpose();
  btQuaternion orientation;
  rotation.getRotation(orientation);
  return tf::Transform(orientation, position);
}

template <typename PointT> 
bool getPlanePoints (const pcl::PointCloud<PointT> &table, 
		     const tf::Transform& table_plane_trans,
		     sensor_msgs::PointCloud &table_points)
{
  // Prepare the output
  table_points.header = table.header;
  table_points.points.resize (table.points.size ());
  for (size_t i = 0; i < table.points.size (); ++i)
  {
    table_points.points[i].x = table.points[i].x;
    table_points.points[i].y = table.points[i].y;
    table_points.points[i].z = table.points[i].z;
  }

  // Transform the data
  tf::TransformListener listener;
  tf::StampedTransform table_pose_frame(table_plane_trans, table.header.stamp, 
                                        table.header.frame_id, "table_frame");
  listener.setTransform(table_pose_frame);
  std::string error_msg;
  if (!listener.canTransform("table_frame", 
			     table_points.header.frame_id, 
			     table_points.header.stamp, 
			     &error_msg))
  {
    ROS_ERROR("Cannot transform point cloud from frame %s to table frame; error %s", 
	      table_points.header.frame_id.c_str(), error_msg.c_str());
    return false;
  }
  try
  {
    listener.transformPointCloud("table_frame", table_points, table_points);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("Failed to transform point cloud from frame %s into table_frame; error %s", 
	      table_points.header.frame_id.c_str(), ex.what());
    return false;
  }
  table_points.header.stamp = table.header.stamp;
  table_points.header.frame_id = "table_frame";
  return true;
}

void getMasksFromClusters(const std::vector<sensor_msgs::PointCloud2> &clusters, 
			  const sensor_msgs::CameraInfo &cam_info,
			  std::vector<sensor_msgs::Image> &masks)
{
  masks.resize(clusters.size());

  Eigen::Matrix4f P;
  int rows=3, cols=4;
  for(int r=0; r<rows; ++r) 
    for(int c=0; c<cols; ++c) 
      P(r,c) = cam_info.P[r*cols+c];
    
  P(3,0) = 0;
  P(3,1) = 0;
  P(3,2) = 0;
  P(3,3) = 1;

  std::cout << "Transformation Matrix " << std::endl << P << std::endl;

  for(size_t i=0; i<clusters.size(); ++i) {
    
    sensor_msgs::PointCloud2 cloud_proj;

    sensor_msgs::Image mask;
    mask.height = cam_info.height;
    mask.width = cam_info.width;
    //mask.encoding = enc::TYPE_32FC1;
    mask.encoding = enc::MONO8;
    mask.is_bigendian = false;
    mask.step = mask.width;
    size_t size = mask.step * mask.height;
    mask.data.resize(size);

    pcl_ros::transformPointCloud( P, clusters[i], cloud_proj);
    
    for(unsigned int j=0; j < cloud_proj.width; j++){
    
      float x, y, z;
	
      memcpy (&x,
	      &cloud_proj.data[j * cloud_proj.point_step + cloud_proj.fields[0].offset], 
	      sizeof (float));
      memcpy (&y,
	      &cloud_proj.data[j * cloud_proj.point_step + cloud_proj.fields[1].offset], 
	      sizeof (float));
      memcpy (&z,
	      &cloud_proj.data[j * cloud_proj.point_step + cloud_proj.fields[2].offset], 
	      sizeof (float));
    
      if( round(y/z) >= 0 && round(y/z)<mask.height && round(x/z) >= 0 && round(x/z)<mask.width) {
	int i = round(y/z) * mask.step + round(x/z);
	mask.data[i] = 255;
      }
    }

    masks[i]=mask;
  }
}

template <typename PointT> void
getClustersFromPCLCloud (const pcl::PointCloud<PointT> &cloud_objects, 			    
			 const std::vector<pcl::PointIndices> &clusters2,
			 std::vector<sensor_msgs::PointCloud2> &clusters)
{
  
  clusters.resize (clusters2.size ());
  for (size_t i = 0; i < clusters2.size (); ++i)
    {
      clusters[i].header.frame_id = cloud_objects.header.frame_id;
      clusters[i].header.stamp = ros::Time::now();
      clusters[i].width        = clusters2[i].indices.size ();
      clusters[i].height       = 1;
      clusters[i].is_dense     = true;
      clusters[i].is_bigendian = false;
      // size of field is 3 for x, y, z and rgb
      clusters[i].fields.resize( 3 + 1);
      clusters[i].fields[0].name = "x"; 
      clusters[i].fields[1].name = "y"; 
      clusters[i].fields[2].name = "z";
      clusters[i].fields[3].name = "rgb";
      int offset = 0;
      for (size_t d = 0; d < clusters[i].fields.size (); ++d, offset += 4) {
	clusters[i].fields[d].offset = offset;
	clusters[i].fields[d].datatype = sensor_msgs::PointField::FLOAT32;
	clusters[i].fields[d].count  = 1;
      }
  
      clusters[i].point_step = offset;
      clusters[i].row_step   = clusters[i].point_step * clusters[i].width;

      clusters[i].data.resize (clusters[i].width * clusters[i].height * clusters[i].point_step);
    
      float rgb = getRGB(1.0,0.0,0.0);

      for(unsigned int j=0; j < clusters[i].width; j++){
	
	memcpy (&clusters[i].data[j * clusters[i].point_step + clusters[i].fields[0].offset], 
		&cloud_objects.points[clusters2[i].indices[j]].x, sizeof (float));
	memcpy (&clusters[i].data[j * clusters[i].point_step + clusters[i].fields[1].offset], 
		&cloud_objects.points[clusters2[i].indices[j]].y, sizeof (float));
	memcpy (&clusters[i].data[j * clusters[i].point_step + clusters[i].fields[2].offset], 
		&cloud_objects.points[clusters2[i].indices[j]].z, sizeof (float));
	memcpy (&clusters[i].data[j * clusters[i].point_step + clusters[i].fields[3].offset], 
		&rgb, sizeof (float));

      }
    }
}

void TabletopSegmentor::processCloud(const sensor_msgs::PointCloud2 &cloud,
				     const sensor_msgs::CameraInfo &cam_info,
                                     TabletopSegmentation::Response &response)
{
  ROS_INFO("Starting process on new cloud");
  ROS_INFO("In frame %s", cloud.header.frame_id.c_str());

  // PCL objects
  boost::shared_ptr<pcl::search::Search<Point> > normals_tree_, clusters_tree_;
  pcl::VoxelGrid<Point> grid_, grid_objects_;
  pcl::PassThrough<Point> pass_;
  pcl::NormalEstimation<Point, pcl::Normal> n3d_;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
  pcl::ProjectInliers<Point> proj_;
  pcl::ConvexHull<Point> hull_;
  pcl::ExtractPolygonalPrismData<Point> prism_;
  pcl::EuclideanClusterExtraction<Point> pcl_cluster_;

  // Filtering parameters
  grid_.setLeafSize (plane_detection_voxel_size_, plane_detection_voxel_size_, plane_detection_voxel_size_);
  grid_objects_.setLeafSize (clustering_voxel_size_, clustering_voxel_size_, clustering_voxel_size_);
  grid_.setFilterFieldName ("z");
  pass_.setFilterFieldName ("z");

  pass_.setFilterLimits (z_filter_min_, z_filter_max_);
  grid_.setFilterLimits (z_filter_min_, z_filter_max_);
  grid_.setDownsampleAllData (false);
  grid_objects_.setDownsampleAllData (false);

  normals_tree_ = boost::make_shared<pcl::search::KdTree<Point> > ();
  clusters_tree_ = boost::make_shared<pcl::search::KdTree<Point> > ();

  // Normal estimation parameters
  n3d_.setKSearch (10);  
  n3d_.setSearchMethod (normals_tree_);
  // Table model fitting parameters
  seg_.setDistanceThreshold (0.05); 
  seg_.setMaxIterations (10000);
  seg_.setNormalDistanceWeight (0.1);
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setProbability (0.99);

  proj_.setModelType (pcl::SACMODEL_PLANE);

  // Clustering parameters
  pcl_cluster_.setClusterTolerance (cluster_distance_);
  pcl_cluster_.setMinClusterSize (min_cluster_size_);
  pcl_cluster_.setSearchMethod (clusters_tree_);

  // Step 1 : Filter, remove NaNs and downsample
  pcl::PointCloud<Point>::Ptr cloud_ptr(new pcl::PointCloud<Point> ());
  pcl::fromROSMsg (cloud, *cloud_ptr);

  pcl::PointCloud<Point>::Ptr cloud_filtered_ptr(new pcl::PointCloud<Point> ());
  pass_.setInputCloud (cloud_ptr);
  pass_.filter (*cloud_filtered_ptr);
  ROS_INFO("Step 1 done");
  if (cloud_filtered_ptr->points.size() < (unsigned int)min_cluster_size_)
  {
    ROS_INFO("Filtered cloud only has %ld points", (long int)cloud_filtered_ptr->points.size());
    response.result = response.NO_TABLE;
    return;
  }

  pcl::PointCloud<Point>::Ptr cloud_downsampled_ptr(new pcl::PointCloud<Point> ());
  grid_.setInputCloud (cloud_filtered_ptr);
  grid_.filter (*cloud_downsampled_ptr);
  if (cloud_downsampled_ptr->points.size() < (unsigned int)min_cluster_size_)
  {
    ROS_INFO("Downsampled cloud only has %ld points", (long int)cloud_downsampled_ptr->points.size());
    response.result = response.NO_TABLE;    
    return;
  }


  // Step 2 : Estimate normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr(new pcl::PointCloud<pcl::Normal> ());
  n3d_.setInputCloud (cloud_downsampled_ptr);
  n3d_.compute (*cloud_normals_ptr);
  ROS_INFO("Step 2 done");

  // Step 3 : Perform planar segmentation
  pcl::PointIndices::Ptr table_inliers_ptr(new pcl::PointIndices ());
  pcl::ModelCoefficients::Ptr table_coefficients_ptr(new pcl::ModelCoefficients ());
  seg_.setInputCloud (cloud_downsampled_ptr);
  seg_.setInputNormals (cloud_normals_ptr);
  seg_.segment (*table_inliers_ptr, *table_coefficients_ptr);

  if (table_coefficients_ptr->values.size () <=3)
    {
      ROS_INFO("Failed to detect table in scan");
      response.result = response.NO_TABLE;    
      return;
    }
  
  if ( table_inliers_ptr->indices.size() < (unsigned int)inlier_threshold_)
    {
      ROS_INFO("Plane detection has %d inliers, below min threshold of %d", (int)table_inliers_ptr->indices.size(),
	       inlier_threshold_);
      response.result = response.NO_TABLE;
      return;
    }

  ROS_INFO ("[TableObjectDetector::input_callback] Table found with %d inliers: [%f %f %f %f].", 
	    (int)table_inliers_ptr->indices.size (),
	    table_coefficients_ptr->values[0], table_coefficients_ptr->values[1], 
	    table_coefficients_ptr->values[2], table_coefficients_ptr->values[3]);
  ROS_INFO("Step 3 done");

  // Step 4 : Project the table inliers on the table
  pcl::PointCloud<Point>::Ptr table_projected_ptr(new pcl::PointCloud<Point> ());
  proj_.setInputCloud (cloud_downsampled_ptr);
  proj_.setIndices (table_inliers_ptr);
  proj_.setModelCoefficients (table_coefficients_ptr);
  proj_.filter (*table_projected_ptr);
  ROS_INFO("Step 4 done");

  sensor_msgs::PointCloud table_points;
  tf::Transform table_plane_trans = getPlaneTransform (*table_coefficients_ptr, up_direction_);
  //takes the points projected on the table and transforms them into the PointCloud message
  //while also transforming them into the table's coordinate system
  if (!getPlanePoints<Point> (*table_projected_ptr, table_plane_trans, table_points))
    {
      response.result = response.OTHER_ERROR;
      return;
    }
  ROS_INFO("Table computed");
 
  response.table = getTable(cloud.header, table_plane_trans, table_points);

  if(rgb_image_)
    response.result = response.SUCCESS;
  else 
    response.result = response.SUCCESS_NO_RGB;
  
  // ---[ Estimate the convex hull on 3D data
  pcl::PointCloud<Point>::Ptr table_hull_ptr(new pcl::PointCloud<Point> ());
  std::vector< pcl::Vertices > polygons;
  //  hull_.setDimension(3);
  hull_.setInputCloud (table_projected_ptr);
  hull_.reconstruct (*table_hull_ptr, polygons);

  // ---[ Get the objects on top of the table
  pcl::PointIndices::Ptr cloud_object_indices_ptr(new pcl::PointIndices ());
  prism_.setInputCloud (cloud_filtered_ptr);
  prism_.setInputPlanarHull (table_hull_ptr);
  ROS_INFO("Using table prism: %f to %f", table_z_filter_min_, table_z_filter_max_);
  prism_.setHeightLimits (table_z_filter_min_, table_z_filter_max_);
  prism_.segment (*cloud_object_indices_ptr);
 

  pcl::PointCloud<Point>::Ptr cloud_objects_ptr(new pcl::PointCloud<Point> ());
  pcl::ExtractIndices<Point> extract_object_indices;
  extract_object_indices.setInputCloud (cloud_filtered_ptr);
  extract_object_indices.setIndices (cloud_object_indices_ptr);
  extract_object_indices.filter (*cloud_objects_ptr);
  
  ROS_INFO (" Number of object point candidates: %d.", (int)cloud_objects_ptr->points.size ());

  if (cloud_objects_ptr->points.empty ()) 
  {
    ROS_INFO("No objects on table");
    return;
  }


  // ---[ Downsample the points
  pcl::PointCloud<Point>::Ptr cloud_objects_downsampled_ptr(new pcl::PointCloud<Point> ());
  grid_objects_.setInputCloud (cloud_objects_ptr);
  grid_objects_.filter (*cloud_objects_downsampled_ptr);

  // ---[ Split the objects into Euclidean clusters
  std::vector<pcl::PointIndices> clusters2;
  //pcl_cluster_.setInputCloud (cloud_objects_ptr);
  pcl_cluster_.setInputCloud (cloud_objects_downsampled_ptr);
  pcl_cluster_.extract (clusters2);
  ROS_INFO ("Number of clusters found matching the given constraints: %d.", (int)clusters2.size ());
  for( size_t i=0; i<clusters2.size(); ++i)
    ROS_INFO ("Cluster %d has %d points.", (int)i, (int)clusters2[i].indices.size());

  //converts clusters into the PointCloud message
  std::vector<sensor_msgs::PointCloud2> clusters;
  getClustersFromPCLCloud<Point> (*cloud_objects_downsampled_ptr, clusters2, clusters);
  ROS_INFO("Clusters converted");
  response.clusters = clusters;  

  // get segmentation mask in image by backprojecting clusters
  std::vector<sensor_msgs::Image> masks;
  getMasksFromClusters(clusters, cam_info, masks);
  ROS_INFO("Masks generated.");
  response.masks = masks;
  

  // DEBUG: Publish point clouds to rviz
//  pcd_pub_.publish(clusters[0]);


  publishClusterMarkers(clusters, cloud.header);
}

bool TabletopSegmentor::mergeCloud( const sensor_msgs::PointCloud2::ConstPtr &ros_cloud_stereo,
				    const sensor_msgs::PointCloud2::ConstPtr &ros_cloud_xtion,
				    const sensor_msgs::CameraInfo::ConstPtr &cam_info,
				    tf::TransformListener &listener,
				    sensor_msgs::PointCloud2::Ptr &ros_cloud_merged)
{  
  // Create pinhole camera model from stereo camera
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(*cam_info);

  // get the transform from Xtion to stereo
  tf::StampedTransform transform;
  bool can_transform = false;
  while(!can_transform){
    try {
      ros::Time acquisition_time = cam_info->header.stamp;
      ros::Duration timeout(1.0 / 30);
      ROS_INFO("Trying to look up transform from %s to %s\n", 
	       ros_cloud_xtion->header.frame_id.c_str(),
	       ros_cloud_stereo->header.frame_id.c_str());
      can_transform = listener.waitForTransform(ros_cloud_xtion->header.frame_id, 
						ros_cloud_stereo->header.frame_id,
						acquisition_time, timeout);
      ros::spinOnce();
    }
    catch (tf::TransformException& ex) {
      ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
      return false;
    }
  }

  if(can_transform) {
    //convert cloud to stereo frame and do annoying conversion from PointCloud2 to PointCloud
    sensor_msgs::PointCloud old_cloud;  
    sensor_msgs::convertPointCloud2ToPointCloud (*ros_cloud_xtion, old_cloud);
    try
      {
	listener.transformPointCloud(ros_cloud_stereo->header.frame_id, old_cloud, old_cloud);  
      }	
    catch (tf::TransformException& ex)
      {
	ROS_ERROR("Failed to transform cloud from frame %s into frame %s", old_cloud.header.frame_id.c_str(), 
		  ros_cloud_stereo->header.frame_id.c_str());
	return false;
      }
    
    ROS_INFO("Transformed Xtion cloud into stereo frame\n");

    // copy stereo cloud into merged cloud 
    *ros_cloud_merged = *ros_cloud_stereo;
    
    // collecting some statistics
    int projected_points = 0;
    int nan_projected_points = 0;
    int merged_projected_points = 0;
    // project XTIOn point cloud into depth map of stereo to merge data
    BOOST_FOREACH(geometry_msgs::Point32 pt,  old_cloud.points) {
      
      if(pt.x!=pt.x && pt.y!=pt.y && pt.z!=pt.z)
	continue;
      
      cv::Point3d pt_cv(pt.x, pt.y, pt.z);
      cv::Point2d uv;
      cam_model.project3dToPixel(pt_cv, uv);
      uv.x = std::floor(uv.x);
      uv.y = std::floor(uv.y);
      if( uv.x>0 && uv.x < cam_info->width &&  
	    uv.y > 0 && uv.y < cam_info->height )  {
	projected_points++;
	cv::Point3f p;
	int i = uv.y*cam_info->width+uv.x;
	memcpy(&p.x, 
	       &ros_cloud_stereo->data[i * ros_cloud_stereo->point_step + 
				       ros_cloud_stereo->fields[0].offset],
	       sizeof (float));
	
	memcpy (&p.y, 
		&ros_cloud_stereo->data[i * ros_cloud_stereo->point_step + 
					ros_cloud_stereo->fields[1].offset],
		sizeof (float));
	memcpy (&p.z, 
		&ros_cloud_stereo->data[i * ros_cloud_stereo->point_step + 
					ros_cloud_stereo->fields[2].offset],
		sizeof (float));
	float rgb;
	memcpy (&rgb, 
		&ros_cloud_stereo->data[i * ros_cloud_stereo->point_step + 
					ros_cloud_stereo->fields[3].offset],
		sizeof (float));
	
	
	// NaN test
	if(p.x!=p.x && p.y!=p.y && p.z!=p.z) {
	  nan_projected_points++;
	
	  float tmp = pt_cv.x;
	  // copy data from transformed xtion cloud directly into merged cloud
	  memcpy( &ros_cloud_merged->data[i * 
					  ros_cloud_merged->point_step + 
					  ros_cloud_merged->fields[0].offset], 
		  &tmp,
		  sizeof (float));
	  
	  tmp = pt_cv.y;
	  memcpy( &ros_cloud_merged->data[i * 
					  ros_cloud_merged->point_step + 
					  ros_cloud_merged->fields[1].offset], 
		  &tmp,
		  sizeof (float));
	  tmp = pt_cv.z;
	  memcpy ( &ros_cloud_merged->data[i * 
					   ros_cloud_merged->point_step + 
					   ros_cloud_merged->fields[2].offset], 
		   &tmp, sizeof (float));

	  float green = getRGB(0.0f, 255.0f, 0.0f);
	  memcpy ( &ros_cloud_merged->data[i * 
					   ros_cloud_merged->point_step + 
					   ros_cloud_merged->fields[3].offset], 
		   &green, sizeof (float));

	} else if(pt_cv.x!=pt_cv.x && pt_cv.y!=pt_cv.y && pt_cv.z!=pt_cv.z) {
	  // NaN test for stereo data
	   float tmp = p.x;
	  // copy data from transformed xtion cloud directly into merged cloud
	  memcpy( &ros_cloud_merged->data[i * 
					  ros_cloud_merged->point_step + 
					  ros_cloud_merged->fields[0].offset], 
		  &tmp,
		  sizeof (float));
	  
	  tmp = p.y;
	  memcpy( &ros_cloud_merged->data[i * 
					  ros_cloud_merged->point_step + 
					  ros_cloud_merged->fields[1].offset], 
		  &tmp,
		  sizeof (float));
	  tmp = p.z;
	  memcpy ( &ros_cloud_merged->data[i * 
					   ros_cloud_merged->point_step + 
					   ros_cloud_merged->fields[2].offset], 
		   &tmp, sizeof (float));

	  float green = getRGB(0.0f, 255.0f, 0.0f);
	  memcpy ( &ros_cloud_merged->data[i * 
					   ros_cloud_merged->point_step + 
					   ros_cloud_merged->fields[3].offset], 
		   &green, sizeof (float));
	  

	} else {
	  merged_projected_points++;

	  // test whether difference is too large between stereo and xtion data
	  cv::Point3f diff( pt_cv.x - p.x, pt_cv.y - p.y, pt_cv.z - p.z);
	  float mag = std::sqrt(diff.x*diff.x + diff.y*diff.y + diff.z*diff.z);
	  cv::Point3f pt_avg;
	  // if difference is too big, we use the xtion data directly, otherwise, we average
	  if(mag>0.02){
	    pt_avg.x = pt_cv.x;
	    pt_avg.y = pt_cv.y;
	    pt_avg.z = pt_cv.z;
	  } else {
	    pt_avg.x = (p.x+pt_cv.x)/2.0f;
	    pt_avg.y = (p.y+pt_cv.y)/2.0f;
	    pt_avg.z = (p.z+pt_cv.z)/2.0f;
	  }
	  
	  // average data if distance is not too big
	  float tmp = pt_avg.x;
	  memcpy( &ros_cloud_merged->data[i * ros_cloud_merged->point_step + 
					  ros_cloud_merged->fields[0].offset], 
		  &tmp,
		  sizeof (float));
	  
	  tmp = pt_avg.y;
	  memcpy( &ros_cloud_merged->data[i * ros_cloud_merged->point_step + 
					  ros_cloud_merged->fields[1].offset], 
		  &tmp,
		  sizeof (float));
	  tmp = pt_avg.z;
	  memcpy ( &ros_cloud_merged->data[i * ros_cloud_merged->point_step + 
					   ros_cloud_merged->fields[2].offset], 
		   &tmp, sizeof (float));
	  memcpy ( &ros_cloud_merged->data[i * ros_cloud_merged->point_step + 
					   ros_cloud_merged->fields[3].offset], 
		   &rgb, sizeof (float));
	  
	}
      }
    }
    ROS_INFO("Projected points: %d, Nan points: %d, Merged points: %d.", 
	     projected_points, 
	     nan_projected_points,
	     merged_projected_points);
  }


  return true;
}


} //namespace tabletop_segmenter

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "tabletop_segmentation_node");
  ros::NodeHandle nh;

  tabletop_segmenter::TabletopSegmentor node(nh);
  ros::spin();
  return 0;
}
