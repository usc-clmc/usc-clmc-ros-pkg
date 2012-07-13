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
  
// Author(s): Marius Muja and Matei Ciocarlie

#include <string>

#include <ros/ros.h>

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
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/transforms.h>

#include "tabletop_segmenter/marker_generator.h"
#include "tabletop_segmenter/TabletopSegmentation.h"

float getRGB( float r, float g, float b){
  union{ int intp; float floatp; } a;
  int res = (int(r*255) << 16) | (int(g*255) << 8) | int(b*255);
  a.intp=res;
  float rgb = *(&a.floatp);
  return rgb;
}

namespace enc = sensor_msgs::image_encodings;

namespace tabletop_segmenter {

class TabletopSegmentor 
{
  typedef pcl::PointXYZ    Point;
  typedef pcl::KdTree<Point>::Ptr KdTreePtr;
  
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

  //------------------- Complete processing -----

  //! Complete processing for new style point cloud
  void processCloud(const sensor_msgs::PointCloud2 &cloud,
		    const sensor_msgs::CameraInfo &cam_info,
		    TabletopSegmentation::Response &response);
  
  //! Clears old published markers and remembers the current number of published markers
  void clearOldMarkers(std::string frame_id);

public:
  //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
  TabletopSegmentor(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
    num_markers_published_ = 1;
    current_marker_id_ = 1;

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("markers_out"), 10);

    pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("tabletop_clusters", 10);

    segmentation_srv_ = nh_.advertiseService(nh_.resolveName("segmentation_srv"),
                                             &TabletopSegmentor::serviceCallback, this);

    //initialize operational flags
    priv_nh_.param<int>("inlier_threshold", inlier_threshold_, 300);
    priv_nh_.param<double>("plane_detection_voxel_size", plane_detection_voxel_size_, 0.01);
    priv_nh_.param<double>("clustering_voxel_size", clustering_voxel_size_, 0.003);
    priv_nh_.param<double>("z_filter_min", z_filter_min_, 0.4);
    priv_nh_.param<double>("z_filter_max", z_filter_max_, 1.25);
    priv_nh_.param<double>("table_z_filter_min", table_z_filter_min_, 0.01);
    priv_nh_.param<double>("table_z_filter_max", table_z_filter_max_, 0.50);
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
    ROS_ERROR("Tabletop object segmenter: no rgb image has been received");
    response.result = response.NO_CLOUD_RECEIVED;
    return true;
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

  ROS_INFO("Point cloud received; processing");
  if (!processing_frame_.empty())
  {
    //convert cloud to base link frame
    sensor_msgs::PointCloud old_cloud;  
    sensor_msgs::convertPointCloud2ToPointCloud (*recent_cloud, old_cloud);
    try
    {
      listener_.transformPointCloud(processing_frame_, old_cloud, old_cloud);    
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("Failed to transform cloud from frame %s into frame %s", old_cloud.header.frame_id.c_str(), 
		processing_frame_.c_str());
      response.result = response.OTHER_ERROR;
      return true;
    }
    sensor_msgs::PointCloud2 converted_cloud;
    sensor_msgs::convertPointCloudToPointCloud2 (old_cloud, converted_cloud);
    ROS_INFO("Input cloud converted to %s frame", processing_frame_.c_str());
    processCloud(converted_cloud, *cam_info, response);
    clearOldMarkers(converted_cloud.header.frame_id);
  }
  else
  {
    processCloud(*recent_cloud, *cam_info, response);
    clearOldMarkers(recent_cloud->header.frame_id);
  }

  response.depth = *recent_depth;
  response.rgb = *recent_rgb;
  response.cam_info = *cam_info;


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
    if (table_points.points[i].x<table.x_min && table_points.points[i].x>-3.0) table.x_min = table_points.points[i].x;
    if (table_points.points[i].x>table.x_max && table_points.points[i].x< 3.0) table.x_max = table_points.points[i].x;
    if (table_points.points[i].y<table.y_min && table_points.points[i].y>-3.0) table.y_min = table_points.points[i].y;
    if (table_points.points[i].y>table.y_max && table_points.points[i].y< 3.0) table.y_max = table_points.points[i].y;
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
  catch (tf::TransformException ex)
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
  KdTreePtr normals_tree_, clusters_tree_;
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

  normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
  clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();

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
  pcl::PointCloud<Point> cloud_t;
  pcl::fromROSMsg (cloud, cloud_t);
  pcl::PointCloud<Point>::ConstPtr cloud_ptr 
    = boost::make_shared<const pcl::PointCloud<Point> > (cloud_t);

  pcl::PointCloud<Point> cloud_filtered;
  pass_.setInputCloud (cloud_ptr);
  pass_.filter (cloud_filtered);
  pcl::PointCloud<Point>::ConstPtr cloud_filtered_ptr = 
    boost::make_shared<const pcl::PointCloud<Point> > (cloud_filtered);
  ROS_INFO("Step 1 done");
  if (cloud_filtered.points.size() < (unsigned int)min_cluster_size_)
  {
    ROS_INFO("Filtered cloud only has %d points", (int)cloud_filtered.points.size());
    response.result = response.NO_TABLE;
    return;
  }

  pcl::PointCloud<Point> cloud_downsampled;
  grid_.setInputCloud (cloud_filtered_ptr);
  grid_.filter (cloud_downsampled);
  pcl::PointCloud<Point>::ConstPtr cloud_downsampled_ptr = 
    boost::make_shared<const pcl::PointCloud<Point> > (cloud_downsampled);
  if (cloud_downsampled.points.size() < (unsigned int)min_cluster_size_)
  {
    ROS_INFO("Downsampled cloud only has %d points", (int)cloud_downsampled.points.size());
    response.result = response.NO_TABLE;    
    return;
  }

  // Step 2 : Estimate normals
  pcl::PointCloud<pcl::Normal> cloud_normals;
  n3d_.setInputCloud (cloud_downsampled_ptr);
  n3d_.compute (cloud_normals);
  pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_ptr = 
    boost::make_shared<const pcl::PointCloud<pcl::Normal> > (cloud_normals);
  ROS_INFO("Step 2 done");

  // Step 3 : Perform planar segmentation
  pcl::PointIndices table_inliers; pcl::ModelCoefficients table_coefficients;
  seg_.setInputCloud (cloud_downsampled_ptr);
  seg_.setInputNormals (cloud_normals_ptr);
  seg_.segment (table_inliers, table_coefficients);
  pcl::PointIndices::ConstPtr table_inliers_ptr = boost::make_shared<const pcl::PointIndices> (table_inliers);
  pcl::ModelCoefficients::ConstPtr table_coefficients_ptr = 
    boost::make_shared<const pcl::ModelCoefficients> (table_coefficients);

  if (table_coefficients.values.size () <=3)
  {
    ROS_INFO("Failed to detect table in scan");
    response.result = response.NO_TABLE;    
    return;
  }

  if ( table_inliers.indices.size() < (unsigned int)inlier_threshold_)
  {
    ROS_INFO("Plane detection has %d inliers, below min threshold of %d", (int)table_inliers.indices.size(),
	     inlier_threshold_);
    response.result = response.NO_TABLE;
    return;
  }

  ROS_INFO ("[TableObjectDetector::input_callback] Table found with %d inliers: [%f %f %f %f].", 
	    (int)table_inliers.indices.size (),
	    table_coefficients.values[0], table_coefficients.values[1], 
	    table_coefficients.values[2], table_coefficients.values[3]);
  ROS_INFO("Step 3 done");

  // Step 4 : Project the table inliers on the table
  pcl::PointCloud<Point> table_projected;
  proj_.setInputCloud (cloud_downsampled_ptr);
  proj_.setIndices (table_inliers_ptr);
  proj_.setModelCoefficients (table_coefficients_ptr);
  proj_.filter (table_projected);
  pcl::PointCloud<Point>::ConstPtr table_projected_ptr = 
    boost::make_shared<const pcl::PointCloud<Point> > (table_projected);
  ROS_INFO("Step 4 done");
  
  sensor_msgs::PointCloud table_points;
  tf::Transform table_plane_trans = getPlaneTransform (table_coefficients, up_direction_);
  //takes the points projected on the table and transforms them into the PointCloud message
  //while also transforming them into the table's coordinate system
  if (!getPlanePoints<Point> (table_projected, table_plane_trans, table_points))
  {
    response.result = response.OTHER_ERROR;
    return;
  }
  ROS_INFO("Table computed");

  response.table = getTable(cloud.header, table_plane_trans, table_points);
  response.result = response.SUCCESS;
  

  // ---[ Estimate the convex hull
  pcl::PointCloud<Point> table_hull;
  hull_.setInputCloud (table_projected_ptr);
  hull_.reconstruct (table_hull);
  pcl::PointCloud<Point>::ConstPtr table_hull_ptr = boost::make_shared<const pcl::PointCloud<Point> > (table_hull);

  // ---[ Get the objects on top of the table
  pcl::PointIndices cloud_object_indices;
  //prism_.setInputCloud (cloud_all_minus_table_ptr);
  prism_.setInputCloud (cloud_filtered_ptr);
  prism_.setInputPlanarHull (table_hull_ptr);
  ROS_INFO("Using table prism: %f to %f", table_z_filter_min_, table_z_filter_max_);
  prism_.setHeightLimits (table_z_filter_min_, table_z_filter_max_);  
  prism_.segment (cloud_object_indices);

  pcl::PointCloud<Point> cloud_objects;
  pcl::ExtractIndices<Point> extract_object_indices;
  extract_object_indices.setInputCloud (cloud_filtered_ptr);
  extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
  extract_object_indices.filter (cloud_objects);
  pcl::PointCloud<Point>::ConstPtr cloud_objects_ptr = boost::make_shared<const pcl::PointCloud<Point> > (cloud_objects);
  ROS_INFO (" Number of object point candidates: %d.", (int)cloud_objects.points.size ());

  if (cloud_objects.points.empty ()) 
  {
    ROS_INFO("No objects on table");
    return;
  }

  // ---[ Downsample the points
  pcl::PointCloud<Point> cloud_objects_downsampled;
  grid_objects_.setInputCloud (cloud_objects_ptr);
  grid_objects_.filter (cloud_objects_downsampled);
  pcl::PointCloud<Point>::ConstPtr cloud_objects_downsampled_ptr 
    = boost::make_shared <const pcl::PointCloud<Point> > (cloud_objects_downsampled);

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
  getClustersFromPCLCloud<Point> (cloud_objects_downsampled, clusters2, clusters);
  ROS_INFO("Clusters converted");
  response.clusters = clusters;  

  // get segmentation mask in image by backprojecting clusters
  std::vector<sensor_msgs::Image> masks;
  getMasksFromClusters(clusters, cam_info, masks);
  ROS_INFO("Masks generated.");
  response.masks = masks;
  

  // DEBUG: Publish point clouds to rviz
  // pcd_pub_.publish(clusters[0]);


  publishClusterMarkers(clusters, cloud.header);
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
