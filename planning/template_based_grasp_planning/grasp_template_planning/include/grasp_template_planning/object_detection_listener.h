/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         object_detection_listener.h

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#ifndef OBJECT_DETECTION_LISTENER_H_
#define OBJECT_DETECTION_LISTENER_H_

#include <limits>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/Pose.h>
#include <tabletop_segmenter/TabletopSegmentation.h>

namespace grasp_template_planning
{

class ObjectDetectionListener
{
public:

  ObjectDetectionListener(){};

  const pcl::PointCloud<pcl::PointXYZ>& getClusterPCL() const;
  void getClusterPC2(sensor_msgs::PointCloud2& ret) const;
  void getClusterPC2Colored(sensor_msgs::PointCloud2& ret) const;
  const geometry_msgs::PoseStamped getTableFrame() const;

  void connectToObjectDetector(ros::NodeHandle& n);
  bool fetchClusterFromObjectDetector();

private:

  ros::ServiceClient cluster_client_;
  bool stereo_exists_;
  ros::ServiceClient cluster_client_stereo_;
  geometry_msgs::PoseStamped table_frame_;
  tabletop_segmenter::TabletopSegmentation tod_communication_, tod_communication_stereo_;
  pcl::PointCloud<pcl::PointXYZ> object_cluster_; //closest object
  pcl::PointCloud<pcl::PointXYZRGB> object_cluster_colored_; //closest object

  bool mergeCloud( const sensor_msgs::PointCloud2 &ros_cloud_stereo,
	  		 const sensor_msgs::PointCloud2 &ros_cloud_xtion,
	  		 const sensor_msgs::CameraInfo &cam_info,
	  		 tf::TransformListener &listener,
	  		 sensor_msgs::PointCloud2 &ros_cloud_merged);
};

}//  namespace
#endif /* OBJECT_DETECTION_LISTENER_H_ */
