/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         object_detection_listener.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <pcl/io/io.h>
#include <grasp_template_planning/object_detection_listener.h>

using namespace std;

namespace grasp_template_planning
{

const pcl::PointCloud<pcl::PointXYZ>& ObjectDetectionListener::getClusterPCL() const
{
  return object_cluster_;
}

const sensor_msgs::PointCloud2& ObjectDetectionListener::getClusterPC2() const
{
  sensor_msgs::PointCloud2 ret;
  pcl::toROSMsg(object_cluster_, ret);

  return ret;
}

const geometry_msgs::PoseStamped ObjectDetectionListener::getTableFrame() const
{
  return table_frame_;
}

void ObjectDetectionListener::connectToObjectDetector(ros::NodeHandle& n)
{
  cluster_client_ = n.serviceClient<tabletop_segmenter::TabletopSegmentation> ("/tabletop_segmentation");
}

bool ObjectDetectionListener::fetchClusterFromObjectDetector()
{
  ROS_DEBUG("grasp_template_planning::ObjectDetectionListener: connecting to tabletop_segmenter ...");
  while (!cluster_client_.waitForExistence(ros::Duration(1.0)) && ros::ok())
  {
    ros::Rate r(1);
    r.sleep();
  }

  if (!cluster_client_.call(tod_communication_))
  {
    ROS_DEBUG("grasp_template_planning::ObjectDetectionListener: Could not get object cluster from tabletop_segmenter.");
    return false;
  }

  if (tod_communication_.response.clusters.size() <= 0)
  {
    ROS_DEBUG("grasp_template_planning::ObjectDetectionListener: Tabletop_segmentation could not recognize any object.");
    return false;
  }
  else
  {
    ROS_DEBUG("grasp_template_planning::ObjectDetectionListener: Got %d cluster(s) from tabletop_segmentation.",
        static_cast<int>(tod_communication_.response.clusters.size()) );
  }

  {
    unsigned int ind_closest = 0;
    double closest_y = numeric_limits<double>::max();
    for (unsigned int i = 0; i < tod_communication_.response.clusters.size(); i++)
    {
    	pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
        pcl::fromROSMsg(tod_communication_.response.clusters[i], tmp_cloud);
      if (tmp_cloud.points[0].y < closest_y)
      {
        closest_y = tmp_cloud.points[0].y;
        ind_closest = i;
        object_cluster_ = tmp_cloud;
      }
    }
  }

  table_frame_ = tod_communication_.response.table.pose;

  return true;
}

} //namespace
