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

#include <Eigen/Eigen>
#include <pcl/io/io.h>
#include <pcl_ros/transforms.h>
#include <grasp_template_planning/object_detection_listener.h>
#include <grasp_template/grasp_template_params.h>
#include <grasp_template_planning/grasp_planning_params.h>
#include <tf/transform_broadcaster.h>

using namespace std;

namespace grasp_template_planning
{

const pcl::PointCloud<pcl::PointXYZ>& ObjectDetectionListener::getClusterPCL() const
{
  return object_cluster_;
}

void ObjectDetectionListener::getClusterPC2(sensor_msgs::PointCloud2& ret) const
{
  pcl::toROSMsg(object_cluster_, ret);
}


void ObjectDetectionListener::getClusterPC2Colored(sensor_msgs::PointCloud2& ret) const
{
  pcl::toROSMsg(object_cluster_colored_, ret);
}

const geometry_msgs::PoseStamped ObjectDetectionListener::getTableFrame() const
{
  return table_frame_;
}

void ObjectDetectionListener::connectToObjectDetector(ros::NodeHandle& n)
{
  cluster_client_ = n.serviceClient<tabletop_segmenter::TabletopSegmentation> ("/template_tabletop_segmentation");
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
	pcl::PointCloud<pcl::PointXYZRGB> object_cluster_viewframe;
	pcl::PointCloud<pcl::PointXYZ> object_cluster_viewframe_nocol;
    for (unsigned int i = 0; i < tod_communication_.response.clusters.size(); i++)
    {
    	pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
        pcl::fromROSMsg(tod_communication_.response.clusters[i], tmp_cloud);
      if (tmp_cloud.points[0].y < closest_y)
      {
        closest_y = tmp_cloud.points[0].y;
        ind_closest = i;
      }
    }
    pcl::fromROSMsg(tod_communication_.response.clusters[ind_closest], object_cluster_viewframe_nocol);
    pcl::fromROSMsg(tod_communication_.response.clusters[ind_closest], object_cluster_viewframe);

//    table_frame_ = tod_communication_.response.table.pose;

    tf::StampedTransform transform;
    grasp_template::GraspTemplateParams params;
    GraspPlanningParams plan_params;
    plan_params.getTransform(transform, params.frameBase(), object_cluster_viewframe.header.frame_id);

    Eigen::Vector3f trans_o = Eigen::Vector3f(transform.getOrigin().getX(),
    		transform.getOrigin().getY(), transform.getOrigin().getZ());
    Eigen::Quaternionf trans_r = Eigen::Quaternionf(transform.getRotation().getW(),
    		transform.getRotation().getX(), transform.getRotation().getY(),
    		transform.getRotation().getZ());
    pcl::transformPointCloud(object_cluster_viewframe_nocol, object_cluster_, trans_o, trans_r);
    object_cluster_.header.stamp = ros::Time::now();
    object_cluster_.header.frame_id = params.frameBase();

    pcl::transformPointCloud(object_cluster_viewframe, object_cluster_colored_, trans_o, trans_r);
    object_cluster_colored_.header.stamp = ros::Time::now();
    object_cluster_colored_.header.frame_id = params.frameBase();

    tf::Pose old_pose, new_pose;
    tf::poseMsgToTF(tod_communication_.response.table.pose.pose, old_pose);
    transform * old_pose = new_pose;
    tf::poseTFToMsg(new_pose, table_frame_.pose);
    table_frame_.header.stamp = ros::Time::now();
    table_frame_.header.frame_id = params.frameBase();
  }





  return true;
}

} //namespace
