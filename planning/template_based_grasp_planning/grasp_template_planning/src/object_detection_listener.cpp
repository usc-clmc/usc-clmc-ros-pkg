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
#include <opencv/cv.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/io/io.h>
#include <pcl_ros/transforms.h>
#include <grasp_template_planning/object_detection_listener.h>
#include <grasp_template/grasp_template_params.h>
#include <grasp_template_planning/grasp_planning_params.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <boost/foreach.hpp>

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
//  cluster_client_stereo_ = n.serviceClient<tabletop_segmenter::TabletopSegmentation> ("/template_tabletop_segmentation_stereo");
//  stereo_exists_ = cluster_client_stereo_.exists();
//
//  if(!stereo_exists_)
//  {
//	  ROS_WARN("stereo vision is not connected!");
//  }
//  else
//  {
//	  ROS_INFO("stereo vision connected");
//  }
}

int getClosestCluster(tabletop_segmenter::TabletopSegmentation& seg)
{
	  unsigned int ind_closest = 0;
	      double closest = numeric_limits<double>::max();
	      for (unsigned int i = 0; i < seg.response.clusters.size(); i++)
	      {
	      	pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
	          pcl::fromROSMsg(seg.response.clusters[i], tmp_cloud);
	        if (tmp_cloud.points[0].x * tmp_cloud.points[0].x + tmp_cloud.points[0].y * tmp_cloud.points[0].y
	      		  + tmp_cloud.points[0].z * tmp_cloud.points[0].z < closest)
	        {
	          closest = tmp_cloud.points[0].x * tmp_cloud.points[0].x + tmp_cloud.points[0].y * tmp_cloud.points[0].y
	        		  + tmp_cloud.points[0].z * tmp_cloud.points[0].z;
	          ind_closest = i;
	        }
	      }

	      return ind_closest;
}

bool ObjectDetectionListener::fetchClusterFromObjectDetector()
{
	tf::TransformListener listener;

  ROS_INFO("grasp_template_planning::ObjectDetectionListener: connecting to tabletop_segmenter ...");
  while (!cluster_client_.waitForExistence(ros::Duration(1.0)) && ros::ok())
  {
    ros::Rate r(1);
    r.sleep();
  }

  if (!cluster_client_.call(tod_communication_))
  {
    ROS_INFO("grasp_template_planning::ObjectDetectionListener: Could not get object cluster from tabletop_segmenter.");
    return false;
  }

//  if(stereo_exists_)
//  {
//	  ROS_INFO("grasp_template_planning::ObjectDetectionListener: connecting to tabletop_segmenter_stereo ...");
//	  while (!cluster_client_stereo_.waitForExistence(ros::Duration(1.0)) && ros::ok())
//	  {
//		ros::Rate r(1);
//		r.sleep();
//	  }
//
//	  if (!cluster_client_stereo_.call(tod_communication_stereo_))
//	  {
//		ROS_INFO("grasp_template_planning::ObjectDetectionListener: Could not get object cluster from tabletop_segmenter_stereo.");
//		stereo_exists_ = false;
//	  }
//  }


  if (tod_communication_.response.clusters.size() <= 0)
  {
    ROS_INFO("grasp_template_planning::ObjectDetectionListener: Tabletop_segmentation could not recognize any object.");
    return false;
  }
  else
  {
    ROS_INFO("grasp_template_planning::ObjectDetectionListener: Got %d cluster(s) from tabletop_segmentation.",
        static_cast<int>(tod_communication_.response.clusters.size()) );
  }

//  if(stereo_exists_)
//  {
//	if (tod_communication_stereo_.response.clusters.size() <= 0)
//	{
//		ROS_INFO("grasp_template_planning::ObjectDetectionListener: Tabletop_segmentation_stereo could not recognize any object.");
//		stereo_exists_ =  false;
//	}
//	else
//	{
//		ROS_INFO("grasp_template_planning::ObjectDetectionListener: Got %d cluster(s) from tabletop_segmentation_stereo.",
//		static_cast<int>(tod_communication_stereo_.response.clusters.size()) );
//	}
//  }

  {
    unsigned int ind_closest = getClosestCluster(tod_communication_);



//    sensor_msgs::PointCloud2* merged_cloud_ptr = &tod_communication_.response.clusters[ind_closest];
//	sensor_msgs::PointCloud2 merged_cloud;
//	if(stereo_exists_)
//	{
//		unsigned int ind_closest_stereo = getClosestCluster(tod_communication_stereo_);
//		merged_cloud_ptr = &merged_cloud;
//
//		stereo_exists_ = mergeCloud( tod_communication_stereo_.response.clusters[ind_closest_stereo],
//				tod_communication_.response.clusters[ind_closest],
//				tod_communication_stereo_.response.cam_info,
//				listener, merged_cloud);
//
//		if(!stereo_exists_)
//			merged_cloud_ptr = &tod_communication_.response.clusters[ind_closest];
//
//	}
	pcl::PointCloud<pcl::PointXYZRGB> object_cluster_viewframe;
	pcl::PointCloud<pcl::PointXYZ> object_cluster_viewframe_nocol;

    pcl::fromROSMsg(tod_communication_.response.clusters[ind_closest], object_cluster_viewframe_nocol);
    pcl::fromROSMsg(tod_communication_.response.clusters[ind_closest], object_cluster_viewframe);

//    ROS_INFO_STREAM("points in cloud: " << object_cluster_viewframe_nocol.size() << " and " <<
//    		object_cluster_viewframe.size());
//    ROS_INFO_STREAM("points: " << object_cluster_viewframe.at(0).x << ", "<< object_cluster_viewframe.at(0).y
//    		<< ", "<< object_cluster_viewframe.at(0).z);
//    table_frame_ = tod_communication_.response.table.pose;

    tf::StampedTransform base_to_clusterworld;
    grasp_template::GraspTemplateParams params;
    GraspPlanningParams plan_params;
//    ROS_INFO_STREAM("getting transform from " << object_cluster_viewframe.header.frame_id << " to " << params.frameBase());
    plan_params.getTransform(base_to_clusterworld, params.frameBase(), object_cluster_viewframe.header.frame_id);


//    ROS_INFO_STREAM("transf: " << base_to_clusterworld.getOrigin().x() << ", " << base_to_clusterworld.getOrigin().y() << ", "
//    		<< base_to_clusterworld.getOrigin().z() << ", "<< base_to_clusterworld.getRotation().w() << ", "
//    		<< base_to_clusterworld.getRotation().x() << ", "<< base_to_clusterworld.getRotation().y() << ", "
//    		<< base_to_clusterworld.getRotation().z() << ", ");

    Eigen::Vector3f trans_o = Eigen::Vector3f(base_to_clusterworld.getOrigin().getX(),
    		base_to_clusterworld.getOrigin().getY(), base_to_clusterworld.getOrigin().getZ());
    Eigen::Quaternionf trans_r = Eigen::Quaternionf(base_to_clusterworld.getRotation().getW(),
    		base_to_clusterworld.getRotation().getX(), base_to_clusterworld.getRotation().getY(),
    		base_to_clusterworld.getRotation().getZ());

    pcl::transformPointCloud(object_cluster_viewframe_nocol, object_cluster_, trans_o, trans_r);
    object_cluster_.header.stamp = ros::Time::now();
    object_cluster_.header.frame_id = params.frameBase();

    pcl::transformPointCloud(object_cluster_viewframe, object_cluster_colored_, trans_o, trans_r);
    object_cluster_colored_.header.stamp = ros::Time::now();
    object_cluster_colored_.header.frame_id = params.frameBase();


//    ROS_INFO_STREAM("points in cloud: " << object_cluster_.size() << " and " <<
//    		object_cluster_colored_.size());
//    ROS_INFO_STREAM("points: " << object_cluster_colored_.at(0).x << ", "<< object_cluster_colored_.at(0).y << ", "<< object_cluster_colored_.at(0).z);

    tf::StampedTransform base_to_tableworld;
//    ROS_INFO_STREAM("table-world frame: " << tod_communication_.response.table.pose.header.frame_id);
	plan_params.getTransform(base_to_tableworld, tod_communication_.response.table.pose.header.frame_id, params.frameBase());
	tf::Transform table_to_tableworld;
    tf::poseMsgToTF(tod_communication_.response.table.pose.pose, table_to_tableworld);
	tf::Transform table_to_base = base_to_tableworld.inverse()*table_to_tableworld;
//    tf::poseMsgToTF(b_pose, transform);
//    Eigen::Vector3d a_t(transform.getOrigin().getX(), transform.getOrigin().getY(),
//    		transform.getOrigin().getZ());
//    Eigen::Vector3d b_t(b_pose.position.x, b_pose.position.y, b_pose.position.z);
//
//    Eigen::Quaterniond a_rot(transform.getRotation().getW(), transform.getRotation().getX(),
//    		transform.getRotation().getY(), transform.getRotation().getZ());
//    Eigen::Quaterniond b_rot(b_pose.orientation.w, b_pose.orientation.x, b_pose.orientation.y,
//    		b_pose.orientation.z);
//
//    b_t = a_rot * b_t + a_t;
//    b_rot = a_rot * b_rot;

//    Eigen::Vector3d ev(), rot, trans;
//    tf::poseMsgToTF(tod_communication_.response.table.pose.pose, old_pose);
//    transform * old_pose = new_pose;
//    tf::poseTFToMsg(new_pose, table_frame_.pose);
    table_frame_.header.stamp = ros::Time::now();
    table_frame_.header.frame_id = params.frameBase();
    table_frame_.pose.position.x = table_to_base.getOrigin().x();
    table_frame_.pose.position.y = table_to_base.getOrigin().y();
    table_frame_.pose.position.z = table_to_base.getOrigin().z();
    table_frame_.pose.orientation.w = table_to_base.getRotation().w();
    table_frame_.pose.orientation.x = table_to_base.getRotation().x();
    table_frame_.pose.orientation.y = table_to_base.getRotation().y();
    table_frame_.pose.orientation.z = table_to_base.getRotation().z();

//    ROS_INFO_STREAM("table_frame received: " << tod_communication_.response.table.pose.pose);
//    ROS_INFO_STREAM("table_frame after: " << table_frame_.pose);
  }




  return true;
}

} //namespace
