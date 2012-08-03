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

float getRGB( uint8_t r, uint8_t g, uint8_t b)
{
  union{ int intp; float floatp; } a;
  int res = (int(r) << 16) | (int(g) << 8) | int(b);
  a.intp=res;
  float rgb = *(&a.floatp);
  return rgb;
}

bool ObjectDetectionListener::mergeCloud( const sensor_msgs::PointCloud2 &ros_cloud_stereo,
  		 const sensor_msgs::PointCloud2 &ros_cloud_xtion,
  		 const sensor_msgs::CameraInfo &cam_info,
  		 tf::TransformListener &listener,
  		 sensor_msgs::PointCloud2 &ros_cloud_merged)
{
  // Create pinhole camera model from stereo camera
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(cam_info);

  // get the transform from Xtion to stereo
  tf::StampedTransform transform;
  bool can_transform = false;
  while(!can_transform){
    try {
      ros::Time acquisition_time = cam_info.header.stamp;
      ros::Duration timeout(1.0 / 30);
      ROS_INFO("Trying to look up transform from %s to %s\n",
	       ros_cloud_xtion.header.frame_id.c_str(),
	       ros_cloud_stereo.header.frame_id.c_str());
      can_transform = listener.waitForTransform(ros_cloud_xtion.header.frame_id,
						ros_cloud_stereo.header.frame_id,
						acquisition_time, timeout);
      ros::spinOnce();
    }
    catch (tf::TransformException& ex) {
      ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
      return false;
    }
  }

  if(can_transform) {
    //convert cloud to xtion frame and do annoying conversion from PointCloud2 to PointCloud
    sensor_msgs::PointCloud old_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud (ros_cloud_xtion, old_cloud);
    try
      {
	listener.transformPointCloud(ros_cloud_stereo.header.frame_id, old_cloud, old_cloud);
      }
    catch (tf::TransformException& ex)
      {
	ROS_ERROR("Failed to transform cloud from frame %s into frame %s", old_cloud.header.frame_id.c_str(),
		  ros_cloud_stereo.header.frame_id.c_str());
	return false;
      }

    ROS_INFO("Tranformed Xtion cloud into stereo frame\n");

    // copy stereo cloud into merged cloud
    ros_cloud_merged = ros_cloud_stereo;

    // some statistics
    int projected_points = 0;
    int nan_projected_points = 0;
    int merged_projected_points = 0;
    // project stereo point cloud into depth map of Xtion to merge data
    BOOST_FOREACH(geometry_msgs::Point32 pt,  old_cloud.points) {

      if(pt.x!=pt.x && pt.y!=pt.y && pt.z!=pt.z)
	continue;

      cv::Point3d pt_cv(pt.x, pt.y, pt.z);
      cv::Point2d uv;
      cam_model.project3dToPixel(pt_cv, uv);
      uv.x = std::floor(uv.x);
      uv.y = std::floor(uv.y);
      if( uv.x>0 && uv.x < cam_info.width &&  uv.y > 0 && uv.y < cam_info.height )  {
	projected_points++;
	cv::Point3f p;
	int i = uv.y*cam_info.width+uv.x;
	memcpy(&p.x,
	       &ros_cloud_stereo.data[i * ros_cloud_stereo.point_step +
				       ros_cloud_stereo.fields[0].offset], sizeof (float));

	memcpy (&p.y,
		&ros_cloud_stereo.data[i * ros_cloud_stereo.point_step +
					ros_cloud_stereo.fields[1].offset], sizeof (float));
	memcpy (&p.z,
		&ros_cloud_stereo.data[i * ros_cloud_stereo.point_step +
					ros_cloud_stereo.fields[2].offset], sizeof (float));
	float rgb;
	memcpy (&rgb,
		&ros_cloud_stereo.data[i * ros_cloud_stereo.point_step +
					ros_cloud_stereo.fields[3].offset], sizeof (float));


	// NaN test
	if(p.x!=p.x && p.y!=p.y && p.z!=p.z) {
	  nan_projected_points++;

	  float tmp = pt_cv.x;
	  // copy data from xtion directly into merged cloud
	  memcpy( &ros_cloud_merged.data[i * ros_cloud_merged.point_step +
					  ros_cloud_merged.fields[0].offset],
		  &tmp,
		  sizeof (float));

	  tmp = pt_cv.y;
	  memcpy( &ros_cloud_merged.data[i * ros_cloud_merged.point_step +
					  ros_cloud_merged.fields[1].offset],
		  &tmp,
		  sizeof (float));
	  tmp = pt_cv.z;
	  memcpy ( &ros_cloud_merged.data[i * ros_cloud_merged.point_step +
					   ros_cloud_merged.fields[2].offset],
		   &tmp, sizeof (float));

	  float green = getRGB(0.0f, 255.0f, 0.0f);
	  memcpy ( &ros_cloud_merged.data[i * ros_cloud_merged.point_step +
					   ros_cloud_merged.fields[3].offset],
		   &green, sizeof (float));
	} else {
	  merged_projected_points++;

	  // test whether difference is to large between stereo and xtion data
	  cv::Point3f diff( pt_cv.x - p.x, pt_cv.y - p.y, pt_cv.z - p.z);
	  float mag = std::sqrt(diff.x*diff.x + diff.y*diff.y + diff.z*diff.z);
	  cv::Point3f pt_avg;
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
	  memcpy( &ros_cloud_merged.data[i * ros_cloud_merged.point_step +
					  ros_cloud_merged.fields[0].offset],
		  &tmp,
		  sizeof (float));

	  tmp = pt_avg.y;
	  memcpy( &ros_cloud_merged.data[i * ros_cloud_merged.point_step +
					  ros_cloud_merged.fields[1].offset],
		  &tmp,
		  sizeof (float));
	  tmp = pt_avg.z;
	  memcpy ( &ros_cloud_merged.data[i * ros_cloud_merged.point_step +
					   ros_cloud_merged.fields[2].offset],
		   &tmp, sizeof (float));
	  memcpy ( &ros_cloud_merged.data[i * ros_cloud_merged.point_step +
					   ros_cloud_merged.fields[3].offset],
		   &rgb, sizeof (float));

	}
      }
    }
    ROS_INFO("Projected points: %d\n Nan points: %d\n, Merged points: %d\n",
	     projected_points,
	     nan_projected_points,
	     merged_projected_points);
  }


  return true;
}

void ObjectDetectionListener::connectToObjectDetector(ros::NodeHandle& n)
{
  cluster_client_ = n.serviceClient<tabletop_segmenter::TabletopSegmentation> ("/template_tabletop_segmentation");
  cluster_client_stereo_ = n.serviceClient<tabletop_segmenter::TabletopSegmentation> ("/template_tabletop_segmentation_stereo");
  stereo_exists_ = cluster_client_stereo_.exists();

  if(!stereo_exists_)
  {
	  ROS_WARN("stereo vision is not connected!");
  }
  else
  {
	  ROS_INFO("stereo vision connected");
  }
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

  if(stereo_exists_)
  {
	  ROS_INFO("grasp_template_planning::ObjectDetectionListener: connecting to tabletop_segmenter_stereo ...");
	  while (!cluster_client_stereo_.waitForExistence(ros::Duration(1.0)) && ros::ok())
	  {
		ros::Rate r(1);
		r.sleep();
	  }

	  if (!cluster_client_stereo_.call(tod_communication_stereo_))
	  {
		ROS_INFO("grasp_template_planning::ObjectDetectionListener: Could not get object cluster from tabletop_segmenter_stereo.");
		stereo_exists_ = false;
	  }
  }


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

  if(stereo_exists_)
  {
	if (tod_communication_stereo_.response.clusters.size() <= 0)
	{
		ROS_INFO("grasp_template_planning::ObjectDetectionListener: Tabletop_segmentation_stereo could not recognize any object.");
		stereo_exists_ =  false;
	}
	else
	{
		ROS_INFO("grasp_template_planning::ObjectDetectionListener: Got %d cluster(s) from tabletop_segmentation_stereo.",
		static_cast<int>(tod_communication_stereo_.response.clusters.size()) );
	}
  }

  {
    unsigned int ind_closest = getClosestCluster(tod_communication_);



    sensor_msgs::PointCloud2* merged_cloud_ptr = &tod_communication_.response.clusters[ind_closest];
	sensor_msgs::PointCloud2 merged_cloud;
	if(stereo_exists_)
	{
		unsigned int ind_closest_stereo = getClosestCluster(tod_communication_stereo_);
		merged_cloud_ptr = &merged_cloud;

		stereo_exists_ = mergeCloud( tod_communication_stereo_.response.clusters[ind_closest_stereo],
				tod_communication_.response.clusters[ind_closest],
				tod_communication_stereo_.response.cam_info,
				listener, merged_cloud);

		if(!stereo_exists_)
			merged_cloud_ptr = &tod_communication_.response.clusters[ind_closest];

	}
	pcl::PointCloud<pcl::PointXYZRGB> object_cluster_viewframe;
	pcl::PointCloud<pcl::PointXYZ> object_cluster_viewframe_nocol;

    pcl::fromROSMsg(*merged_cloud_ptr, object_cluster_viewframe_nocol);
    pcl::fromROSMsg(*merged_cloud_ptr, object_cluster_viewframe);

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


	plan_params.getTransform(transform, params.frameBase(), object_cluster_viewframe.header.frame_id);
    const geometry_msgs::Pose& b_pose = tod_communication_.response.table.pose.pose;
    Eigen::Vector3d a_t(transform.getOrigin().getX(), transform.getOrigin().getY(),
    		transform.getOrigin().getZ());
    Eigen::Vector3d b_t(b_pose.position.x, b_pose.position.y, b_pose.position.z);

    Eigen::Quaterniond a_rot(transform.getRotation().getW(), transform.getRotation().getX(),
    		transform.getRotation().getY(), transform.getRotation().getZ());
    Eigen::Quaterniond b_rot(b_pose.orientation.w, b_pose.orientation.x, b_pose.orientation.y,
    		b_pose.orientation.z);

    b_t = a_rot * b_t + a_t;
    b_rot = a_rot * b_rot;

//    Eigen::Vector3d ev(), rot, trans;
//    tf::poseMsgToTF(tod_communication_.response.table.pose.pose, old_pose);
//    transform * old_pose = new_pose;
//    tf::poseTFToMsg(new_pose, table_frame_.pose);
    table_frame_.header.stamp = ros::Time::now();
    table_frame_.header.frame_id = params.frameBase();
    table_frame_.pose.position.x = b_t.x();
    table_frame_.pose.position.y = b_t.y();
    table_frame_.pose.position.z = b_t.z();
    table_frame_.pose.orientation.w = b_rot.w();
    table_frame_.pose.orientation.x = b_rot.x();
    table_frame_.pose.orientation.y = b_rot.y();
    table_frame_.pose.orientation.z = b_rot.z();

//    ROS_INFO_STREAM("table_frame received: " << tod_communication_.response.table.pose.pose);
//    ROS_INFO_STREAM("table_frame after: " << table_frame_.pose);
  }




  return true;
}

} //namespace
