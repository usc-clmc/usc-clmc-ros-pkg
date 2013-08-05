/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         data_loader.cpp

 \author       Daniel Kappler
 \date         July 31, 2013

 *********************************************************************/

#include <deep_learning/log_loader.h>

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <usc_utilities/file_io.h>
#include <visualization_msgs/Marker.h>

#include <grasp_template/Heightmap.h>
#include <grasp_template/dismatch_measure.h>
#include <grasp_template/heightmap_sampling.h>
#include <grasp_template_planning/GraspLog.h>

Data_loader::Data_loader(const std::string &log_topic)
{
  _log_topic = log_topic;
}

bool Data_loader::Load_trial_log(
    const std::string &path_bagfile,
    std::vector<grasp_template::GraspTemplate, Eigen::aligned_allocator<grasp_template::GraspTemplate> > &result_template,
    std::vector<std::string> &result_uuid, std::vector<float> &result_success, sensor_msgs::PointCloud2 &result_object_cloud,
    geometry_msgs::Pose &result_view_point)
{

  // reset all result vectors
  result_template.clear();
  result_uuid.clear();
  result_success.clear();

  // read execution log from the bagfile
  std::vector < grasp_template_planning::GraspLog > grasp_trial_log;
  usc_utilities::FileIO < grasp_template_planning::GraspLog
  > ::readFromBagFile(grasp_trial_log, _log_topic, path_bagfile);

  // grasp_trial_log is sorted because in the bag they might not
  // grasp_trial_log is split up such that the messages are smaller

  // this is disgusting, but the content of GraspLog is spread over 3 messages
  // in the following we sort the messages
  std::vector<const grasp_template_planning::GraspLog*> grasp_trial_log_sorted;
  grasp_trial_log_sorted.resize(3);
  for (unsigned int i = 0; i < grasp_trial_log.size(); i++)
  {
    const int seq_nr = grasp_trial_log[i].seq;
    grasp_trial_log_sorted[seq_nr] = &(grasp_trial_log[i]);

  }

  // applied grasp is the one executed
  grasp_template_planning::GraspAnalysis grasp_applied = grasp_trial_log_sorted[1]->applied_grasp;
  if ((grasp_applied.grasp_success = 0.0) || (grasp_applied.grasp_success == 1.0))
  {

    grasp_template::GraspTemplate g_temp = grasp_template::GraspTemplate(grasp_applied.grasp_template,
        grasp_applied.template_pose.pose);
    grasp_template::DismatchMeasure d_measure(grasp_applied.grasp_template, grasp_applied.template_pose.pose,
        grasp_applied.gripper_pose.pose);
    d_measure.applyDcMask(g_temp);
    result_template.push_back(g_temp);
    result_uuid.push_back(grasp_applied.uuid);
    result_success.push_back(grasp_applied.grasp_success);
  }

  // here we pull out what is required for heightmap computation
  result_object_cloud = grasp_trial_log_sorted[0]->target_object;

  grasp_applied = grasp_trial_log_sorted[1]->applied_grasp;

  geometry_msgs::Pose table_frame = grasp_trial_log_sorted[1]->table_frame;
  geometry_msgs::Point vp = grasp_applied.viewpoint_transform.pose.position;
  geometry_msgs::Quaternion vo = grasp_applied.viewpoint_transform.pose.orientation;


  // matched grasps are the ones in the library
  grasp_template_planning::GraspAnalysis grasp_matched = grasp_trial_log_sorted[1]->matched_grasp;

  grasp_template::Heightmp grasp_heightmap = grasp_matched.grasp_template;
  geometry_msgs::Pose template_pose = grasp_matched.template_pose.pose;
  geometry_msgs::Pose gripper_pose = grasp_matched.gripper_pose.pose;

  // convert some data structures
  pcl::PointCloud < pcl::PointXYZ > pcl_cloud;
  pcl::fromROSMsg(object_cloud, pcl_cloud);

  Eigen::Vector3d viewpoint_pos(vp.x, vp.y, vp.z);
  Eigen::Quaterniond viewpoint_rot(vo.w, vo.x, vo.y, vo.z);

  result_view_point.position.x = vp.x;
  result_view_point.position.y = vp.y;
  result_view_point.position.z = vp.z;

  result_view_point.orientation.x = vo.x;
  result_view_point.orientation.y = vo.y;
  result_view_point.orientation.z = vo.z;
  result_view_point.orientation.w = vo.w;

  // this guy generates heightmaps from point clouds
  grasp_template::HeightmapSampling heightmap_computation = grasp_template::HeightmapSampling(viewpoint_pos,
      viewpoint_rot);
  heightmap_computation.initialize(pcl_cloud, table_frame);

  // HsIterator is implemented in the same header where I implemented HeightmapSampling
  for (grasp_template::HsIterator it = heightmap_computation.getIterator(); !it.passedLast() && ros::ok(); it.inc())
  {
    grasp_template::GraspTemplate t;

    // check the overloaded functions of generateTemplateOnHull and generateTemplate
    // they provide more options to extract templates
    heightmap_computation.generateTemplateOnHull(t, it);

    grasp_template::DismatchMeasure d_measure(grasp_matched.grasp_template, grasp_matched.template_pose.pose,
        grasp_matched.gripper_pose.pose);
    d_measure.applyDcMask(t);

    result_template.push_back(t);
    result_uuid.push_back("__NONE__");
    result_success.push_back(-1.0);
    ROS_ERROR("finish for faster loading");
    break;
  }
  return true;
}

bool Data_loader::Load_templates(
    const std::string &path_bagfile,
    std::vector<grasp_template::GraspTemplate, Eigen::aligned_allocator<grasp_template::GraspTemplate> > &result_template,
    std::vector<std::string> &result_uuid)
{

  return true;
}
