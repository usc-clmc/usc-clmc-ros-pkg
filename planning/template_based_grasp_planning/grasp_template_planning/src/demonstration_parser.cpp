/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         demonstration_parser.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <Eigen/Eigen>
#include <grasp_template/heightmap_sampling.h>
#include <grasp_template_planning/grasp_demo_library.h>
#include <grasp_template_planning/SimpleLabel.h>
#include <grasp_template_planning/demonstration_parser.h>

using namespace std;
using namespace Eigen;
using namespace grasp_template;

namespace grasp_template_planning
{

DemonstrationParser::DemonstrationParser(const GraspDemoLibrary& lib)
{
  library_handler_ = &lib;
}

bool DemonstrationParser::analyzeGrasp(GraspAnalysis& analysis)
{
  if (!library_handler_->isDemonstrationLoaded())
  {
    ROS_DEBUG("grasp_template_planning::DemonstrationParser: "
        "No grasp demonstration was loaded, yet.");
    return false;
  }

  GraspDemoObjectMap::const_iterator oc_it = library_handler_->getObjects()->begin();
  GraspDemoPoseMap::const_iterator gp_it = library_handler_->getGripperPoses()->begin();
  GraspDemoPoseMap::const_iterator tp_it = library_handler_->getTablePoses()->begin();
  string demo_id = getDemoId();

  if (oc_it == library_handler_->getObjects()->end() || gp_it == library_handler_-> getGripperPoses()->end()
  /*|| js_it == library_handler_->getJointStates()->end()*/|| tp_it == library_handler_->getTablePoses()->end())
    return false;

  geometry_msgs::PoseStamped viewpoint;
  viewpoint = library_handler_->getViewpointTransforms()->begin()->second;

  /* parse */
  demo_id_ = demo_id;
  object_frame_id_ = oc_it->second.header.frame_id;

  Vector3d vp_trans;
  vp_trans.x() = viewpoint.pose.position.x;
  vp_trans.y() = viewpoint.pose.position.y;
  vp_trans.z() = viewpoint.pose.position.z;
  Quaterniond vp_rot;
  vp_rot.w() = viewpoint.pose.orientation.w;
  vp_rot.x() = viewpoint.pose.orientation.x;
  vp_rot.y() = viewpoint.pose.orientation.y;
  vp_rot.z() = viewpoint.pose.orientation.z;

  HeightmapSampling t_gen(vp_trans, vp_rot);
  pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
  pcl::fromROSMsg(oc_it->second, tmp_cloud);
  t_gen.initialize(tmp_cloud, tp_it->second.pose);
  Vector3d ref_point;
  computeRefPoint(ref_point, gp_it->second);
  GraspTemplate templt;
  t_gen.generateTemplateOnHull(templt, ref_point);

  DoubleVector fingerpositions;
  string gripper_arch;
  ros::param::get("~hand_architecture", gripper_arch);
  if (gripper_arch.compare("armrobot") == 0)
  {
    fingerpositions = library_handler_->getFingerpositions()->begin()->second;
  }
  GraspPlanningParams::setIdAndTime(analysis);
  if (createAnalysisMsg(templt, gp_it->second, 0.08/*getGripperStateFromJointState(js_it->second)*/, viewpoint,
                        analysis, fingerpositions))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void DemonstrationParser::computeRefPoint(Vector3d& result, const geometry_msgs::PoseStamped& gripper_pose) const
{
  Vector3d trans(gripper_pose.pose.position.x, gripper_pose.pose.position.y, gripper_pose.pose.position.z);
  Quaterniond rot(gripper_pose.pose.orientation.w, gripper_pose.pose.orientation.x,
      gripper_pose.pose.orientation.y, gripper_pose.pose.orientation.z);

  Vector3d delta;
  getTemplateExtractionPoint(delta);
  result = rot * delta + trans;
}

bool DemonstrationParser::createAnalysisMsg(const GraspTemplate& templt,
    const geometry_msgs::PoseStamped& gripper_pose, double gripper_joint_state,
    const geometry_msgs::PoseStamped& viewpoint, GraspAnalysis& analysis,
    const DoubleVector& fingerpositions) const
{
  /* set id */
  analysis.demo_id = demo_id_;
  analysis.demo_filename = library_handler_->getDemoFilename();

  /* set viewpoint */
  analysis.viewpoint_transform = viewpoint;

  /* set template */
  templt.heightmap_.toHeightmapMsg(analysis.grasp_template);

  geometry_msgs::PoseStamped templt_transform;
  analysis.template_pose.header.frame_id = object_frame_id_;
  analysis.template_pose.header.stamp = ros::Time::now();
  templt.getPose(analysis.template_pose.pose);

  /* set grasp success */
  analysis.grasp_success = 1.0;

  /* set gripper transform and state */
  analysis.gripper_pose = gripper_pose;
  analysis.gripper_joint_state = gripper_joint_state;
  analysis.fingerpositions = fingerpositions;

  return true;
}

string DemonstrationParser::getDemoId() const
{
  /* will crash, if event does not exist */
  string demo_id_str = library_handler_->getEvents()->upper_bound(labelGraspDemoId())->first;
  demo_id_str.erase(0, labelGraspDemoId().length());
  return demo_id_str;
}

void DemonstrationParser::templtToAnalysis(const GraspTemplate& templt, const string& frame_id, GraspAnalysis& analysis)
{
  templt.heightmap_.toHeightmapMsg(analysis.grasp_template);

  analysis.template_pose.header.stamp = ros::Time::now();
  analysis.template_pose.header.frame_id = frame_id;

  analysis.template_pose.pose.position.x = templt.object_to_template_translation_.x();
  analysis.template_pose.pose.position.y = templt.object_to_template_translation_.y();
  analysis.template_pose.pose.position.z = templt.object_to_template_translation_.z();

  analysis.template_pose.pose.orientation.w = templt.object_to_template_rotation_.w();
  analysis.template_pose.pose.orientation.x = templt.object_to_template_rotation_.x();
  analysis.template_pose.pose.orientation.y = templt.object_to_template_rotation_.y();
  analysis.template_pose.pose.orientation.z = templt.object_to_template_rotation_.z();
}

} //namespace
