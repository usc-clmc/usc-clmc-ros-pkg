/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         visualization.h

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <map>
#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <sensor_msgs/PointCloud.h>
#include <grasp_template_planning/planning_pipeline.h>
#include <geometry_msgs/PoseStamped.h>
#include <grasp_template_planning/grasp_planning_params.h>

namespace grasp_template_planning
{

class Visualization : protected GraspPlanningParams
{
public:
  Visualization(bool slim_publishing = false, double hz = 0.4);
  ~Visualization();

  void initialize(ros::NodeHandle& n);
  void resetData(const PlanningPipeline& dp, const GraspAnalysis& ref,
      const GraspAnalysis& trgt_templt);
  void startPublishing();
  void stopPublishing();
  void publishOnce();

private:
  boost::mutex mutex_;
  boost::shared_ptr<boost::thread> publish_thread_;
  volatile bool stop_requested_;
  std::map<std::string, ros::Publisher> publisher_;
  ros::Rate rate_;
  bool slim_publishing_;

  sensor_msgs::PointCloud matching_object_;
  sensor_msgs::PointCloud target_object_;
  geometry_msgs::PoseStamped matching_gripper_;
  geometry_msgs::PoseStamped target_gripper_;
  geometry_msgs::PoseStamped target_templt_pose_;
  geometry_msgs::PoseStamped viewpoint_pose_;
  geometry_msgs::PoseStamped table_pose_;
  std::vector<visualization_msgs::Marker> matching_gripper_mesh_;
  std::vector<visualization_msgs::Marker> target_gripper_mesh_;
  std::vector<visualization_msgs::Marker> matching_templt_;
  std::vector<visualization_msgs::Marker> target_templt_;
  std::vector<visualization_msgs::Marker> matching_hm_;
  std::vector<visualization_msgs::Marker> target_hm_;
  visualization_msgs::Marker target_normals_;
  sensor_msgs::PointCloud search_points_;
  sensor_msgs::PointCloud target_hull_;
  std::vector<visualization_msgs::Marker> target_hull_mesh_;
  std::vector<visualization_msgs::Marker> gripper_table_intersect_;

  virtual std::vector<visualization_msgs::Marker> visualizeGripper(
      const std::string& ns, const std::string& frame_id,
      const geometry_msgs::Pose& pose, double gripper_state = 0) const;
  visualization_msgs::Marker visualizeGripperPart(const std::string& resource,
      const std::string& ns, const std::string& frame_id, int id) const;

  void transformMarkers(std::vector<visualization_msgs::Marker>& container,
      const Eigen::Vector3d& trans, const Eigen::Quaterniond& rot);
  void transformPose(geometry_msgs::Pose& pose, const Eigen::Vector3d& trans,
      const Eigen::Quaterniond& rot);
  void transformPointCloud(sensor_msgs::PointCloud& pc, const Eigen::Vector3d& trans,
      const Eigen::Quaterniond& rot);
  void adaptForTemplateComparison(std::vector<visualization_msgs::Marker>& m1,
      std::vector<visualization_msgs::Marker>& m2, const Eigen::Vector3d& trans,
      const Eigen::Quaterniond& rot);
  void doPublishing();
  void computeHullMesh(const pcl::PointCloud<pcl::PointXYZ>& points,
      const std::vector<pcl::Vertices>& vertices);

protected:
  visualization_msgs::Marker visualizeGripperBox(const std::string& ns,
      const std::string& frame_id, int id) const;

};
} //namespace
#endif /* VISUALIZATION_H_ */
