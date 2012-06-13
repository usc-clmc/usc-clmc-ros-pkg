/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         planning_pipeline.h

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#ifndef PLANNING_PIPELINE_H_
#define PLANNING_PIPELINE_H_

#include <vector>

#include <geometry_msgs/Pose.h>
#include <grasp_template/grasp_template.h>
#include <grasp_template/heightmap_sampling.h>
#include <grasp_template_planning/grasp_demo_library.h>
#include <grasp_template_planning/grasp_pool.h>
#include <grasp_template_planning/grasp_creator_interface.h>
#include <grasp_template_planning/grasp_planning_params.h>
#include <grasp_template_planning/template_matching.h>

namespace grasp_template_planning
{

class PlanningPipeline : public GraspCreatorInterface, private GraspPlanningParams
{
public:

  PlanningPipeline(const std::string& demo_path, const std::string& library_path,
      const std::string& failures_path);

  sensor_msgs::PointCloud target_object_;
  boost::shared_ptr<grasp_template::HeightmapSampling> templt_generator_;
  boost::shared_ptr<GraspDemoLibrary> library_;
  geometry_msgs::Pose table_frame_;

  bool getRelatedObject(const GraspAnalysis& analysis, sensor_msgs::PointCloud& container) const;

  bool initialize(const std::string& object_filename);
  bool initialize(const sensor_msgs::PointCloud& cluster, const geometry_msgs::Pose& table_pose);
  bool addFailure(const GraspAnalysis& lib_grasp, const GraspAnalysis& failure);
  bool addSuccess(const geometry_msgs::Pose& grasp_pose, const GraspAnalysis& succ_demo,
      const TemplateMatching& match_handl); //TODO: This has never been tried out!
  geometry_msgs::PoseStamped projectedGripperPose(const GraspAnalysis& analysis,
      const grasp_template::GraspTemplate& templt) const;
  virtual void createGrasp(const grasp_template::GraspTemplate& templt,
      const GraspAnalysis& lib_grasp, GraspAnalysis& result) const;
  void planGrasps(boost::shared_ptr<TemplateMatching>& pool) const;

private:

  bool offline_;
  std::string demonstrations_folder_, library_path_, failures_path_;
  std::string target_folder_, target_file_; //defined when planning offline

  boost::shared_ptr<const std::vector<grasp_template::GraspTemplate, Eigen::aligned_allocator<grasp_template::GraspTemplate> > >
  extractTemplatesParallel() const;
};

} //namespace
#endif /* PLANNING_PIPELINE_H_ */
