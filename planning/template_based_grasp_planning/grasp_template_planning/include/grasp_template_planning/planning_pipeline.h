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

#include <rosbag/bag.h>
#include <geometry_msgs/Pose.h>
#include <grasp_template/grasp_template.h>
#include <grasp_template/heightmap_sampling.h>
#include <grasp_template_planning/grasp_demo_library.h>
#include <grasp_template_planning/grasp_pool.h>
#include <grasp_template_planning/grasp_creator_interface.h>
#include <grasp_template_planning/grasp_planning_params.h>
#include <grasp_template_planning/template_matching.h>
#include <grasp_template_planning/GraspLog.h>

namespace grasp_template_planning
{

class PlanningPipeline : public GraspCreatorInterface, public GraspPlanningParams
{
public:

  PlanningPipeline(const std::string& demo_path, const std::string& library_path,
      const std::string& failures_path, const std::string& successes_path,
      const std::string& log_data_path);
  virtual ~PlanningPipeline();

  sensor_msgs::PointCloud2 target_object_;
  boost::shared_ptr<grasp_template::HeightmapSampling> templt_generator_;
  boost::shared_ptr<GraspDemoLibrary> library_;
  geometry_msgs::Pose table_frame_;
  GraspLog log_;
  std::string demonstrations_folder_, library_path_, failures_path_, successes_path_, log_data_path_;
  rosbag::Bag log_bag_;

  bool getRelatedObject(const GraspAnalysis& analysis, sensor_msgs::PointCloud2& container) const;

  bool initialize(const std::string& object_filename, bool log_data = true);
  bool initialize(const sensor_msgs::PointCloud2& cluster, const geometry_msgs::Pose& table_pose, bool log_data = true);
  virtual bool addFailure(const GraspAnalysis& lib_grasp, const GraspAnalysis& failure);
  virtual bool addSuccess(const GraspAnalysis& lib_grasp, const GraspAnalysis& success);
//  bool addSuccess(const geometry_msgs::Pose& grasp_pose, const GraspAnalysis& succ_demo,
//      const TemplateMatching& match_handl); //TODO: This has never been tried out!
  geometry_msgs::PoseStamped projectedGripperPose(const GraspAnalysis& analysis,
      const grasp_template::GraspTemplate& templt) const;
  virtual void createGrasp(const grasp_template::GraspTemplate& templt,
      const GraspAnalysis& lib_grasp, GraspAnalysis& result) const;
  virtual void planGrasps(boost::shared_ptr<TemplateMatching>& pool) const;
  virtual bool logPlannedGrasps(const TemplateMatching& pool, unsigned int max_num_grasps);
  virtual bool logGraspResult(const GraspAnalysis& res_ana, const TemplateMatching& pool, int rank);
  virtual bool writeLogToBag();

private:

  bool offline_, log_data_;
  std::string target_folder_, target_file_; //defined when planning offline

  boost::shared_ptr<const std::vector<grasp_template::GraspTemplate, Eigen::aligned_allocator<grasp_template::GraspTemplate> > >
  extractTemplatesParallel() const;
};

} //namespace
#endif /* PLANNING_PIPELINE_H_ */
