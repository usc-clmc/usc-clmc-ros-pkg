/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         grasp_planning_server.h

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#ifndef GRASP_PLANNING_SERVER_H_
#define GRASP_PLANNING_SERVER_H_

#include <map>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <grasp_template_planning/visualization.h>
#include <grasp_template_planning/planning_pipeline.h>
#include <object_manipulation_msgs/GraspPlanning.h>
#include <object_manipulation_msgs/GraspPlanningAction.h>
#include <object_manipulation_msgs/GraspPlanningActionGoal.h>
#include <grasp_template_planning/template_matching.h>
#include <grasp_template_planning/object_detection_listener.h>
#include <pr2_template_based_grasping/PlanningVisualization.h>
#include <pr2_template_based_grasping/PlanningFeedback.h>
#include <pr2_template_based_grasping/PlanningSummary.h>

namespace pr2_template_based_grasping
{

class GraspPlanningServer : private grasp_template_planning::GraspPlanningParams
{
public:
  GraspPlanningServer(ros::NodeHandle& nh, const std::string& demo_path,
      const std::string& lib_path, const std::string& failure_path, const std::string& success_path);

  bool plan(object_manipulation_msgs::GraspPlanning::Request &req,
            object_manipulation_msgs::GraspPlanning::Response &res);
  bool giveFeedback(PlanningFeedback::Request& req, PlanningFeedback::Response& res);
  bool visualize(PlanningVisualization::Request& req, PlanningVisualization::Response& res);
  bool getLog(PlanningSummary::Request& req, PlanningSummary::Response& res);

private:

  boost::mutex mutex_;
  ros::NodeHandle& nh_;

  grasp_template_planning::PlanningPipeline planning_pipe_;
  grasp_template_planning::Visualization visualizer_;
  boost::shared_ptr<grasp_template_planning::TemplateMatching> grasp_pool_;
  std::map<std::string, unsigned int> grasp_keys_;
  PlanningFeedback::Request grasp_feedback_;
  ros::Publisher attempt_pub_;
  geometry_msgs::Pose table_frame_;
  sensor_msgs::PointCloud2 target_cloud_;
  grasp_template_planning::ObjectDetectionListener object_detection_;
  ros::ServiceServer planning_service_;
  ros::ServiceServer vis_service_;
  ros::ServiceServer planning_feedback_service_;
  ros::ServiceServer planning_summary_service_;
  static const unsigned int PC_NUM_GRASP_OUTPUT = 100;

  void convertGrasps(const grasp_template_planning::TemplateMatching& pool,
                     std::vector<object_manipulation_msgs::Grasp>& goals);
  bool updateGraspLibrary();
  int getGraspResultIndex(unsigned int pool_index) const;
};

} //namespace
#endif /* GRASP_PLANNING_SERVER_H_ */
