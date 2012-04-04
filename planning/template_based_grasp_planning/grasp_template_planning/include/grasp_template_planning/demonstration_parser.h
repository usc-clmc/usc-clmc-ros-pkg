/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         demonstration_parser.h

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#ifndef DEMONSTRATION_PARSER_H
#define DEMONSTRATION_PARSER_H

#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <grasp_template_planning/grasp_planning_params.h>
#include <grasp_template/grasp_template.h>
#include <grasp_template_planning/GraspAnalysis.h>
#include <grasp_template_planning/grasp_demo_library.h>
#include <grasp_template_planning/SimpleLabel.h>

namespace grasp_template_planning
{

class DemonstrationParser : private GraspPlanningParams
{
public:

  DemonstrationParser(const GraspDemoLibrary& lib);

  /* parses *.bag file of taught grasp to GraspAnalysis messages */
  bool analyzeGrasp(GraspAnalysis& analysis);
  std::string getDemoId() const;
  static void templtToAnalysis(const grasp_template::GraspTemplate& templt,
      const std::string& frame_id, GraspAnalysis& analysis);

private:

  const GraspDemoLibrary* library_handler_;
  std::string demo_id_;
  std::string object_frame_id_;

  void computeRefPoint(Eigen::Vector3d& result, const geometry_msgs::PoseStamped& gripper_pose) const;
  bool createAnalysisMsg(const grasp_template::GraspTemplate& templt,
      const geometry_msgs::PoseStamped& gripper_pose, double gripper_joint_state,
      const geometry_msgs::PoseStamped& viewpoint, GraspAnalysis& analysis,
      const DoubleVector& fingerpositions) const;
};

} //namespace
#endif /* DEMONSTRATION_PARSER_H */
