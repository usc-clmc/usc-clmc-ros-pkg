/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         demo_writer.h

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#ifndef DEMO_WRITER_H_
#define DEMO_WRITER_H_

#include <string>

#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <grasp_template_planning/SimpleLabel.h>
#include <grasp_template_planning/grasp_planning_params.h>

namespace grasp_template_planning
{

class DemoWriter : GraspPlanningParams
{
public:
  DemoWriter(const std::string& filename);
  ~DemoWriter();

  bool writeDemonstration(const sensor_msgs::PointCloud2& object, const geometry_msgs::PoseStamped& gripper_pose,
      const geometry_msgs::PoseStamped& table_pose, const geometry_msgs::PoseStamped& viewpoint,
      const std::vector<double>& fingerpositions);
//  bool writeDemonstration(const sensor_msgs::PointCloud2& object, const geometry_msgs::PoseStamped& gripper_pose,
//      const geometry_msgs::PoseStamped& table_pose, const geometry_msgs::PoseStamped& viewpoint);
  void close();

private:

  rosbag::Bag bag_;
};

} //namespace
#endif /* DEMO_WRITER_H_ */
