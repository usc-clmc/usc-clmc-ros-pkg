/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         demo_writer.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <grasp_template_planning/demo_writer.h>
#include <grasp_template_planning/DoubleVector.h>

namespace grasp_template_planning
{

DemoWriter::DemoWriter(const std::string& filename)
{
  ROS_DEBUG("grasp_template_planning::DemoWriter: Opening new bag to store demonstration: %s",
      filename.c_str());
  try
  {
    bag_.open(filename, rosbag::bagmode::Write);

  }
  catch (rosbag::BagIOException ex)
  {
    ROS_DEBUG("grasp_template_planning::DemoWriter: Problem when opening demo file >%s< : %s.",
        filename.c_str(), ex.what());
  }
}

bool DemoWriter::writeDemonstration(const sensor_msgs::PointCloud2& object,
    const geometry_msgs::PoseStamped& gripper_pose,
    const geometry_msgs::PoseStamped& table_pose,
    const geometry_msgs::PoseStamped& viewpoint,
    const std::vector<double>& fingerpositions)
{
  SimpleLabel id_msg;
  id_msg.event_label = labelGraspDemoId();
  id_msg.event_label.append(createId());
  id_msg.stamp = ros::Time::now();

  DoubleVector fs;
  fs.vals = fingerpositions;

  ROS_DEBUG("grasp_template_planning::DemoWriter: Writing demonstration to bag: %s",
      bag_.getFileName().c_str());
  try
  {
    bag_.write(topicGraspDemoEvents(), ros::Time::now(), id_msg);
    bag_.write(topicGraspDemoObjectCluster(), ros::Time::now(), object);
    bag_.write(topicGraspDemoGripperPose(), ros::Time::now(), gripper_pose);
    bag_.write(topicGraspDemoTable(), ros::Time::now(), table_pose);
    bag_.write(topicViewpointTransform(), ros::Time::now(), viewpoint);
    bag_.write(topicFingerpositions(), ros::Time::now(), fs);
  }
  catch (rosbag::BagIOException ex)
  {
    ROS_DEBUG("grasp_template_planning::DemoWriter: Problem when writing demonstration to file >%s< : %s.",
        bag_.getFileName().c_str(), ex.what());

    return false;
  }
  return true;
}

DemoWriter::~DemoWriter()
{
  close();
}

//bool DemoWriter::writeDemonstration(const sensor_msgs::PointCloud2& object,
//                                    const geometry_msgs::PoseStamped& gripper_pose,
//                                    const geometry_msgs::PoseStamped& table_pose,
//                                    const geometry_msgs::PoseStamped& viewpoint)
//{
//  std::vector<double> fingerpositions;
//  fingerpositions.resize(4);
//  return writeDemonstration(object, gripper_pose, table_pose, viewpoint, fingerpositions);
//}

void DemoWriter::close()
{
  bag_.close();
}

} //namespace
