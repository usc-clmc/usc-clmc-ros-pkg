/*
 * stomp_cost_function_input.cpp
 *
 *  Created on: May 24, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/stomp_cost_function_input.h>

namespace stomp_ros_interface
{

StompCostFunctionInput::StompCostFunctionInput(boost::shared_ptr<StompCollisionSpace const> collision_space,
                       boost::shared_ptr<StompRobotModel const> robot_model,
                       const StompRobotModel::StompPlanningGroup* planning_group)
{
  collision_space_ = collision_space;
  robot_model_ = robot_model;
  planning_group_ = planning_group;

  // allocate memory
  int nj = robot_model_->getNumKDLJoints();
  int nc = planning_group_->collision_points_.size();
  int nl = planning_group_->fk_solver_->getSegmentNames().size();
  joint_angles_ = KDL::JntArray(planning_group_->num_joints_);
  joint_angles_vel_ = KDL::JntArray(planning_group_->num_joints_);
  joint_angles_acc_ = KDL::JntArray(planning_group_->num_joints_);
  all_joint_angles_ = KDL::JntArray(nj);
  for (int i=0; i<nj; ++i)
    all_joint_angles_(i) = 0.0;
  joint_axis_.resize(nj);
  joint_pos_.resize(nj);
  segment_frames_.resize(nl);
  collision_point_pos_.resize(nc);
  collision_point_vel_.resize(nc);
  collision_point_acc_.resize(nc);
  full_fk_done_ = false;
}

StompCostFunctionInput::~StompCostFunctionInput()
{
}

int StompCostFunctionInput::getNumDimensions() const
{
  return planning_group_->num_joints_;
}

void StompCostFunctionInput::doFK(boost::shared_ptr<KDL::TreeFkSolverJointPosAxisPartial> fk_solver)
{
  // first copy the group joints into all_joints:
  for (int i=0; i<planning_group_->num_joints_; ++i)
  {
    //printf("%f\t", joint_angles_(i));
    int kdl_index = planning_group_->stomp_joints_[i].kdl_joint_index_;
    all_joint_angles_(kdl_index) = joint_angles_(i);
  }

  //  if (!full_fk_done_)
  {
    fk_solver->JntToCartFull(all_joint_angles_, joint_pos_, joint_axis_, segment_frames_);
    full_fk_done_ = true;
  }
//  else
//  {
//    fk_solver->JntToCartPartial(joint_angles_, joint_pos_, joint_axis_, segment_frames_);
//  }

//  for (unsigned int i=0; i<segment_frames_.size(); ++i)
//  {
//    printf("segment %s: %f, %f, %f\n", planning_group_->fk_solver_->getSegmentNames()[i].c_str(),
//           segment_frames_[i].p.x(),
//           segment_frames_[i].p.y(),
//           segment_frames_[i].p.z());
//  }

  for (unsigned int i=0; i<planning_group_->collision_points_.size(); ++i)
  {
    planning_group_->collision_points_[i].getTransformedPosition(segment_frames_, collision_point_pos_[i]);
//    if (i==planning_group_->collision_points_.size()-1)
//    {
//      printf("%f, %f, %f\n", collision_point_pos_[i].x(),
//             collision_point_pos_[i].y(),
//             collision_point_pos_[i].z());
//    }
  }
}

void StompCostFunctionInput::publishVizMarkers(const ros::Time& stamp, ros::Publisher& publisher)
{
//  for (int i=0; i<planning_group_->num_joints_; ++i)
//  {
//    printf("%f\t", joint_angles_(i));
//  }
//  printf("\n");
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(collision_point_pos_.size());
  for (unsigned int i=0; i<collision_point_pos_.size(); ++i)
  {
    visualization_msgs::Marker& marker = marker_array.markers[i];
    marker.header.frame_id = robot_model_->getReferenceFrame();
    marker.header.stamp = stamp;
    marker.ns=planning_group_->name_;
    marker.id=i;
    marker.type=visualization_msgs::Marker::SPHERE;
    marker.action=visualization_msgs::Marker::ADD;
    //marker.points.resize(group_->collision_points_.size());
    marker.color.a=1.0;
    marker.color.r=0.0;
    marker.color.g=1.0;
    marker.color.b=0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.position.x = collision_point_pos_[i].x();
    marker.pose.position.y = collision_point_pos_[i].y();
    marker.pose.position.z = collision_point_pos_[i].z();
    marker.scale.x=2*planning_group_->collision_points_[i].getRadius();
    marker.scale.y=2*planning_group_->collision_points_[i].getRadius();
    marker.scale.z=2*planning_group_->collision_points_[i].getRadius();
  }
  publisher.publish(marker_array);
}

} /* namespace stomp_ros_interface */
