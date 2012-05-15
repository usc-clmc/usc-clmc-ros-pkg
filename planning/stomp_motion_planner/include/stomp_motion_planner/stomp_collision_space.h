/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#ifndef STOMP_COLLISION_SPACE_H_
#define STOMP_COLLISION_SPACE_H_

#include <mapping_msgs/CollisionMap.h>
#include <mapping_msgs/CollisionObject.h>
#include <motion_planning_msgs/RobotState.h>

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <stomp_motion_planner/stomp_collision_point.h>
#include <stomp_motion_planner/stomp_robot_model.h>
#include <Eigen/Core>
#include <distance_field/distance_field.h>
#include <distance_field/propagation_distance_field.h>
#include <distance_field/pf_distance_field.h>
#include <planning_environment/models/collision_models.h>
#include <planning_environment/monitors/collision_space_monitor.h>

namespace stomp_motion_planner
{

class StompCollisionSpace
{

// public:

//   struct KnownObject {

//     KnownObject(void) 
//     {
//     }

//     ~KnownObject() {
//     }
    
//     void deleteBodies() {
//       for(unsigned int i = 0; i < bodies_.size(); i++) {
//         delete bodies_[i];
//       }
//       bodies_.clear();
//     }

//     std::vector<bodies::Body*> bodies_;
//     std::vector<tf::Vector3> voxels_;

//   };

public:
  StompCollisionSpace();
  virtual ~StompCollisionSpace();

  /**
   * \brief Callback for CollisionMap messages
   */
  //void collisionMapCallback(const mapping_msgs::CollisionMapConstPtr& collision_map);

  //void collisionObjectCallback(const mapping_msgs::CollisionObjectConstPtr &collisionObject);

  /**
   * \brief Initializes the collision space, listens for messages, etc
   *
   * \return false if not successful
   */
  bool init(planning_environment::CollisionSpaceMonitor* monitor_, double max_radius_clearance, std::string& reference_frame);

  /**
   * \brief Lock the collision space from updating/reading
   */
  void lock();

  /**
   * \brief Unlock the collision space for updating/reading
   */
  void unlock();

  double getDistanceGradient(double x, double y, double z,
      double& gradient_x, double& gradient_y, double& gradient_z) const;

  void setStartState(const StompRobotModel::StompPlanningGroup& planning_group, const motion_planning_msgs::RobotState& robot_state);

  inline void worldToGrid(tf::Vector3 origin, double wx, double wy, double wz, int &gx, int &gy, int &gz) const;

  inline void gridToWorld(tf::Vector3 origin, int gx, int gy, int gz, double &wx, double &wy, double &wz) const;

  template<typename Derived, typename DerivedOther>
  bool getCollisionPointPotentialGradient(const StompCollisionPoint& collision_point, const Eigen::MatrixBase<Derived>& collision_point_pos,
      double& potential, Eigen::MatrixBase<DerivedOther>& gradient) const;

private:

  std::vector<tf::Vector3> interpolateTriangle(tf::Vector3 v0, 
                                             tf::Vector3 v1, 
                                             tf::Vector3 v2, double min_res);
 
  double dist(const tf::Vector3 &v0, const tf::Vector3 &v1)
  {
    return sqrt( (v1.x()-v0.x())*(v1.x()-v0.x()) + 
                 (v1.y()-v0.y())*(v1.y()-v0.y()) +  
                 (v1.z()-v0.z())*(v1.z()-v0.z()) );
  }
 
  ros::NodeHandle node_handle_, root_handle_;
  distance_field::PropagationDistanceField* distance_field_;

  std::string reference_frame_;
  boost::mutex mutex_;
  std::vector<tf::Vector3> cuboid_points_;

  double max_expansion_;
  double resolution_;
  double field_bias_x_;
  double field_bias_y_;
  double field_bias_z_;

  planning_environment::CollisionSpaceMonitor *monitor_;
  std::map<std::string, std::vector<std::string> > planning_group_link_names_;
  std::map<std::string, std::vector<bodies::Body *> > planning_group_bodies_;

  void initCollisionCuboids();
  void addCollisionCuboid(const std::string param_name);

  planning_environment::CollisionModels* collision_models_;

  void loadRobotBodies();
  void updateRobotBodiesPoses(const planning_models::KinematicState& state);
  void getVoxelsInBody(const bodies::Body &body, std::vector<tf::Vector3> &voxels);
  void addCollisionObjectsToPoints(std::vector<tf::Vector3>& points, const tf::Transform& cur);
  void addBodiesInGroupToPoints(const std::string& group, std::vector<tf::Vector3> &voxels);
  void addAllBodiesButExcludeLinksToPoints(std::string group_name, std::vector<tf::Vector3>& body_points);  

  std::map<std::string, std::vector<std::string> > distance_exclude_links_;
  std::map<std::string, std::vector<std::string> > distance_include_links_;

};

///////////////////////////// inline functions follow ///////////////////////////////////

inline void StompCollisionSpace::lock()
{
  mutex_.lock();
}

inline void StompCollisionSpace::unlock()
{
  mutex_.unlock();
}

inline double StompCollisionSpace::getDistanceGradient(double x, double y, double z,
    double& gradient_x, double& gradient_y, double& gradient_z) const
{
  return distance_field_->getDistanceGradient(x, y, z, gradient_x, gradient_y, gradient_z);
}

template<typename Derived, typename DerivedOther>
bool StompCollisionSpace::getCollisionPointPotentialGradient(const StompCollisionPoint& collision_point, const Eigen::MatrixBase<Derived>& collision_point_pos,
    double& potential, Eigen::MatrixBase<DerivedOther>& gradient) const
{
  Eigen::Vector3d field_gradient;
  double field_distance = getDistanceGradient(
      collision_point_pos(0), collision_point_pos(1), collision_point_pos(2),
      field_gradient(0), field_gradient(1), field_gradient(2));

  field_gradient(0) += field_bias_x_;
  field_gradient(1) += field_bias_y_;
  field_gradient(2) += field_bias_z_;

  double d = field_distance - collision_point.getRadius();

  // three cases below:
  if (d >= collision_point.getClearance())
  {
    potential = 0.0;
    gradient.setZero();
  }
  else if (d >= 0.0)
  {
    double diff = (d - collision_point.getClearance());
    double gradient_magnitude = diff * collision_point.getInvClearance(); // (diff / clearance)
    potential = 0.5*gradient_magnitude*diff;
    gradient = gradient_magnitude * field_gradient;
  }
  else // if d < 0.0
  {
    gradient = field_gradient;
    potential = -d + 0.5 * collision_point.getClearance();
  }

  return (field_distance <= collision_point.getRadius()); // true if point is in collision
}

inline void StompCollisionSpace::worldToGrid(tf::Vector3 origin, double wx, double wy, double wz, int &gx, int &gy, int &gz) const {
  gx = (int)((wx - origin.x()) * (1.0 / resolution_));
  gy = (int)((wy - origin.y()) * (1.0 / resolution_));
  gz = (int)((wz - origin.z()) * (1.0 / resolution_));
}

/** \brief Convert from voxel grid coordinates to world coordinates. */
inline void StompCollisionSpace::gridToWorld(tf::Vector3 origin, int gx, int gy, int gz, double &wx, double &wy, double &wz) const {
  wx = gx * resolution_ + origin.x();
  wy = gy * resolution_ + origin.y();
  wz = gz * resolution_ + origin.z();
}

} // namespace stomp

#endif /* STOMP_COLLISION_SPACE_H_ */
