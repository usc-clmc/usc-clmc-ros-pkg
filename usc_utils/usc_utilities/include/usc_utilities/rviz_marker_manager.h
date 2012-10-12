/*
 * rviz_marker_manager.h
 *
 *  Created on: Oct 10, 2012
 *      Author: kalakris
 */

#ifndef RVIZ_MARKER_MANAGER_H_
#define RVIZ_MARKER_MANAGER_H_

#include <usc_utilities/rviz_publisher.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

namespace usc_utilities
{

class RvizMarkerManager
{
public:
  RvizMarkerManager(const std::string& topic_name);
  virtual ~RvizMarkerManager();

  /**
   * Publish a pose as markers
   *
   * ns: namespace
   * size: length of each arrow
   */
  void publishPose(const geometry_msgs::PoseStamped& pose,
                   const std::string& ns,
                   int id,
                   double size);
  void publishPose(const tf::Transform& pose,
                   const std::string& frame,
                   const std::string& ns,
                   int id,
                   double size);

  void publishMesh(const std::string& resource,
                   const tf::Transform& pose,
                   const std::string& frame,
                   const std::string& ns,
                   int id);

  void clearAll();

private:
  RvizPublisher pub_;

  typedef std::map<std::pair<std::string, int>, visualization_msgs::Marker> ClearListMap;
  ClearListMap clear_list_;
  void addToClearList(const visualization_msgs::Marker& marker);

};

} /* namespace usc_utilities */
#endif /* RVIZ_MARKER_MANAGER_H_ */
