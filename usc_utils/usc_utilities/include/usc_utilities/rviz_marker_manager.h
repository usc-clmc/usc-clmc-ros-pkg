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

  /*! Publish a mesh
   * @param resource
   * @param pose
   * @param frame
   * @param ns
   * @param id
   * @param r (default is 0.0)
   * @param g (default is 0.0)
   * @param b (default is 1.0)
   * @param a (default is 0.5)
   */
  void publishMesh(const std::string& resource,
                   const tf::Transform& pose,
                   const std::string& frame,
                   const std::string& ns,
                   int id,
                   const double r = 0.0,
                   const double g = 0.0,
                   const double b = 1.0,
                   const double a = 0.5);

  /*! Removes a mesh
   * @param ns namespace of the marker
   * @param id id of the marker
   */
  void clearMarker(const std::string& ns, const int id);

  void clearAll();

private:
  RvizPublisher pub_;

  typedef std::map<std::pair<std::string, int>, visualization_msgs::Marker> ClearListMap;
  ClearListMap clear_list_;
  void addToClearList(const visualization_msgs::Marker& marker);

};

} /* namespace usc_utilities */
#endif /* RVIZ_MARKER_MANAGER_H_ */
