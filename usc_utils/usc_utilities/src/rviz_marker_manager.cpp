/*
 * rviz_marker_manager.cpp
 *
 *  Created on: Oct 10, 2012
 *      Author: kalakris
 */

#include <usc_utilities/rviz_marker_manager.h>
#include <sstream>

namespace usc_utilities
{

RvizMarkerManager::RvizMarkerManager(const std::string& topic_name):
    pub_(topic_name)
{

}

void RvizMarkerManager::publishPose(const geometry_msgs::PoseStamped& pose,
                 const std::string& ns,
                 int id,
                 double size)
{
  tf::Transform tf;
  tf::poseMsgToTF(pose.pose, tf);
  publishPose(tf, pose.header.frame_id, ns, id, size);
}

void RvizMarkerManager::publishPose(const tf::Transform& pose,
                 const std::string& frame,
                 const std::string& ns,
                 int id,
                 double size)
{
  std::stringstream ss;
  ss << ns;
  if (id>=0)
    ss << id;

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time();
  marker.id = id;
  marker.ns = ss.str();
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points.resize(2);
  marker.points[0].x = pose.getOrigin().getX();
  marker.points[0].y = pose.getOrigin().getY();
  marker.points[0].z = pose.getOrigin().getZ();
  marker.scale.x = size/10.0;
  marker.scale.y = size/5.0;
  marker.color.a = 1.0;

  tf::Matrix3x3 identity;
  identity.setIdentity();
  for (int i=0; i<3; ++i)
  {
    marker.id = i;

    tf::Vector3 tip = pose * (identity.getColumn(i) * size);
    marker.points[1].x = tip.getX();
    marker.points[1].y = tip.getY();
    marker.points[1].z = tip.getZ();
    marker.color.r = (i==0) ? 1.0 : 0.0;
    marker.color.g = (i==1) ? 1.0 : 0.0;
    marker.color.b = (i==2) ? 1.0 : 0.0;
    pub_.publish(marker);
    addToClearList(marker);
  }

}

void RvizMarkerManager::publishMesh(const std::string& resource,
                 const tf::Transform& pose,
                 const std::string& frame,
                 const std::string& ns,
                 int id,
                 const double r, const double g, const double b, const double a)
{
  std::stringstream ss;
  ss << ns;
  //if (id>=0)
  //ss << id;

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ss.str();
  marker.id = id;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.mesh_resource = resource;
  tf::poseTFToMsg(pose, marker.pose);
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  pub_.publish(marker);
  addToClearList(marker);
}

void RvizMarkerManager::publishMesh(const std::string& resource,
                                    const geometry_msgs::Pose& pose,
                                    const std::string& frame,
                                    const std::string& ns,
                                    int id,
                                    const double r, const double g, const double b, const double a)
{
  tf::Transform transform;
  tf::poseMsgToTF(pose, transform);
  publishMesh(resource, transform, frame, ns, id, r, g, b, a);
}

void RvizMarkerManager::addToClearList(const visualization_msgs::Marker& marker)
{
  clear_list_.insert(std::make_pair(std::make_pair(marker.ns, marker.id), marker));
}

void RvizMarkerManager::clearMarker(const std::string& ns, const int id, const bool force, const std::string frame_id)
{
  std::vector<ClearListMap::value_type> v(clear_list_.begin(), clear_list_.end());
  ClearListMap::iterator item = clear_list_.find(std::pair<std::string, int>(ns, id));
  if (item == clear_list_.end())
  {
    if (force)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = frame_id;
      marker.action = visualization_msgs::Marker::DELETE;
      marker.ns = ns;
      marker.id = id;
      pub_.publish(marker);
    }
    // ROS_WARN("Cannot find marker with namespace >%s< and id >%i<. Not clearing it.", ns.c_str(), id);
    // ClearListMap::const_iterator ci;
    // for (ci = clear_list_.begin(); ci != clear_list_.end(); ++ci)
    // {
    // ROS_WARN(" ns: %s id: %i", ci->first.first.c_str(), ci->first.second);
    // }
    return;
  }
  visualization_msgs::Marker marker = item->second;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.action = visualization_msgs::Marker::DELETE;
  pub_.publish(marker);
  clear_list_.erase(item);
}

void RvizMarkerManager::clearAll()
{
  std::vector<ClearListMap::value_type> v(clear_list_.begin(), clear_list_.end());
  for (unsigned int i=0; i<v.size(); ++i)
  {
    visualization_msgs::Marker marker = v[i].second;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETE;
    pub_.publish(marker);
  }
  clear_list_.clear();
}

RvizMarkerManager::~RvizMarkerManager()
{
}

} /* namespace usc_utilities */
