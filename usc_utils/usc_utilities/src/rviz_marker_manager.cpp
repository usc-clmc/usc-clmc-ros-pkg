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
                 double size)
{
  tf::Transform tf;
  tf::poseMsgToTF(pose.pose, tf);
  publishPose(tf, pose.header.frame_id, ns, size);
}

void RvizMarkerManager::publishPose(const tf::Transform& pose,
                 const std::string& frame,
                 const std::string& ns,
                 double size)
{
  std::stringstream ss;
  ss << ns;

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time();
  marker.id = 0;
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
  std::vector<visualization_msgs::Marker> markers;
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
    //addToClearList(marker);
    markers.push_back(marker);
  }
  addToClearList(ns, markers);

}

void RvizMarkerManager::publishMesh(const std::string& resource,
                 const tf::Transform& pose,
                 const std::string& frame,
                 const std::string& ns,
                 const double r, const double g, const double b, const double a)
{
  std::stringstream ss;
  ss << ns;

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ss.str();
  marker.id = 0;
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
                                    const double r, const double g, const double b, const double a)
{
  tf::Transform transform;
  tf::poseMsgToTF(pose, transform);
  publishMesh(resource, transform, frame, ns, r, g, b, a);
}

void RvizMarkerManager::publishText(const std::string& text,
                 const tf::Transform& pose,
                 const std::string& frame,
                 const std::string& ns,
                 const double size,
                 const double r,
                 const double g,
                 const double b,
                 const double a)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.text = text;
  tf::poseTFToMsg(pose, marker.pose);
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = size;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  pub_.publish(marker);
  addToClearList(marker);
}

void RvizMarkerManager::addToClearList(const visualization_msgs::Marker& marker)
{
  std::vector<visualization_msgs::Marker> markers;
  markers.push_back(marker);
  addToClearList(marker.ns, markers);
}

void RvizMarkerManager::addToClearList(const std::string& ns, const std::vector<visualization_msgs::Marker>& markers)
{
  clear_list_.insert(std::make_pair(ns, markers));
}

void RvizMarkerManager::clearMarker(const std::string& ns, const bool force)
{
  //std::vector<ClearListMap::value_type> v(clear_list_.begin(), clear_list_.end());
  ClearListMap::iterator item = clear_list_.find(ns);
  if (item == clear_list_.end())
  {
    if (force)
    {
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::DELETE;
      marker.ns = ns;
      marker.id = 0;
      pub_.publish(marker);
    }
    return;
  }
  std::vector<visualization_msgs::Marker> markers = item->second;
  for (unsigned int i=0; i<markers.size(); ++i)
  {
    visualization_msgs::Marker marker = markers[i];
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETE;
    pub_.publish(marker);
  }
  clear_list_.erase(item);
}

void RvizMarkerManager::clearAll()
{
  std::vector<ClearListMap::value_type> v(clear_list_.begin(), clear_list_.end());
  for (unsigned int i=0; i<v.size(); ++i)
  {
    clearMarker(v[i].first, false);
  }
  clear_list_.clear();
}

RvizMarkerManager::~RvizMarkerManager()
{
}

} /* namespace usc_utilities */
