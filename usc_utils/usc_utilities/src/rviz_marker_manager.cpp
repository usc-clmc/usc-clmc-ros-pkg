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
                 int id)
{
  std::stringstream ss;
  ss << ns;
  if (id>=0)
    ss << id;

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time();
  marker.ns = ss.str();
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.mesh_resource = resource;
  tf::poseTFToMsg(pose, marker.pose);
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.5;
  pub_.publish(marker);
  addToClearList(marker);
}


void RvizMarkerManager::addToClearList(const visualization_msgs::Marker& marker)
{
  clear_list_.insert(std::make_pair(std::make_pair(marker.ns, marker.id), marker));
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
