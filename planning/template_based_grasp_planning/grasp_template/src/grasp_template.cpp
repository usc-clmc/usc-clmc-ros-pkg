/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         grasp_template.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <vector>
#include <cmath>

#include <visualization_msgs/Marker.h>
#include <grasp_template/grasp_template.h>

using namespace visualization_msgs;
using namespace Eigen;
using namespace std;

namespace grasp_template
{

GraspTemplate::GraspTemplate(const Heightmap& hm, const geometry_msgs::Pose& pose) :
  heightmap_(hm.num_tiles_x, hm.num_tiles_y, hm.map_length_x, hm.map_length_y)
{
  object_to_template_translation_.x() = pose.position.x;
  object_to_template_translation_.y() = pose.position.y;
  object_to_template_translation_.z() = pose.position.z;

  object_to_template_rotation_.w() = pose.orientation.w;
  object_to_template_rotation_.x() = pose.orientation.x;
  object_to_template_rotation_.y() = pose.orientation.y;
  object_to_template_rotation_.z() = pose.orientation.z;

  heightmap_.setGrid(hm.heightmap);
}

void GraspTemplate::getPose(geometry_msgs::Pose& pose) const
{
  pose.position.x = object_to_template_translation_.x();
  pose.position.y = object_to_template_translation_.y();
  pose.position.z = object_to_template_translation_.z();

  pose.orientation.w = object_to_template_rotation_.w();
  pose.orientation.x = object_to_template_rotation_.x();
  pose.orientation.y = object_to_template_rotation_.y();
  pose.orientation.z = object_to_template_rotation_.z();
}

vector<Marker> GraspTemplate::getVisualization(const string& ns, const string& frame_id, int id) const
{
  Marker solid, empty, unset, fog, dont_care, zm, tabl;
  solid.header.frame_id = frame_id;
  solid.header.stamp = ros::Time::now();
  tabl.header.frame_id = frame_id;
  tabl.header.stamp = ros::Time::now();
  empty.header.frame_id = frame_id;
  empty.header.stamp = ros::Time::now();
  unset.header.frame_id = frame_id;
  unset.header.stamp = ros::Time::now();
  fog.header.frame_id = frame_id;
  fog.header.stamp = ros::Time::now();
  dont_care.header.frame_id = frame_id;
  dont_care.header.stamp = ros::Time::now();
  zm.header.frame_id = frame_id;
  zm.header.stamp = ros::Time::now();

  solid.ns = ns;
  tabl.ns = ns;
  empty.ns = ns;
  unset.ns = ns;
  fog.ns = ns;
  dont_care.ns = ns;
  zm.ns = "z_achsis";
  solid.id = id;
  empty.id = id + 1;
  unset.id = id + 2;
  fog.id = id + 3;
  dont_care.id = id + 4;
  zm.id = id + 5;
  tabl.id = id + 6;

  solid.type = Marker::POINTS;
  solid.action = Marker::ADD;
  tabl.type = Marker::POINTS;
  tabl.action = Marker::ADD;

  empty.type = Marker::POINTS;
  empty.action = Marker::ADD;
  unset.type = Marker::POINTS;
  unset.action = Marker::ADD;
  fog.type = Marker::POINTS;
  fog.action = Marker::ADD;
  dont_care.type = Marker::POINTS;
  dont_care.action = Marker::ADD;
  zm.type = Marker::ARROW;
  zm.action = Marker::ADD;

  double scale = 0.004;
  solid.pose.orientation.w = 1.0;
  solid.scale.x = scale;
  solid.scale.y = scale;
  solid.color.g = 1.0;
  solid.color.a = 1.0;

  tabl.pose.orientation.w = 1.0;
  tabl.scale.x = scale;
  tabl.scale.y = scale;
  tabl.color.r = 1.0;
  tabl.color.a = 1.0;

  empty.pose.orientation.w = 1.0;
  empty.scale.x = scale;
  empty.scale.y = scale;
  empty.color.b = 1.0;
  empty.color.a = 1.0;

  unset.pose.orientation.w = 1.0;
  unset.scale.x = scale;
  unset.scale.y = scale;
  unset.color.r = 1.0;
  unset.color.a = 1.0;

  fog.pose.orientation.w = 1.0;
  fog.scale.x = scale;
  fog.scale.y = scale;
  fog.color.g = 0.5;
  fog.color.r = 0.5;
  fog.color.b = 0.5;
  fog.color.a = 1.0;

  dont_care.pose.orientation.w = 1.0;
  dont_care.scale.x = scale;
  dont_care.scale.y = scale;
  dont_care.color.g = 0.0;
  dont_care.color.b = 1.0;
  dont_care.color.a = 1.0;

  zm.scale.x = 0.005;
  zm.scale.y = 0.01;
  zm.color.b = 1.0;
  zm.color.r = 1.0;
  zm.color.a = 1.0;

  for (unsigned int ix = 0; ix < heightmap_.getNumTilesX(); ++ix)
  {
    for (unsigned int iy = 0; iy < heightmap_.getNumTilesY(); ++iy)
    {

      Vector3d eig_point;
      heightmap_.gridToWorldCoordinates(ix, iy, eig_point.x(), eig_point.y());

      double raw = heightmap_.getGridTileRaw(ix, iy);
      if (heightmap_.isUnset(raw) || heightmap_.isEmpty(raw))
      {
        eig_point.z() = 0;
      }
      else
      {
        eig_point.z() = heightmap_.getGridTile(eig_point.x(), eig_point.y());
      }

      Transform<double, 3, Eigen::Affine> to_world;
      to_world.fromPositionOrientationScale(object_to_template_translation_, object_to_template_rotation_,
                                            Vector3d::Ones());
      eig_point = to_world * eig_point;

      geometry_msgs::Point p;
      p.x = static_cast<float> (eig_point.x());
      p.y = static_cast<float> (eig_point.y());
      p.z = static_cast<float> (eig_point.z());

      if (heightmap_.isUnset(raw))
      {
        unset.points.push_back(p);
      }
      else if (heightmap_.isEmpty(raw))
      {
        empty.points.push_back(p);
      }
      else if (heightmap_.isSolid(raw))
        solid.points.push_back(p);
      else if (heightmap_.isFog(raw))
        fog.points.push_back(p);
      else if (heightmap_.isDontCare(raw))
        dont_care.points.push_back(p);
      else if (heightmap_.isTable(raw))
        tabl.points.push_back(p);
    }
  }

  Vector3d z_achsis(0, 0, 0.1);
  z_achsis = object_to_template_rotation_ * z_achsis;
  geometry_msgs::Point p;
  p.x = static_cast<float> (object_to_template_translation_.x());
  p.y = static_cast<float> (object_to_template_translation_.y());
  p.z = static_cast<float> (object_to_template_translation_.z());
  zm.points.push_back(p);
  p.x += static_cast<float> (z_achsis.x());
  p.y += static_cast<float> (z_achsis.y());
  p.z += static_cast<float> (z_achsis.z());
  zm.points.push_back(p);

  vector<Marker> mv;
  mv.push_back(solid);
  mv.push_back(unset);
  mv.push_back(fog);
  mv.push_back(empty);
  mv.push_back(dont_care);
  mv.push_back(zm);
  mv.push_back(tabl);
  return mv;
}

} //namespace
