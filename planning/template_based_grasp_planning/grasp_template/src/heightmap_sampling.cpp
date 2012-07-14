/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         heightmap_sampling.cpp

 \author       Alexander Herzog
 \date         April 1 2012

 *********************************************************************/

#include <limits>

#include <ros/ros.h>
#include <pcl/surface/convex_hull.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/features/normal_3d.h>
#include <grasp_template/heightmap_sampling.h>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace boost;
using namespace visualization_msgs;

namespace grasp_template
{

inline void computeTempltOrientation(const Normal& n, double z_angle, Quaterniond& orientation)
{
  Quaterniond rot_to_hm_frame;
  rot_to_hm_frame.setFromTwoVectors(Vector3d(0, 0, 1), Vector3d(n.normal[0],
      n.normal[1], n.normal[2]));

  Quaterniond rot_about_z(AngleAxisd(z_angle, Vector3d(0, 0, 1)));
  orientation = rot_to_hm_frame * rot_about_z;
}

HsIterator::HsIterator(unsigned int elements, double angle_step_size) :
  current_angle_(0.0), index_(0), elements_(elements), angle_step_size_(angle_step_size)
{
}

void HsIterator::inc()
{
  current_angle_ += angle_step_size_;
  if (current_angle_ > 2 * M_PI)
  {
    current_angle_ -= 2 * M_PI;
    index_++;
  }
}

void HsIterator::setIndex(unsigned int index)
{
  current_angle_ = 0;
  index_ = index;
}

bool HsIterator::passedLast()
{
  return index_ >= elements_;
}

/* HeightmapSampling */

HeightmapSampling::HeightmapSampling()
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    if (!listener.waitForTransform(frameBase(), frameViewPoint(), ros::Time(0), ros::Duration(5.0)))
    {
      ROS_DEBUG("grasp_template::HeightmapSampling: Wait for transform timed out! "
          "Using default viewpoint transformation!");

      viewp_rot_ = Quaterniond::Identity();
      viewp_trans_ = getDefaultViewpoint();
    }
    else
    {
      listener.lookupTransform(frameBase(), frameViewPoint(), ros::Time(0), transform);

      viewp_trans_.x() = transform.getOrigin().x();
      viewp_trans_.y() = transform.getOrigin().y();
      viewp_trans_.z() = transform.getOrigin().z();

      viewp_rot_.w() = transform.getRotation().w();
      viewp_rot_.x() = transform.getRotation().x();
      viewp_rot_.y() = transform.getRotation().y();
      viewp_rot_.z() = transform.getRotation().z();
    }
  }
  catch (tf::TransformException ex)
  {
    ROS_DEBUG("grasp_template::HeightmapSampling: %s", ex.what());
  }
}

HeightmapSampling::HeightmapSampling(const Vector3d& viewpoint_trans,
    const Quaterniond& viewpoint_quat)
{
  viewp_rot_ = viewpoint_quat;
  viewp_trans_ = viewpoint_trans;
  ROS_DEBUG_STREAM("grasp_template::HeightmapSampling: viewpoint_trans: " << viewp_trans_ <<
      " viewpoint_rot: " << viewp_rot_.w() << " "<< viewp_rot_.x() << " "
      << viewp_rot_.y() << " "<< viewp_rot_.z() << " ");
}

void HeightmapSampling::initialize(const pcl::PointCloud<pcl::PointXYZ>& cluster,
    const geometry_msgs::Pose& table)
{
  table_pose_ = table;

  point_cloud_.reset(new PointCloud<PointXYZ> (cluster));
  img_ss_.initialize(cluster, viewp_trans_, viewp_rot_);

  calculateConvexHull();
  calculateNormalsFromHullSurface();
}

bool HeightmapSampling::generateTemplateOnHull(GraspTemplate& templt,
    const Vector3d& ref, double z_angle)
{
  /* find nearest point */
  double min_dist = numeric_limits<double>::max();
  if (search_points_->size() == 0 || normals_.size() == 0)
    return false;
  PointXYZ closest = *search_points_->begin();
  Normal min_n = *normals_.begin();

  PointCloud<Normal>::const_iterator n_it = normals_.begin();
  for (PointCloud<PointXYZ>::const_iterator p_it = search_points_->begin();
      n_it != normals_.end() && p_it != search_points_->end(); n_it++, p_it++)
  {
    Vector3d diff = ref - Vector3d(p_it->x, p_it->y, p_it->z);
    double dist = diff.norm();

    if (dist < min_dist)
    {
      min_dist = dist;
      closest = *p_it;
      min_n = *n_it;
    }
  }

  Vector3d center(closest.x, closest.y, closest.z);
  Quaterniond orientation;
  computeTempltOrientation(min_n, 0, orientation);

  return generateTemplate(templt, center, orientation);
}

bool HeightmapSampling::generateTemplateOnHull(GraspTemplate& templt, const HsIterator& it)
{
  const PointXYZ& p = search_points_->points[it.index_];
  Vector3d center(p.x, p.y, p.z);
  Quaterniond orientation;
  computeTempltOrientation(normals_.points[it.index_], it.current_angle_, orientation);

  return generateTemplate(templt, center, orientation);
}

bool HeightmapSampling::generateTemplate(GraspTemplate& templt, const Vector3d& position,
                                         const Quaterniond& orientation)
{
  templt.object_to_template_translation_ = position;
  templt.object_to_template_rotation_ = orientation;

  Transform<double, 3, Affine> to_templt;
  to_templt.fromPositionOrientationScale(templt.object_to_template_translation_,
      templt.object_to_template_rotation_, Vector3d::Ones());
  to_templt = to_templt.inverse();

  /* choose points in range for heightmap and set heights */
  for (PointCloud<PointXYZ>::const_iterator it = point_cloud_->begin(); it != point_cloud_->end(); it++)
  {
    /* transform to heightmap frame */
    Vector3d p(it->x, it->y, it->z);
    p = to_templt * p;

    /* check if point is in range */
    if (std::abs(p(0)) <= templt.heightmap_.getMapLengthX() / 2.0 && std::abs(p(1))
        <= templt.heightmap_.getMapLengthY() / 2.0)
    {
      /* is in range, so add to heightmap */
      double cur = templt.heightmap_.getGridTile(p(0), p(1));
      if (templt.heightmap_.isUnset(cur) || cur < p(2))
      {
        templt.heightmap_.setGridTileSolid(p(0), p(1), p(2));
      }
    }
    else
    {
      continue;
    }
  }

  /* add fog and empty tiles */
  Transform<double, 3, Affine> to_world = to_templt.inverse();
  img_ss_.resetBinOrientation(templt.object_to_template_rotation_);
  for (unsigned int ix = 0; ix < templt.heightmap_.getNumTilesX(); ix++)
  {
    for (unsigned int iy = 0; iy < templt.heightmap_.getNumTilesY(); iy++)
    {
      double raw = templt.heightmap_.getGridTileRaw(ix, iy);
      if (templt.heightmap_.isUnset(raw))
      {
        Vector3d bin_center_ts;
        templt.heightmap_.gridToWorldCoordinates(ix, iy, bin_center_ts.x(), bin_center_ts.y());
        bin_center_ts.z() = 0;

        Vector3d bin_center_world = to_world * bin_center_ts;

        if (!img_ss_.resetBin(bin_center_world))
        {
          templt.heightmap_.setGridTileEmpty(bin_center_ts.x(), bin_center_ts.y());

          continue;
        }

        double fog_height;

        if (img_ss_.getFogHeight(fog_height))
        {
          templt.heightmap_.setGridTileFog(bin_center_ts.x(), bin_center_ts.y(), fog_height);
        }
        else
        {
          templt.heightmap_.setGridTileEmpty(bin_center_ts.x(), bin_center_ts.y());
        }
      }
    }
  }

  //add table to the template
  addTable(templt);

  return true;
}

void HeightmapSampling::addTable(grasp_template::GraspTemplate& t) const
{
  Vector3d table_point, dir_1(1, 0, 0), dir_2(0, 1, 0);
  table_point.x() = table_pose_.position.x;
  table_point.y() = table_pose_.position.y;
  table_point.z() = table_pose_.position.z;
  Quaterniond table_orientation;
  table_orientation.w() = table_pose_.orientation.w;
  table_orientation.x() = table_pose_.orientation.x;
  table_orientation.y() = table_pose_.orientation.y;
  table_orientation.z() = table_pose_.orientation.z;
  dir_1 = table_orientation * dir_1;
  dir_2 = table_orientation * dir_2;

  table_point = t.object_to_template_rotation_.inverse() * (table_point -
      t.object_to_template_translation_);
  dir_1 = t.object_to_template_rotation_.inverse() * dir_1;
  dir_2 = t.object_to_template_rotation_.inverse() * dir_2;

  Vector3d n = dir_1.cross(dir_2);
  n.normalize();
  const double tile_length_x = t.heightmap_.getMapLengthX() / t.heightmap_.getNumTilesX();
  const double tile_length_y = t.heightmap_.getMapLengthY() / t.heightmap_.getNumTilesY();
  const double x0 = -t.heightmap_.getMapLengthX() / 2.0 + tile_length_x / 2.0;
  const double y0 = -t.heightmap_.getMapLengthY() / 2.0 + tile_length_y / 2.0;

  if (abs(n.z()) < 0.000001)
  {
    return;
  }

  for (unsigned int ix = 0; ix < t.heightmap_.getNumTilesX(); ix++)
  {
    for (unsigned int iy = 0; iy < t.heightmap_.getNumTilesY(); iy++)
    {
      const double x = x0 + ix * tile_length_x;
      const double y = y0 + iy * tile_length_y;

      double z = (n.dot(table_point) - (x * n.x() + y * n.y())) / n.z();
      Vector3d tbl(x, y, z);
      tbl = tbl - table_point;

      const double cur_tile_val = t.heightmap_.getGridTile(x, y);
      if (cur_tile_val < z || t.heightmap_.isEmpty(cur_tile_val))
        t.heightmap_.setGridTileTable(x, y, z);
    }
  }
}

//void HeightmapSampling::pclToSensorMsg(const PointCloud<PointXYZ>& in,
//    sensor_msgs::PointCloud& out) const
//{
//  out.header.frame_id = point_cloud_->header.frame_id;
//  out.header.stamp = ros::Time::now();
//
//  for (unsigned int i = 0; i < in.size(); i++)
//  {
//    geometry_msgs::Point32 p;
//    p.x = in.points[i].x;
//    p.y = in.points[i].y;
//    p.z = in.points[i].z;
//
//    out.points.push_back(p);
//  }
//}

Marker HeightmapSampling::getVisualizationNormals(const string& ns, const string& frame_id, int id) const
{
  Marker line_list;

  line_list.header.frame_id = frame_id;//
  line_list.header.stamp = ros::Time::now();

  line_list.ns = ns;//"templt_cluster_viz";
  line_list.id = id;//1

  line_list.type = Marker::LINE_LIST;
  line_list.action = Marker::ADD;

  line_list.pose.orientation.w = 1.0;
  line_list.scale.x = 0.001;
  line_list.color.g = 1.0;
  line_list.color.a = 1.0;

  PointCloud<Normal>::const_iterator normals_it = normals_.begin();
  for (PointCloud<PointXYZ>::const_iterator points_it = search_points_->begin();
      points_it != search_points_->end()  && normals_it != normals_.end(); points_it++,
      normals_it++)
  {
    /* from point */
    geometry_msgs::Point p;
    p.x = points_it->x;
    p.y = points_it->y;
    p.z = points_it->z;

    line_list.points.push_back(p);

    /* in normal direction */
    p.x += normals_it->normal[0] * 0.1;
    p.y += normals_it->normal[1] * 0.1;
    p.z += normals_it->normal[2] * 0.1;

    line_list.points.push_back(p);
  }

  return line_list;
}

bool HeightmapSampling::calculateNormalsFromHullSurface()
{
  normals_.points.clear();
  shared_ptr < PointCloud<PointXYZ> > sp;
  sp.reset(new PointCloud<PointXYZ> ());
  Vector3d search_point_center(0, 0, 0);
  vector < Vector3d > normals_prestep;

  for (unsigned int i = 0; i < convex_hull_vertices_.size(); i++)
  {
    PointXYZ p(0, 0, 0);

    for (unsigned int j = 0; j < convex_hull_vertices_[i].vertices.size(); j++)
    {
      unsigned int index = convex_hull_vertices_[i].vertices[j];

      p.x += convex_hull_points_->points[index].x;
      p.y += convex_hull_points_->points[index].y;
      p.z += convex_hull_points_->points[index].z;

    }

    p.x /= convex_hull_vertices_[i].vertices.size();
    p.y /= convex_hull_vertices_[i].vertices.size();
    p.z /= convex_hull_vertices_[i].vertices.size();

    search_point_center += Vector3d(p.x, p.y, p.z);

    sp->push_back(p);

    /* normal */
    Vector3d p_centr(p.x, p.y, p.z);

    const PointXYZ& p_a = convex_hull_points_-> points[convex_hull_vertices_[i].vertices[0]];
    const PointXYZ& p_b = convex_hull_points_-> points[convex_hull_vertices_[i].vertices[1]];
    const PointXYZ& p_c = convex_hull_points_-> points[convex_hull_vertices_[i].vertices[2]];

    Vector3d va(p_b.x - p_a.x, p_b.y - p_a.y, p_b.z - p_a.z);
    Vector3d vb(p_c.x - p_a.x, p_c.y - p_a.y, p_c.z - p_a.z);
    Vector3d vn = va.cross(vb);

    vn.normalize();

    normals_prestep.push_back(vn);

  }

  search_point_center /= sp->points.size();

  /* flip normals away from center */
  vector<bool> backside;
  backside.resize(normals_prestep.size(), false);
  for (unsigned int i = 0; i < normals_prestep.size(); i++)
  {
    Vector3d s_point(sp->points[i].x - search_point_center.x(), sp->points[i].y - search_point_center.y(),
                     sp->points[i].z - search_point_center.z());

    if (s_point.dot(normals_prestep[i]) < 0)
    {
      normals_prestep[i] = -normals_prestep[i];
    }

    Vector3d vec_vs(sp->points[i].x - viewp_trans_.x(), sp->points[i].y - viewp_trans_.y(),
        sp->points[i].z - viewp_trans_.z());
    vec_vs.normalize();

    if (vec_vs.dot(normals_prestep[i]) > 0)
    {
      backside[i] = true;
    }
  }

  /* set normals */
  for (unsigned int i = 0; i < normals_prestep.size(); i++)
  {
    if (backside[i])
    {
      continue;
    }

    Normal pcl_n;
    pcl_n.normal_x = normals_prestep[i].x();
    pcl_n.normal_y = normals_prestep[i].y();
    pcl_n.normal_z = normals_prestep[i].z();
    normals_.push_back(pcl_n);
  }

  /* filter and set search-points */
  for (int i = backside.size() - 1; i >= 0; i--)
  {
    if (backside[i])
    {
      sp->erase(sp->points.begin() + i);
    }
  }
  search_points_ = sp;

  ROS_DEBUG_STREAM("grasp_template::HeightmapSampling: #normals: " << normals_.size()
      << ", #search_points: " << search_points_->points.size() << endl);
  return true;
}

bool HeightmapSampling::calculateConvexHull()
{
  ConvexHull < PointXYZ > cv;
  cv.setInputCloud(point_cloud_);
  boost::shared_ptr < PointCloud<PointXYZ> > cv_points;
  cv_points.reset(new PointCloud<PointXYZ> ());
  cv.reconstruct(*cv_points, convex_hull_vertices_);

  convex_hull_points_ = cv_points;

  return true;
}

} //namespace
