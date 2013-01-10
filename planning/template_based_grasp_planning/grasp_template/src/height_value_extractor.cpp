/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks              ...

 \file         height_value_extractor.cpp

 \author       Alexander Herzog
 \date         April 1 2012

 *********************************************************************/

#include <vector>

#include <geometry_msgs/Point32.h>
#include <grasp_template/grasp_template_params.h>
#include <grasp_template/height_value_extractor.h>
#include <grasp_template/template_heightmap.h>

using namespace std;
using namespace Eigen;

namespace grasp_template
{

HeightValueExtractor::HeightValueExtractor()
{
  roi_threshold_ = max(GraspTemplateParams::getTemplateWidth() / TemplateHeightmap::TH_DEFAULT_NUM_TILES_X,
      GraspTemplateParams::getTemplateWidth() / TemplateHeightmap::TH_DEFAULT_NUM_TILES_Y) * 0.1;
}

void HeightValueExtractor::getPointsInROI(vector<int>& indices) const
{
  for (int i = 0; i < points_.cols(); i++)
  {
    if (isPointInRegionOfInterest(points_.col(i)))
    {
      indices.push_back(i);
    }
  }
}

bool HeightValueExtractor::getFogHeight(double& fog_height)
{
  vector<int> poi_indices;
  getPointsInROI(poi_indices);
  bool is_fog = false;
  int max_index = -1;
  double max_height = -numeric_limits<double>::max();

  for (unsigned int i = 0; i < poi_indices.size(); i++)
  {
    int point_index = poi_indices[i];
    const Vector3d& point = points_.col(point_index);
    if (isPointOccludingBin(point))
    {
      is_fog = true;

      /* get height along bin ray */
      double cur_height = bin_orientation_rot_.col(2).dot(point - bin_center_);

      if (cur_height > max_height) //point is "higher"
      {
        max_height = cur_height;
        max_index = point_index;
      }
    }
  }

  if (is_fog)
  {
    const Vector3d& p = points_.col(max_index);
    const double lambda = sep_const_ / (sep_normal_.dot(p));
    fog_height = bin_orientation_rot_.col(2).dot(lambda * p - bin_center_);
  }
  return is_fog;
}

void HeightValueExtractor::initialize(const pcl::PointCloud<pcl::PointXYZ>& pc, const Vector3d& trans, const Quaterniond& rot)
{
  to_grid_transform_.fromPositionOrientationScale(rot.inverse() * (-trans), rot.inverse(), Vector3d::Ones());
  points_.resize(3, pc.points.size());

  for (unsigned int i = 0; i < pc.points.size(); i++)
  {
    Vector3d p;
    p.x() = pc.points[i].x;
    p.y() = pc.points[i].y;
    p.z() = pc.points[i].z;
    p = to_grid_transform_ * p;
    points_.col(i) = p;
  }
}

bool HeightValueExtractor::isPointInRegionOfInterest(const Vector3d& p) const
{
  return abs(p.dot(roi_normal_)) < roi_threshold_;
}

bool HeightValueExtractor::isPointOccludingBin(const Vector3d& p) const
{
  return sep_normal_.dot(p - bin_center_) < 0;
}

void HeightValueExtractor::resetBinOrientation(const Quaterniond& orientation)
{
  bin_orientation_rot_ = to_grid_transform_.rotation() * orientation.toRotationMatrix();
  //TODO: handle the case where the orientation and bin_center are linear dependent
}

bool HeightValueExtractor::resetBin(const Vector3d& bin_center)
{

  bin_center_ = to_grid_transform_ * bin_center;

  roi_normal_ = bin_center_.cross(bin_orientation_rot_.col(2));
  if (abs(roi_normal_.x()) + abs(roi_normal_.y()) + abs(roi_normal_.z()) < 0.000001)
  {
    ROS_DEBUG("grasp_template::HeightValueExtractor: Cross product is invalid!");
    return false;
  }
  roi_normal_.normalize();
  if (abs(roi_normal_.norm() - 1) > 0.000001)

  {
    ROS_DEBUG("grasp_template::HeightValueExtractor: Normal is invalid!");
    return false;
  }

  sep_normal_ = roi_normal_.cross(bin_orientation_rot_.col(2));
  sep_normal_.normalize();
  if (bin_center_.dot(sep_normal_) < 0)
  {
    sep_normal_ = -sep_normal_;
  }

  sep_const_ = bin_center_.dot(sep_normal_);
  return true;
}

} //namespace
