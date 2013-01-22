/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         dismatch_measure.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <grasp_template/heightmap_difference.h>
#include <grasp_template/dismatch_measure.h>

using namespace std;
using namespace Eigen;

namespace grasp_template
{

/* TemplateDissimilarity definitions */

TemplateDissimilarity::TemplateDissimilarity()
{
//	clamping_fac_ = 10.0;
  relevants_ = distances_sum_ = distance_sums_[0] = distance_sums_[1] = distance_sums_[2] = distance_sums_[3] = 0;
  ss_ = sf_ = sd_ = st_ = fs_ = ff_ = fd_ = ft_ = ds_ = df_ = dd_ = dt_ = ts_ = tf_ = td_ = tt_ = 0;
}

double TemplateDissimilarity::getMinOverlay() const
{
  const double sols_first = sd_ + sf_ + ss_ + st_;
  const double sols_second = ds_ + fs_ + ss_ + ts_;

  if (sols_first < 0.001 || sols_second < 0.001)
  {
    return 0.001;
  }
  const double ratio_first = static_cast<double> (ss_) / sols_first;
  const double ratio_second = static_cast<double> (ss_) / sols_second;

  double min_ratio = std::min(ratio_first, ratio_second);

  if (min_ratio < 0.001)
  {
    min_ratio = 0.001;
  }

  return min_ratio;
}

double TemplateDissimilarity::getRegionOverlay(unsigned int type) const
{
  const double N = TemplateHeightmap::TH_DEFAULT_NUM_TILES_X * TemplateHeightmap::TH_DEFAULT_NUM_TILES_X;

	double num_first, num_second, overlay;
	switch (type)
	{
	case TS_SOLID:
	  num_first = static_cast<double>(sf_+sd_+st_);
	  num_second = static_cast<double>(fs_+ds_+ts_);
	  overlay = static_cast<double>(ss_);
	break;

	case TS_DONTCARE:
	  num_first = static_cast<double>(ds_+df_+dt_);
	  num_second = static_cast<double>(sd_+fd_+td_);
	  overlay = static_cast<double>(dd_);
	break;

	case TS_FOG:
	  num_first = static_cast<double>(fs_+fd_+ft_);
	  num_second = static_cast<double>(sf_+df_+tf_);
	  overlay = static_cast<double>(ff_);
	break;

	case TS_TABLE:
	  num_first = static_cast<double>(ts_+tf_+td_);
	  num_second = static_cast<double>(st_+ft_+dt_);
	  overlay = static_cast<double>(tt_);
	break;

	default:
	break;
	}

//	double ret = clamping_fac_ + 1.0;

	double ret = N;

	if(overlay != 0.0)
	  ret = std::max(num_first, num_second);

	return ret / N;  //0 < ret <= 1
}

//double TemplateDissimilarity::getScore() const
//{
//	double occ_pun = 1;//(1 + ff_/TemplateHeightmap::TH_DEFAULT_NUM_TILES_X/TemplateHeightmap::TH_DEFAULT_NUM_TILES_Y);
//  return occ_pun*occ_pun * distances_sum_ / relevants_ / getMinOverlay();
//}

double TemplateDissimilarity::getScore() const
{
  const double N = TemplateHeightmap::TH_DEFAULT_NUM_TILES_X * TemplateHeightmap::TH_DEFAULT_NUM_TILES_X;

  double weighted_overlays_normed = 2.0*getRegionOverlay(TS_SOLID) + 1.0*getRegionOverlay(TS_DONTCARE) +
		  0.0*getRegionOverlay(TS_FOG) + 1.0*getRegionOverlay(TS_TABLE);

  double dist_sum_normed = 0.0;
  double dist_s_normalizer = (N - dd_)*max_dist_;

  if(dist_s_normalizer > 0.000000001)
	  dist_sum_normed = (1 / dist_s_normalizer) * 500.0 * distances_sum_;

  double ret = weighted_overlays_normed + dist_sum_normed;

//  std::cout << ret << "\t"<< getRegionOverlay(TS_SOLID) << "\t"<<
//		  getRegionOverlay(TS_DONTCARE) << "\t"<< getRegionOverlay(TS_TABLE) <<
//		  "\t"<< dist_sum_normed << std::endl;

  return ret;
}

bool TemplateDissimilarity::operator()(const TemplateDissimilarity& first, const TemplateDissimilarity& second)
{
  return isBetter(first, second);
}

bool TemplateDissimilarity::isBetter(const TemplateDissimilarity& first, const TemplateDissimilarity& second)
{
	return first.getScore() < second.getScore();
}

/* DismatchMeasure definitions */

DismatchMeasure::DismatchMeasure(const GraspTemplate& templt, const geometry_msgs::Pose& gripper_pose) :
  lib_template_(templt)
{
  constructClass(gripper_pose);
}

DismatchMeasure::DismatchMeasure(const Heightmap& hm, const geometry_msgs::Pose& templt_pose,
                                 const geometry_msgs::Pose& gripper_pose) :
  lib_template_(hm, templt_pose)
{
  constructClass(gripper_pose);
}

TemplateDissimilarity DismatchMeasure::getScore(const GraspTemplate& sample) const
{
  return getScore(sample, lib_template_);
}

//TemplateDissimilarity DismatchMeasure::getScore(const GraspTemplate& sample, const GraspTemplate& lib_templt) const
//{
//  HeightmapDifference diff(sample.heightmap_, lib_templt.heightmap_);
//  TemplateDissimilarity score;
//  score.max_dist_ = max_dist_;
//  fillStateStat(diff, score);
//
//  for (unsigned int i = 0; i < diff.diff_.size(); i++)
//  {
//    const double val = abs(diff.diff_[i]);
//    double scr = -1;
//
//    switch (diff.states_[i].first)
//    {
//      case TS_SOLID:
//        switch (diff.states_[i].second)
//        {
//          case TS_SOLID:
//            scr = weights_[0][0] * val;
//            break;
//          case TS_DONTCARE:
//            scr = weights_[0][1] * val;
//            break;
//          case TS_FOG:
//            scr = weights_[0][2] * val;
//            break;
//          case TS_TABLE:
//            scr = weights_[0][3] * val;
//            break;
//          default:
//            break;
//        }
//        break;
//
//      case TS_DONTCARE:
//        switch (diff.states_[i].second)
//        {
//          case TS_SOLID:
//            scr = weights_[1][0] * val;
//            break;
//          case TS_DONTCARE:
//            scr = weights_[1][1] * val;
//            break;
//          case TS_FOG:
//            scr = weights_[1][2] * val;
//            break;
//          case TS_TABLE:
//            scr = weights_[1][3] * val;
//            break;
//          default:
//            break;
//        }
//        break;
//
//        case TS_FOG:
//          switch (diff.states_[i].second)
//          {
//            case TS_SOLID:
//              scr = weights_[2][0] * val;
//              break;
//            case TS_DONTCARE:
//              scr = weights_[2][1] * val;
//              break;
//            case TS_FOG:
//              scr = weights_[2][2] * val;
//              break;
//            case TS_TABLE:
//              scr = weights_[2][3] * val;
//              break;
//            default:
//              break;
//          }
//          break;
//
//      case TS_TABLE:
//        switch (diff.states_[i].second)
//        {
//          case TS_SOLID:
//            scr = weights_[3][0] * val;
//            break;
//          case TS_DONTCARE:
//            scr = weights_[3][1] * val;
//            break;
//          case TS_FOG:
//            scr = weights_[3][2] * val;
//            break;
//          case TS_TABLE:
//            scr = weights_[3][3] * val;
//            break;
//          default:
//            break;
//        }
//        break;
//
//      default:
//        break;
//    }
//
//    score.distances_sum_ += scr;
//    score.relevants_ += 1;
//
//  }
//
//  return score;
//}

TemplateDissimilarity DismatchMeasure::getScore(const GraspTemplate& sample, const GraspTemplate& lib_templt) const
{
  HeightmapDifference diff(sample.heightmap_, lib_templt.heightmap_);
  TemplateDissimilarity score;
  score.max_dist_ = max_dist_;
  fillStateStat(diff, score);

  for (unsigned int i = 0; i < diff.diff_.size(); i++)
  {
    const double val = abs(diff.diff_[i]);
    double scr = -1;

    switch (diff.states_[i].first)
    {
      case TS_SOLID:
        switch (diff.states_[i].second)
        {
          case TS_SOLID:
            scr = weights_[0][0] * val;
            break;
          case TS_DONTCARE:
            scr = weights_[0][1] * val;
            break;
          case TS_FOG:
            scr = weights_[0][2] * val;
            break;
          case TS_TABLE:
            scr = weights_[0][3] * val;
            break;
          default:
            break;
        }
        break;

      case TS_DONTCARE:
        switch (diff.states_[i].second)
        {
          case TS_SOLID:
            scr = weights_[1][0] * val;
            break;
          case TS_DONTCARE:
            scr = weights_[1][1] * val;
            break;
          case TS_FOG:
            scr = weights_[1][2] * val;
            break;
          case TS_TABLE:
            scr = weights_[1][3] * val;
            break;
          default:
            break;
        }
        break;

        case TS_FOG:
          switch (diff.states_[i].second)
          {
            case TS_SOLID:
              scr = weights_[2][0] * val;
              break;
            case TS_DONTCARE:
              scr = weights_[2][1] * val;
              break;
            case TS_FOG:
              scr = weights_[2][2] * val;
              break;
            case TS_TABLE:
              scr = weights_[2][3] * val;
              break;
            default:
              break;
          }
          break;

      case TS_TABLE:
        switch (diff.states_[i].second)
        {
          case TS_SOLID:
            scr = weights_[3][0] * val;
            break;
          case TS_DONTCARE:
            scr = weights_[3][1] * val;
            break;
          case TS_FOG:
            scr = weights_[3][2] * val;
            break;
          case TS_TABLE:
            scr = weights_[3][3] * val;
            break;
          default:
            break;
        }
        break;

      default:
        break;
    }

    score.distances_sum_ += scr;
    score.relevants_ += 1;

  }

  return score;
}

void DismatchMeasure::applyDcMask(GraspTemplate& templt) const
{
  const double tile_length_x = templt.heightmap_.getMapLengthX() / templt.heightmap_.getNumTilesX();
  const double tile_length_y = templt.heightmap_.getMapLengthY() / templt.heightmap_.getNumTilesY();
  const double x0 = -templt.heightmap_.getMapLengthX() / 2.0 + tile_length_x / 2.0;
  const double y0 = -templt.heightmap_.getMapLengthY() / 2.0 + tile_length_y / 2.0;

  for (unsigned int ix = 0; ix < templt.heightmap_.getNumTilesX(); ix++)
  {
    for (unsigned int iy = 0; iy < templt.heightmap_.getNumTilesY(); iy++)
    {
      const double x = x0 + ix * tile_length_x;
      const double y = y0 + iy * tile_length_y;

      const double cur = templt.heightmap_.getGridTile(ix, iy);

      if (cur < mask_[ix][iy] || templt.heightmap_.isEmpty(cur) || cur > 0.1)
        templt.heightmap_.setGridTileDontCare(x, y, mask_[ix][iy]);
    }
  }
}

void DismatchMeasure::fillStateStat(const HeightmapDifference& diff, TemplateDissimilarity& score) const
{
  for (unsigned int i = 0; i < diff.diff_.size(); i++)
  {
    switch (diff.states_[i].first)
    {
      case TS_SOLID:
        switch (diff.states_[i].second)
        {
          case TS_SOLID:
            score.ss_++;
            break;
          case TS_FOG:
            score.sf_++;
            break;
          case TS_DONTCARE:
            score.sd_++;
            break;
          case TS_TABLE:
            score.st_++;
            break;
          default:
            break;
        }
        break;

      case TS_FOG:
        switch (diff.states_[i].second)
        {
          case TS_SOLID:
            score.fs_++;
            break;
          case TS_FOG:
            score.ff_++;
            break;
          case TS_DONTCARE:
            score.fd_++;
            break;
          case TS_TABLE:
            score.ft_++;
            break;
          default:
            break;
        }
        break;

      case TS_DONTCARE:
        switch (diff.states_[i].second)
        {
          case TS_SOLID:
            score.ds_++;
            break;
          case TS_FOG:
            score.df_++;
            break;
          case TS_DONTCARE:
            score.dd_++;
            break;
          case TS_TABLE:
            score.dt_++;
            break;
          default:
            break;
        }
        break;

      case TS_TABLE:
        switch (diff.states_[i].second)
        {
          case TS_SOLID:
            score.ts_++;
            break;
          case TS_FOG:
            score.tf_++;
            break;
          case TS_DONTCARE:
            score.td_++;
            break;
          case TS_TABLE:
            score.tt_++;
            break;
          default:
            break;
        }
        break;

      default:
        break;
    } //main switch
  } //for loop
}

void DismatchMeasure::computeMask(std::vector<std::vector<double> >& mask) const
{
  Eigen::Vector3d gripper_to_world_t(lib_template_gripper_pose_.position.x, lib_template_gripper_pose_.position.y,
                                     lib_template_gripper_pose_.position.z);
  Eigen::Quaterniond gripper_to_world_r(lib_template_gripper_pose_.orientation.w,
                                        lib_template_gripper_pose_.orientation.x,
                                        lib_template_gripper_pose_.orientation.y,
                                        lib_template_gripper_pose_.orientation.z);

  Vector3d e1, e2;
  actuatorBoundingBox(e1, e2);
  Vector3d ex(1, 0, 0), ey(0, 1, 0), ez(0, 0, 1);

  double dx = abs(e2.x() - e1.x());
  double dy = abs(e2.y() - e1.y());
  double dz = abs(e2.z() - e1.z());

  ex = dx * ex;
  ey = dy * ey;
  ez = dz * ez;

  e1 = gripper_to_world_r * e1 + gripper_to_world_t;
  e1 = lib_template_.object_to_template_rotation_.inverse() * (e1 - lib_template_.object_to_template_translation_);
  e2 = gripper_to_world_r * e2 + gripper_to_world_t;
  e2 = lib_template_.object_to_template_rotation_.inverse() * (e2 - lib_template_.object_to_template_translation_);

  ex = gripper_to_world_r * ex;
  ex = lib_template_.object_to_template_rotation_.inverse() * ex;
  ey = gripper_to_world_r * ey;
  ey = lib_template_.object_to_template_rotation_.inverse() * ey;
  ez = gripper_to_world_r * ez;
  ez = lib_template_.object_to_template_rotation_.inverse() * ez;

  double min_z = numeric_limits<double>::max();
  e2.z() - ey.z() < min_z ? min_z = e2.z() - ey.z() : min_z;
  e2.z() - ez.z() < min_z ? min_z = e2.z() - ez.z() : min_z;

  mask.resize(lib_template_.heightmap_.getNumTilesX());
  for (unsigned int ix = 0; ix < lib_template_.heightmap_.getNumTilesX(); ix++)
  {
    mask[ix].resize(lib_template_.heightmap_.getNumTilesY());
    for (unsigned int iy = 0; iy < lib_template_.heightmap_.getNumTilesY(); iy++)
    {
      mask[ix][iy] = 0;
    }
  }

  planeToMask(e1, ex, ey, mask); //broad side
  //	planeToMask(e1, ez, ey, mask);	//top
  planeToMask(e1, ez, ex, mask); //small side
  //
  planeToMask(e2, -ex, -ey, mask);
  planeToMask(e2, -ey, -ez, mask);
  planeToMask(e2, -ez, -ex, mask);

}

void DismatchMeasure::maskTemplate()
{
  computeMask( mask_);
  applyDcMask( lib_template_);
}

void DismatchMeasure::planeToMask(const Eigen::Vector3d& p, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2,
                                  std::vector<std::vector<double> >& mask) const
{
  Eigen::Vector3d n = v1.cross(v2);
  n.normalize();
  const double tile_length_x = lib_template_.heightmap_.getMapLengthX() / lib_template_.heightmap_.getNumTilesX();
  const double tile_length_y = lib_template_.heightmap_.getMapLengthY() / lib_template_.heightmap_.getNumTilesY();
  const double x0 = -lib_template_.heightmap_.getMapLengthX() / 2.0 + tile_length_x / 2.0;
  const double y0 = -lib_template_.heightmap_.getMapLengthY() / 2.0 + tile_length_y / 2.0;

  if (abs(n.z()) < 0.000001)
  {
    return;
  }

  const double v1_norm_sqr = v1.norm() * v1.norm();
  const double v2_norm_sqr = v2.norm() * v2.norm();

  for (unsigned int ix = 0; ix < lib_template_.heightmap_.getNumTilesX(); ix++)
  {
    for (unsigned int iy = 0; iy < lib_template_.heightmap_.getNumTilesY(); iy++)
    {
      const double x = x0 + ix * tile_length_x;
      const double y = y0 + iy * tile_length_y;

      double z = (n.dot(p) - (x * n.x() + y * n.y())) / n.z();
      Eigen::Vector3d t(x, y, z);
      t = t - p;
      const double t_proj1 = t.dot(v1) / v1_norm_sqr;
      const double t_proj2 = t.dot(v2) / v2_norm_sqr;
      if (t_proj1 >= 0 && t_proj1 <= 1 && t_proj2 >= 0 && t_proj2 <= 1)
      {
        if (mask[ix][iy] > z)
          mask[ix][iy] = z;
      }
    } //inner for loop
  } //outer for loop
}

//void DismatchMeasure::constructClass(const geometry_msgs::Pose& gripper_pose)
//{
//  lib_template_gripper_pose_ = gripper_pose;
//  maskTemplate();
//
//  //apply bounding box cut offs
//  max_dist_ = 0;
//  for (unsigned int i = 0; i < mask_.size(); i++)
//  {
//    for (unsigned int j = 0; j < mask_[i].size(); j++)
//    {
//      max_dist_ += abs(mask_[i][j]);
//    }
//  }
//
//  //set solid-void occlusion weights
//  weights_.resize(4);
//  for (unsigned int i = 0; i < 4; i++)
//  {
//    weights_[i].resize(4);
//  }
//  weights_[0][0] = weights_[0][1] = weights_[0][2] = weights_[0][3] = weights_[1][0] = 50;
//  weights_[1][1] = weights_[1][2] = weights_[1][3] = 12;
//  weights_[2][0] = 50;
//  weights_[2][1] = weights_[2][2] = weights_[2][3] = 12;
//  weights_[3][0] = 50;
//  weights_[3][1] = weights_[3][2] = weights_[3][3] = 12;
//}

void DismatchMeasure::constructClass(const geometry_msgs::Pose& gripper_pose)
{
  lib_template_gripper_pose_ = gripper_pose;
  maskTemplate();

  //apply bounding box cut offs
  max_dist_ = 0;
  for (unsigned int i = 0; i < mask_.size(); i++)
  {
    for (unsigned int j = 0; j < mask_[i].size(); j++)
    {
      max_dist_ += abs(mask_[i][j]);
    }
  }

  //set solid-void occlusion weights
  weights_.resize(4);
  for (unsigned int i = 0; i < 4; i++)
  {
    weights_[i].resize(4);
  }
  weights_[0][0] = 1;
  weights_[0][1] = weights_[0][2] = weights_[0][3] = weights_[1][0] = 1;
  weights_[1][1] = weights_[1][2] = weights_[1][3] = 1;
  weights_[2][0] = 1;
  weights_[2][1] = weights_[2][2] = weights_[2][3] = 1;
  weights_[3][0] = 1;
  weights_[3][1] = weights_[3][2] = weights_[3][3] = 1;
}
}
