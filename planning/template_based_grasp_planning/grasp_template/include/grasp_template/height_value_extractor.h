/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks              ...

 \file         height_value_extractor.h

 \author       Alexander Herzog
 \date         April 1 2012

 *********************************************************************/

#ifndef HEIGHT_VALUE_EXTRACTOR_H_
#define HEIGHT_VALUE_EXTRACTOR_H_

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace grasp_template
{

class HeightValueExtractor
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HeightValueExtractor();

  Eigen::Transform<double, 3, Eigen::Affine> to_grid_transform_;
  Eigen::Matrix<double, 3, Eigen::Dynamic> points_;

  void getPointsInROI(std::vector<int>& indices) const;
  bool getFogHeight(double& fog_height);

  void initialize(const pcl::PointCloud<pcl::PointXYZ>& pc, const Eigen::Vector3d& trans, const Eigen::Quaterniond& rot);
  bool isPointInRegionOfInterest(const Eigen::Vector3d& p) const;
  bool isPointOccludingBin(const Eigen::Vector3d& p) const;

  /* set orientation always BEFORE bin */
  void resetBinOrientation(const Eigen::Quaterniond& orientation);
  bool resetBin(const Eigen::Vector3d& bin_center);

private:

  Eigen::Vector3d bin_center_;
  Eigen::Matrix3d bin_orientation_rot_;

  Eigen::Vector3d roi_normal_;
  Eigen::Vector3d sep_normal_;
  double sep_const_; //sep_normal_ * Xi == sep_const_, Xi in sep-plane
  double roi_threshold_;
};

} //namespace
#endif /* HEIGHT_VALUE_EXTRACTOR_H_ */
