/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         dismatch_measure.h

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#ifndef DISMATCH_MEASURE_H
#define DISMATCH_MEASURE_H

#include <Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <grasp_template/heightmap_difference.h>
#include <grasp_template/grasp_template.h>
#include <grasp_template/grasp_template_params.h>

namespace grasp_template
{

struct TemplateDissimilarity
{
public:

  TemplateDissimilarity();

  unsigned int relevants_;
  unsigned int ss_, sf_, sd_, st_, fs_, ff_, fd_, ft_, ds_, df_, dd_, dt_, ts_, tf_, td_, tt_;
  double distances_sum_, max_dist_;
  double distance_sums_[TS_UNSET];

  double getMinOverlay() const;
  double getRegionOverlay(unsigned int type) const;
  double getScore() const;
  double getAllFog() const{return static_cast<double>(sf_ + ff_ + df_ + tf_ + fs_ + fd_ + ft_);};

  bool operator()(const TemplateDissimilarity& first, const TemplateDissimilarity& second);

  static bool isBetter(const TemplateDissimilarity& first, const TemplateDissimilarity& second);

private:
//  double clamping_fac_;
};

class DismatchMeasure : GraspTemplateParams
{
public:

  DismatchMeasure(const GraspTemplate& templt, const geometry_msgs::Pose& gripper_pose);
  DismatchMeasure(const Heightmap& hm, const geometry_msgs::Pose& templt_pose, const geometry_msgs::Pose& gripper_pose);

  const GraspTemplate& getLibTemplt() const { return lib_template_;};
  TemplateDissimilarity getScore(const GraspTemplate& sample) const;
  TemplateDissimilarity getScore(const GraspTemplate& sample, const GraspTemplate& lib_templt) const;
  void applyDcMask(GraspTemplate& templt) const;

private:

  GraspTemplate lib_template_;
  geometry_msgs::Pose lib_template_gripper_pose_;
  std::vector<std::vector<double> > mask_;
  double max_dist_;
  std::vector<std::vector<double> > weights_;

  void fillStateStat(const HeightmapDifference& diff, TemplateDissimilarity& score) const;
  void computeMask(std::vector<std::vector<double> >& mask) const;
  void maskTemplate();
  void planeToMask(const Eigen::Vector3d& p, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, std::vector<
      std::vector<double> >& mask) const;
  void constructClass(const geometry_msgs::Pose& gripper_pose);
};

} //namespace
#endif /* DISMATCH_MEASURE_H */
