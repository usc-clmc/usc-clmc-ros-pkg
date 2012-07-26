/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         grasp_pool.h

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#ifndef GRASP_POOL_H_
#define GRASP_POOL_H_

#include <vector>
#include <iostream>

#include <grasp_template/dismatch_measure.h>
#include <grasp_template_planning/GraspAnalysis.h>

namespace grasp_template_planning
{

typedef std::map<grasp_template::TemplateDissimilarity, GraspAnalysis,
    grasp_template::TemplateDissimilarity> FailureScoreMap;

class GraspPool
{
public:

  GraspPool(){};

  std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > grasps_;
  std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > lib_grasps_;
  std::vector<grasp_template::TemplateDissimilarity> lib_scores_;
  std::vector<FailureScoreMap> failure_scores_;

  void reset();

private:

  std::vector<unsigned int> max_score_ranking_;
  std::vector<unsigned int> fail_avoidance_ranking_;
};

} //namespace
#endif /* GRASP_POOL_H_ */
