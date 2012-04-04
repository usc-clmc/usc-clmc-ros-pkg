/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         grasp_pool.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <grasp_template_planning/grasp_pool.h>

using namespace std;
using namespace grasp_template;

namespace grasp_template_planning
{

void GraspPool::reset()
{
  grasps_.clear();
  lib_grasps_.clear();
  lib_scores_.clear();
  failure_scores_.clear();
}

} //namespace
