/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         grasp_creator_interface.h

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#ifndef GRASP_CREATOR_INTERFACE_H_
#define GRASP_CREATOR_INTERFACE_H_

#include <grasp_template/grasp_template.h>
#include <grasp_template_planning/GraspAnalysis.h>

namespace grasp_template_planning
{

class GraspCreatorInterface
{
public:

  virtual void createGrasp(const grasp_template::GraspTemplate& templt, const GraspAnalysis& lib_grasp,
      GraspAnalysis& result) const = 0;
};

} //namespace
#endif /* GRASP_CREATOR_INTERFACE_H_ */
