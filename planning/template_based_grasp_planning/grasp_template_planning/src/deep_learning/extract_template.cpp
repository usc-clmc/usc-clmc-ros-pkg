/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         extract_template.cpp

 \author       Daniel Kappler
 \date         July 30, 2013

 *********************************************************************/

#include <deep_learning/extract_template.h>

Extract_template::Extract_template()
{

}
void Extract_random_grasp_templates(
    grasp_template::GraspTemplate &g_temp,
    std::vector<grasp_template::GraspTemplate, Eigen::aligned_allocator<grasp_template::GraspTemplate> > &result_template)
{
  result_template.clear();

  for (int i = 0; i < _max_samples; ++i)
  {
    gras_template::GraspTemplate local_grasp_template(g_temp);

    // sample templt_pose
    // this should be identity for now

    // sample gripper_pose
    // transform this in the template frame and then
    // sample from the applied grasps
    // this is required to actually compute the mask

    // heightmap is not important
    grasp_template::DismatchMeasure d_measure(local_grasp_template.heightmap_, identity_pose, sampled_gripper_pose);
    d_measure.applyDcMask(local_grasp_template);
    result_template.push_back(local_grasp_template);
  }
}
