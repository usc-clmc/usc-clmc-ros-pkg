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

#include <ctime>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

Extract_template::Extract_template(Eigen::Vector3d bounding_box_corner_1, Eigen::Vector3d bounding_box_corner_2)
 _uni(generator(std::time(0),boost::uniform_real<> uni_dist(0,1))),_bounding_box_corner_1(bounding_box_corner_1),
 _bounding_box_corner_2(bounding_box_corner_2)
{
  // have to rescale the random stuff


}

void Get_random_grasp_templates(
    grasp_template::GraspTemplate &g_temp,
    std::vector<grasp_template::GraspTemplate, Eigen::aligned_allocator<grasp_template::GraspTemplate> > &result_template)
{
  result_template.clear();

  double x, y, z;
  double roll, pitch, yaw;
  for (int i = 0; i < _max_samples; ++i)
  {
    gras_template::GraspTemplate local_grasp_template(g_temp);

    // sample templt_pose
    // this should be identity for now

    // sample gripper_pose
    // transform this in the template frame and then
    // sample from the applied grasps
    // this is required to actually compute the mask

    // sample x,y,z

    // sample roll,pitch,yaw

    // I can use roll pitch yaw since I will only sample small rotations
    // such that a gimbal lock will most likely not occur
    Eigen::Quaterniond sample_orientation;
    sample_orientation *= Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

    // heightmap is not important
    grasp_template::DismatchMeasure d_measure(local_grasp_template.heightmap_, identity_pose, sampled_gripper_pose);
    d_measure.applyDcMask(local_grasp_template);
    result_template.push_back(local_grasp_template);
  }
}
