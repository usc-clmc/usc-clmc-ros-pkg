/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         extract_template.h

 \author       Daniel Kappler
 \date         July 30, 2013

 *********************************************************************/

#ifndef EXTRACT_TEMPLATE_H
#define EXTRACT_TEMPLATE_H

#include <Eigen/Eigen>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

typedef boost::mt19937 base_generator_type;

class Extract_template
{
private:
  boost::variate_generator<base_generator_type&, boost::uniform_real<> > _uni;
  Eigen::Vector3d _bounding_box_corner_1;
  Eigen::Vector3d _bounding_box_corner_2;

  Extract_template():
  _uni(generator(std::time(0),boost::uniform_real<> uni_dist(-1,1))),_bounding_box_corner_1(bounding_box_corner_1),
  {
  }
  ;Extract_template(Extract_template &extract_template):
  _uni(generator(std::time(0),boost::uniform_real<> uni_dist(-1,1))),_bounding_box_corner_1(bounding_box_corner_1),
  {
  }
  ;

  double _Get_sample_value(double scaling);
  Eigen::Quaterniond _Get_sample_orientation(Eigen::Quaterniond &base_orientation);

protected:
public:
  Extract_template(Eigen::Vector3d bounding_box_corner_1, Eigen::Vector3d bounding_box_corner_2);
  virtual ~Extract_template()
  {
  }
  ;


  void Get_random_grasp_templates(
      grasp_template::GraspTemplate &g_temp,
      std::vector<grasp_template::GraspTemplate, Eigen::aligned_allocator<grasp_template::GraspTemplate> > &result_template);

};

#endif /*EXTRACT_TEMPLATE_H*/
