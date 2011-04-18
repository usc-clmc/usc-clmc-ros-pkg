/*
 * kdl_conversions.h
 *
 *  Created on: Jan 21, 2011
 *      Author: kalakris
 */

#ifndef KDL_CONVERSIONS_H_
#define KDL_CONVERSIONS_H_

#include <vector>
#include <kdl/jntarray.hpp>

namespace robot_info
{

static inline void kdlToVector(const KDL::JntArray& kdl_array, std::vector<double>& array)
{
  array.resize(kdl_array.rows());
  for (unsigned int i=0; i<kdl_array.rows(); ++i)
  {
    array[i] = kdl_array(i);
  }
}

static inline void vectorToKdl(const std::vector<double>& array, KDL::JntArray& kdl_array)
{
  kdl_array.resize(array.size());
  for (unsigned int i=0; i<kdl_array.rows(); ++i)
  {
    kdl_array(i) = array[i];
  }
}

} // namespace robot_info

#endif /* KDL_CONVERSIONS_H_ */
