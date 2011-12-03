/*
 * cost_function_input.h
 *
 *  Created on: Jul 27, 2011
 *      Author: kalakris
 */

#ifndef COST_FUNCTION_INPUT_H_
#define COST_FUNCTION_INPUT_H_

#include <learnable_cost_function/input.h>
#include <constrained_inverse_kinematics/chain.h>

namespace constrained_inverse_kinematics
{

class CostFunctionInput: public learnable_cost_function::Input
{
public:
  CostFunctionInput();
  virtual ~CostFunctionInput();

  boost::shared_ptr<const Chain> chain_;
  KinematicsInfo kinematics_info_;
  KDL::JntArray joint_angles_;
  KDL::Frame tool_frame_;
  int num_dimensions_;

  virtual int getNumDimensions() const;
};

}

#endif /* COST_FUNCTION_INPUT_H_ */
