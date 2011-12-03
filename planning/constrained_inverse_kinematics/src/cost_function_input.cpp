/*
 * cost_function_input.cpp
 *
 *  Created on: Jul 27, 2011
 *      Author: kalakris
 */

#include <constrained_inverse_kinematics/cost_function_input.h>

namespace constrained_inverse_kinematics
{

CostFunctionInput::CostFunctionInput()
{
}

CostFunctionInput::~CostFunctionInput()
{
}

int CostFunctionInput::getNumDimensions() const
{
  return num_dimensions_;
}

}
