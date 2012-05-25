/*
 * input.h
 *
 *  Created on: Jul 27, 2011
 *      Author: kalakris
 */

#ifndef LEARNABLE_COST_FUNCTION_INPUT_H_
#define LEARNABLE_COST_FUNCTION_INPUT_H_

namespace learnable_cost_function
{

/**
 * Input to a cost function. Must be able to save/load to disk.
 */
class Input
{
public:
  Input(){};
  virtual ~Input(){};

  virtual int getNumDimensions() const = 0; // number of input dimensions
  virtual double getTime() const { return 0.0; } // time is considered a special variable

};

}

#endif /* LEARNABLE_COST_FUNCTION_INPUT_H_ */
