/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		trajectory_simulator.h

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Jan 23, 2011

 *********************************************************************/

#ifndef CARTESIAN_TRAJECTORY_SIMULATOR_H_
#define CARTESIAN_TRAJECTORY_SIMULATOR_H_

// system includes
#include <dynamic_movement_primitive/dynamic_movement_primitive.h>

#include <inverse_kinematics/inverse_kinematics_with_nullspace_optimization.h>

// local includes

namespace SL
{

/*!
 */
template<class DMPType>
  class TrajectorySimulator
  {

  public:

    /*!
     */
    static double check(typename DMPType::DMPPtr dmp);

  private:

    /*!
     */
    TrajectorySimulator() {};

    /*!
     */
    virtual ~TrajectorySimulator() {};

    /*!
     * @param dmp
     */
    void prepareSimulatedControllerData(typename DMPType::DMPPtr dmp);

  };

  template<class DMPType>
    double TrajectorySimulator<DMPType>::check(typename DMPType::DMPPtr dmp)
  {

    return true;
  }

  template<class DMPType>
    void TrajectorySimulator<DMPType>::prepareSimulatedControllerData(typename DMPType::DMPPtr dmp)
  {

  }

}

#endif /* CARTESIAN_TRAJECTORY_SIMULATOR_H_ */
