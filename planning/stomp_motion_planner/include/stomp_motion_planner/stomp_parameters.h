/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#ifndef STOMP_PARAMETERS_H_
#define STOMP_PARAMETERS_H_

#include <ros/ros.h>

namespace stomp_motion_planner
{

class StompParameters
{
public:
  StompParameters();
  virtual ~StompParameters();

  void initFromNodeHandle();

  double getPlanningTimeLimit() const;
  void setPlanningTimeLimit(double planning_time_limit);
  int getMaxIterations() const;
  int getMaxIterationsAfterCollisionFree() const;
  double getSmoothnessCostWeight() const;
  double getObstacleCostWeight() const;
  double getConstraintCostWeight() const;
  double getStateValidityCostWeight() const;
  double getEndeffectorVelocityCostWeight() const;
  double getTorqueCostWeight() const;
  bool getAnimatePath() const;
  double getLearningRate() const;
  double getSmoothnessCostVelocity() const;
  double getSmoothnessCostAcceleration() const;
  double getSmoothnessCostJerk() const;
  std::vector<double> getSmoothnessCosts() const;
  bool getAddRandomness() const;
  bool getUseHamiltonianMonteCarlo() const;
  double getHmcDiscretization() const;
  double getHmcStochasticity() const;
  double getHmcAnnealingFactor() const;
  double getRidgeFactor() const;
  bool getUsePseudoInverse() const;
  double getPseudoInverseRidgeFactor() const;
  bool getAnimateEndeffector() const;
  std::string getAnimateEndeffectorSegment() const;
  bool getUseChomp() const;

private:
  double planning_time_limit_;
  int max_iterations_;
  int max_iterations_after_collision_free_;
  double smoothness_cost_weight_;
  double obstacle_cost_weight_;
  double constraint_cost_weight_;
  double state_validity_cost_weight_;
  double torque_cost_weight_;
  double endeffector_velocity_cost_weight_;
  double learning_rate_;
  bool animate_path_;
  double smoothness_cost_velocity_;
  double smoothness_cost_acceleration_;
  double smoothness_cost_jerk_;
  bool add_randomness_;
  bool use_hamiltonian_monte_carlo_;
  double hmc_stochasticity_;
  double hmc_discretization_;
  double hmc_annealing_factor_;
  double ridge_factor_;
  bool use_pseudo_inverse_;
  double pseudo_inverse_ridge_factor_;
  bool animate_endeffector_;
  std::string animate_endeffector_segment_;
  bool use_chomp_;

};

/////////////////////// inline functions follow ////////////////////////

inline double StompParameters::getPlanningTimeLimit() const
{
  return planning_time_limit_;
}

inline void StompParameters::setPlanningTimeLimit(double planning_time_limit)
{
  planning_time_limit_ = planning_time_limit;
}

inline int StompParameters::getMaxIterations() const
{
  return max_iterations_;
}

inline int StompParameters::getMaxIterationsAfterCollisionFree() const
{
  return max_iterations_after_collision_free_;
}

inline double StompParameters::getSmoothnessCostWeight() const
{
  return smoothness_cost_weight_;
}

inline double StompParameters::getObstacleCostWeight() const
{
  return obstacle_cost_weight_;
}

inline double StompParameters::getConstraintCostWeight() const
{
  return constraint_cost_weight_;
}

inline double StompParameters::getStateValidityCostWeight() const
{
  return state_validity_cost_weight_;
}

inline double StompParameters::getEndeffectorVelocityCostWeight() const
{
  return endeffector_velocity_cost_weight_;
}

inline double StompParameters::getTorqueCostWeight() const
{
  return torque_cost_weight_;
}

inline double StompParameters::getLearningRate() const
{
  return learning_rate_;
}

inline bool StompParameters::getAnimatePath() const
{
  return animate_path_;
}

inline bool StompParameters::getAddRandomness() const
{
  return add_randomness_;
}

inline double StompParameters::getSmoothnessCostVelocity() const
{
  return smoothness_cost_velocity_;
}

inline double StompParameters::getSmoothnessCostAcceleration() const
{
  return smoothness_cost_acceleration_;
}

inline double StompParameters::getSmoothnessCostJerk() const
{
  return smoothness_cost_jerk_;
}

inline double StompParameters::getHmcDiscretization() const
{
  return hmc_discretization_;
}

inline double StompParameters::getHmcStochasticity() const
{
  return hmc_stochasticity_;
}

inline double StompParameters::getHmcAnnealingFactor() const
{
  return hmc_annealing_factor_;
}

inline bool StompParameters::getUseHamiltonianMonteCarlo() const
{
  return use_hamiltonian_monte_carlo_;
}

inline double StompParameters::getRidgeFactor() const
{
  return ridge_factor_;
}

inline bool StompParameters::getUsePseudoInverse() const
{
  return use_pseudo_inverse_;
}

inline double StompParameters::getPseudoInverseRidgeFactor() const
{
  return pseudo_inverse_ridge_factor_;
}

inline bool StompParameters::getAnimateEndeffector() const
{
  return animate_endeffector_;
}

inline std::string StompParameters::getAnimateEndeffectorSegment() const
{
  return animate_endeffector_segment_;
}

inline std::vector<double> StompParameters::getSmoothnessCosts() const
{
  std::vector<double> ret(3);
  ret[0] = smoothness_cost_velocity_;
  ret[1] = smoothness_cost_acceleration_;
  ret[2] = smoothness_cost_jerk_;
  return ret;
}

inline bool StompParameters::getUseChomp() const
{
  return use_chomp_;
}


} // namespace stomp

#endif /* STOMP_PARAMETERS_H_ */
