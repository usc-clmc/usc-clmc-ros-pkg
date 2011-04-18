/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal 
 *********************************************************************
 \remarks		...
 
 \file		nc2010_transformation_system.cpp

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 6, 2010

 *********************************************************************/

// system includes
#include <stdio.h>
#include <math.h>
#include <cassert>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

// local includes
#include <dmp_lib/nc2010_transformation_system.h>
#include <dmp_lib/logger.h>
#include <dmp_lib/quaternion_utilities.h>
#include <dmp_lib/utilities.h>

using namespace std;
using namespace Eigen;

namespace dmp_lib
{

NC2010TransformationSystem& NC2010TransformationSystem::operator=(const NC2010TransformationSystem& nc2010ts)
{
  Logger::logPrintf("NC2010TransformationSystem assignment.", Logger::DEBUG);

  // first assign all memeber variables
  assert(Utilities<NC2010TSParam>::assign(parameters_, nc2010ts.parameters_));
  assert(Utilities<NC2010TSState>::assign(states_, nc2010ts.states_));

  // then assign all base class variables
  TransformationSystem::parameters_.clear();
  for (int i = 0; i < (int)parameters_.size(); ++i)
  {
    TransformationSystem::parameters_.push_back(parameters_[i]);
  }
  TransformationSystem::states_.clear();
  for (int i = 0; i < (int)states_.size(); ++i)
  {
    TransformationSystem::states_.push_back(states_[i]);
  }
  integration_method_ = nc2010ts.integration_method_;
  initialized_ = nc2010ts.initialized_;
  return *this;
}

bool NC2010TransformationSystem::initialize(const NC2010TSParamPtr parameters,
                                              const NC2010TSStatePtr state,
                                              IntegrationMethod integration_method)
{
  vector<NC2010TSParamPtr> vec_parameters;
  vec_parameters.push_back(parameters);
  vector<NC2010TSStatePtr> vec_states;
  vec_states.push_back(state);
  return initialize(vec_parameters, vec_states, integration_method);
}

bool NC2010TransformationSystem::initialize(const vector<NC2010TSParamPtr> parameters,
                                              const vector<NC2010TSStatePtr> states,
                                              IntegrationMethod integration_method)
{
  Logger::logPrintf("Initializing NC2010 transformation system.", Logger::DEBUG);

  // first initialize all memeber variables
  assert(Utilities<NC2010TSParam>::assign(parameters_, parameters));
  assert(Utilities<NC2010TSState>::assign(states_, states));

  // then initialize base class
  vector<TSParamPtr> ts_params;
  for (int i = 0; i < (int)parameters_.size(); ++i)
  {
    ts_params.push_back(parameters_[i]);
  }
  vector<TSStatePtr> ts_states;
  for (int i = 0; i < (int)states_.size(); ++i)
  {
    ts_states.push_back(states_[i]);
  }
  return TransformationSystem::initialize(ts_params, ts_states, integration_method);
}

bool NC2010TransformationSystem::get(NC2010TSParamConstPtr& parameters,
                                       NC2010TSStateConstPtr& state) const
{
  if(getNumDimensions() != 1)
  {
    Logger::logPrintf("Number of dimensions >%i< is not equal 1. Cannot get parameters and state.", Logger::ERROR, getNumDimensions());
    return false;
  }
  parameters = parameters_[0];
  state = states_[0];
  return true;
}

bool NC2010TransformationSystem::get(vector<NC2010TSParamConstPtr>& parameters,
                                       vector<NC2010TSStateConstPtr>& states) const
{
  if (!initialized_)
  {
    Logger::logPrintf("NC2010 Transformation system is not initialized. Cannot return parameters and states.", Logger::ERROR);
    return false;
  }
  parameters.clear();
  states.clear();
  for (int i = 0; i < getNumDimensions(); ++i)
  {
    parameters.push_back(parameters_[i]);
    states.push_back(states_[i]);
  }
  return true;
}

void NC2010TransformationSystem::reset()
{
    assert(initialized_);
    for(int i=0; i<getNumDimensions(); ++i)
    {
      states_[i]->reset();
    }
}

bool NC2010TransformationSystem::integrateAndFit(const vector<State>& target_states,
                                                   const CSStatePtr canonical_system_state,
                                                   const Time& dmp_time)
{
  assert(initialized_);
  switch (integration_method_)
  {
    case NORMAL:
    {
      for (int i = 0; i < getNumDimensions(); ++i)
      {
        // set target state
        states_[i]->target_ = target_states[i];

        // compute nonlinear target function
        states_[i]->ft_ = ((states_[i]->target_.getXdd() * pow(dmp_time.getTau(), 2) + parameters_[i]->d_gain_ * states_[i]->target_.getXd()
            * dmp_time.getTau()) / parameters_[i]->k_gain_) - (states_[i]->goal_ - states_[i]->target_.getX()) + (states_[i]->goal_ - states_[i]->start_)
            * canonical_system_state->getStateX();

        // the nonlinearity is computed by LWR (later)
        states_[i]->function_input_.push_back(canonical_system_state->getStateX());
        states_[i]->function_target_.push_back(states_[i]->ft_ / canonical_system_state->getStateX());

        // compute transformation system (make use of target knowledge)
        states_[i]->internal_.setXdd((parameters_[i]->k_gain_ * (states_[i]->goal_ - states_[i]->current_.getX()) - parameters_[i]->d_gain_
            * states_[i]->internal_.getXd() - parameters_[i]->k_gain_ * (states_[i]->goal_ - states_[i]->start_) * canonical_system_state->getStateX()
            + parameters_[i]->k_gain_ * states_[i]->ft_) / dmp_time.getTau());

        states_[i]->current_.setXd(states_[i]->internal_.getXd() / dmp_time.getTau());
        states_[i]->current_.setXdd(states_[i]->internal_.getXdd() / dmp_time.getTau());

        // integrate the system twice
        states_[i]->internal_.addXd(states_[i]->getInternalStateXdd() * dmp_time.getDeltaT());
        states_[i]->current_.addX(states_[i]->getCurrentStateXd() * dmp_time.getDeltaT());
      }
      break;
    }
    case QUATERNION:
    {

      if (target_states.size() != 4)
      {
        Logger::logPrintf("Cannot integrate and fit quaternion with only >%i< target states.", Logger::ERROR, (int)target_states.size());
        return false;
      }

      Vector4d target_quat, target_quatd, target_quatdd;
      for (int i = 0; i < 4; ++i)
      {
        states_[i]->target_ = target_states[i];
        target_quat(i) = states_[i]->target_.getX();
        target_quatd(i) = states_[i]->target_.getXd();
        target_quatdd(i) = states_[i]->target_.getXdd();
      }

      Vector3d target_angular_velocity;
      getAngularVelocity(target_quat, target_quatd, target_angular_velocity);

      Vector3d target_angular_acceleration;
      getAngularAccelerations(target_quat, target_quatd, target_quatdd, target_angular_acceleration);

      // for debugging
      for (int i = 1; i < 4; ++i)
      {
        states_[i]->target_.setXd(target_angular_velocity(i - 1));
        states_[i]->target_.setXdd(target_angular_acceleration(i - 1));
      }

      Vector3d current_angular_velocity = Vector3d::Zero();
      Vector3d current_angular_acceleration = Vector3d::Zero();
      Vector3d internal_angular_velocity = Vector3d::Zero();
      Vector3d internal_angular_acceleration = Vector3d::Zero();
      for (int i = 1; i < 4; ++i)
      {
        current_angular_velocity(i - 1) = states_[i]->current_.getXd();
        current_angular_acceleration(i - 1) = states_[i]->current_.getXdd();
        internal_angular_velocity(i - 1) = states_[i]->internal_.getXd();
        internal_angular_acceleration(i - 1) = states_[i]->internal_.getXdd();
      }

      Vector4d current_quat, start_quat, goal_quat;
      for (int i = 0; i < 4; ++i)
      {
        current_quat(i) = states_[i]->current_.getX();
        start_quat(i) = states_[i]->start_;
        goal_quat(i) = states_[i]->goal_;
      }

      Vector3d current_goal_scaling;
      getQuatError(current_quat, goal_quat, current_goal_scaling);

      Vector3d static_goal_scaling;
      getQuatError(start_quat, goal_quat, static_goal_scaling);

      Matrix3d K = Matrix3d::Zero();
      Matrix3d D = Matrix3d::Zero();
      for (int i = 0; i < 3; ++i)
      {
        K(i, i) = parameters_[i + 1]->k_gain_;
        D(i, i) = parameters_[i + 1]->d_gain_;
      }

      Vector3d ft;
      ft = K.inverse() * (target_angular_acceleration * pow(dmp_time.getTau(), 2) + D * target_angular_velocity * dmp_time.getTau()) - current_goal_scaling
          + static_goal_scaling * canonical_system_state->getStateX();

      for (int i = 1; i < 4; ++i)
      {
        states_[i]->ft_ = ft(i - 1);

        // the nonlinearity is computed by LWR (later)
        states_[i]->function_input_.push_back(canonical_system_state->getStateX());
        states_[i]->function_target_.push_back(states_[i]->ft_ / canonical_system_state->getStateX());
      }

      // transformation state derivatives (make use of target knowledge)
      internal_angular_acceleration = (K * current_goal_scaling
          - D * internal_angular_velocity
          - K * static_goal_scaling * canonical_system_state->getStateX()
          + K * ft) / dmp_time.getTau();

      current_angular_velocity = internal_angular_velocity / dmp_time.getTau();
      current_angular_acceleration = internal_angular_acceleration / dmp_time.getTau();

      // integrate the system
      internal_angular_velocity = internal_angular_velocity + internal_angular_acceleration * dmp_time.getDeltaT();
      Vector4d current_quatd;
      getQuaternionVelocity(current_quat, current_angular_velocity, current_quatd);
      current_quat = current_quat + current_quatd * dmp_time.getDeltaT();
      current_quat.normalize();

      for (int i = 1; i < 4; ++i)
      {
        states_[i]->internal_.setXd(internal_angular_velocity(i - 1));
        states_[i]->internal_.setXdd(internal_angular_acceleration(i - 1));
        states_[i]->current_.setXd(current_angular_velocity(i - 1));
        states_[i]->current_.setXdd(current_angular_acceleration(i - 1));
      }
      for (int i = 0; i < 4; ++i)
      {
        states_[i]->current_.setX(current_quat(i));
      }

      break;
    }
  }
  return true;
}

// REAL-TIME REQUIREMENTS
// TODO: make the can ptr const
bool NC2010TransformationSystem::integrate(const CSStatePtr canonical_system_state,
                                             const Time& dmp_time,
                                             const VectorXd& feedback,
                                             const int num_iterations)
{
  assert(initialized_);
  switch(integration_method_)
  {
    case NORMAL:
    {
      double dt = dmp_time.getDeltaT() / static_cast<double> (num_iterations);
      for (int i = 0; i < getNumDimensions(); ++i)
      {
        for (int n = 0; n < num_iterations; ++n)
        {
          // compute nonlinearity using LWR
          double prediction = 0;
          if (!parameters_[i]->lwr_model_->predict(canonical_system_state->getStateX(), prediction))
          {
            Logger::logPrintf("Could not predict output (Real-time violation).", Logger::ERROR);
            return false;
          }

          states_[i]->f_ = prediction * canonical_system_state->getStateX();

          // compute transformation system
          states_[i]->internal_.setXdd( (parameters_[i]->k_gain_ * (states_[i]->goal_ - states_[i]->current_.getX())
              - parameters_[i]->d_gain_ * states_[i]->internal_.getXd()
              - parameters_[i]->k_gain_ * (states_[i]->goal_ - states_[i]->start_) * canonical_system_state->getStateX()
              + parameters_[i]->k_gain_ * states_[i]->f_ ) / dmp_time.getTau() );

          states_[i]->current_.setXd( states_[i]->internal_.getXd() / dmp_time.getTau() );
          states_[i]->current_.setXdd( states_[i]->internal_.getXdd() / dmp_time.getTau() );

          // integrate the system twice
          states_[i]->internal_.addXd(states_[i]->internal_.getXdd() * dt);
          states_[i]->current_.addX(states_[i]->current_.getXd() * dt);

        }
      }
      break;
    }
    case QUATERNION:
    {

      double dt = dmp_time.getDeltaT() / static_cast<double> (num_iterations);
      for (int n = 0; n < num_iterations; ++n)
      {
        // In this transformation system, quaternions and angular velocities/accelerations are
        // mixed. The 4D state is composed as follows:
        //
        //  state[i]   X             Xd           Xdd
        //        0    Q0 (scalar)   not used     not used
        //        1    Q1            angul. vel   angul. acc
        //        2    Q2            angul. vel   angul. acc
        //        3    Q3            angul. vel   angul. acc  
        
        // Get non-linear components 1..3
        Vector3d f;
        for (int i = 1; i < 4; ++i)
        {
          // compute nonlinearity using LWR
          double prediction = 0;
          if (!parameters_[i]->lwr_model_->predict(canonical_system_state->getStateX(), prediction))
          {
            Logger::logPrintf("Could not predict output (Real-time violation).", Logger::ERROR);
            return false;
          }
          f(i-1) = prediction * canonical_system_state->getStateX();
        }

        // Put k- and d-gain values in matrices, because all output dimensions will be computed at
        // once
        Matrix3d K = Matrix3d::Zero();
        Matrix3d D = Matrix3d::Zero();
        for (int i = 0; i < 3; ++i)
        {
          K(i,i) = parameters_[i+1]->k_gain_;
          D(i,i) = parameters_[i+1]->d_gain_;
        }

        // Get the angular velocities (Xd,Xdd) from states 1..3
        Vector3d current_angular_velocity = Vector3d::Zero();
        Vector3d current_angular_acceleration = Vector3d::Zero();
        Vector3d internal_angular_velocity = Vector3d::Zero();
        Vector3d internal_angular_acceleration = Vector3d::Zero();
        for (int i = 1; i < 4; ++i)
        {
          current_angular_velocity(i - 1) = states_[i]->current_.getXd();
          current_angular_acceleration(i - 1) = states_[i]->current_.getXdd();
          internal_angular_velocity(i - 1) = states_[i]->internal_.getXd();
          internal_angular_acceleration(i - 1) = states_[i]->internal_.getXdd();
        }

        // Get the quaterion (X) from states 0..3
        Vector4d current_quat, start_quat, goal_quat;
        for (int i = 0; i < 4; ++i)
        {
          current_quat(i) = states_[i]->current_.getX();
          start_quat(i) = states_[i]->start_;
          goal_quat(i) = states_[i]->goal_;
        }
        
        // Now comes the good part

        // Compute quaternion error. This is Trick 1: the error between the two 4D quaternions is a
        // 3D vector of angular velocities. This 3D vector is used to compute the angular
        // accelerations (integration part 1), as well as the next quaternion (integration part 2)
        Vector3d current_goal_scaling;
        getQuatError(current_quat, goal_quat, current_goal_scaling);

        Vector3d static_goal_scaling;
        getQuatError(start_quat, goal_quat, static_goal_scaling);

        // Integrate the transformation system (integration part 1)
        internal_angular_acceleration = (K * current_goal_scaling
            - D * internal_angular_velocity
            - K * static_goal_scaling * canonical_system_state->getStateX()
            + K * f) / dmp_time.getTau();

        current_angular_velocity = internal_angular_velocity / dmp_time.getTau();
        current_angular_acceleration = internal_angular_acceleration / dmp_time.getTau();

        // Trick 2: Use the 3D vector of angular velocities to compute the 4D quaternion velocity
        // Nice properties: quaternion remains normalized, and angular velocities and quaternion 
        // orientation are compatible. 
        internal_angular_velocity = internal_angular_velocity + internal_angular_acceleration * dt;
        Vector4d current_quatd;
        getQuaternionVelocity(current_quat, current_angular_velocity, current_quatd);
        // Integrate the quaternion (integration part 2)
        current_quat = current_quat + current_quatd * dt;
        current_quat.normalize();

        // Store the results, i.e.
        //   angular velocities/accelerations in Xd/Xdd of states 1..3
        //   quaternion in X of states 0..3
        for (int i = 1; i < 4; ++i)
        {
          states_[i]->f_ = f(i-1);
          states_[i]->internal_.setXd(internal_angular_velocity(i - 1));
          states_[i]->internal_.setXdd(internal_angular_acceleration(i - 1));
          states_[i]->current_.setXd(current_angular_velocity(i - 1));
          states_[i]->current_.setXdd(current_angular_acceleration(i - 1));
        }
        for (int i = 0; i < 4; ++i)
        {
          states_[i]->current_.setX(current_quat(i));
        }
      }
      break;
    }
  }

  return true;
}

}
