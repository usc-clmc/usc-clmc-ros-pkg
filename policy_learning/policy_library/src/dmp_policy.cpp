/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

 \file    dmp_policy.cpp

 \author  Peter Pastor
 \date    Jun 10, 2010

 **********************************************************************/

// system includes
#include <assert.h>

// ros includes
#include <Eigen/Core>
#include <ros/console.h>

#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_utilities.h>

// import most common Eigen types
using namespace Eigen;

// local includes
#include <policy_library/dmp_policy.h>

namespace policy_library
{

bool DMPPolicy::initialize(dmp_lib::DMPPtr dmp)
{
  if (!dmp->isInitialized())
  {
    ROS_ERROR("DMP is not initialized.");
    return false;
  }
  dmp_ = dmp;
  return (initialized_ = true);
}

bool DMPPolicy::setNumTimeSteps(const int num_time_steps)
{
  if (!initialized_)
  {
    return false;
  }
  num_time_steps_ = num_time_steps;
  return true;
}

bool DMPPolicy::getNumTimeSteps(int& num_time_steps)
{
  if(num_time_steps_ < 0)
  {
    ROS_ERROR("Number of time steps >%i< has not been set yet.", num_time_steps_);
    return false;
  }
  num_time_steps = num_time_steps_;
  return true;
}

bool DMPPolicy::getNumDimensions(int& num_dimensions)
{
  if (!initialized_)
  {
    return false;
  }
  // TODO: When DMP includes a quaternion the number of dimension is actually less
  num_dimensions = dmp_->getNumDimensions();
  return true;
}

bool DMPPolicy::getNumParameters(std::vector<int>& num_params)
{
  if (!initialized_)
  {
    return false;
  }
  return dmp_->getNumRFS(num_params);
}

bool DMPPolicy::getBasisFunctions(std::vector<MatrixXd>& basis_functions)
{
  if (!initialized_)
  {
    return false;
  }
  return dmp_->generateBasisFunctionMatrix(num_time_steps_, basis_functions);
}

bool DMPPolicy::computeControlCosts(const std::vector<Eigen::MatrixXd>& control_cost_matrices,
                                    const std::vector<std::vector<Eigen::VectorXd> >& parameters,
                                    const double weight,
                                    std::vector<Eigen::VectorXd>& control_costs)
{
  int num_dimensions = control_cost_matrices.size();
  int num_time_steps = parameters[0].size();

  std::vector<int> num_parameters(num_dimensions);
  getNumParameters(num_parameters);

  // initialize output vector if necessary:
  if (int(control_costs.size()) != num_dimensions)
  {
    control_costs.clear();
    for (int d = 0; d < num_dimensions; ++d)
    {
      control_costs.push_back(VectorXd::Zero(num_time_steps));
    }
  }

  // compute the costs
  for (int d = 0; d < num_dimensions; ++d)
  {
    for (int t = 0; t < num_time_steps; ++t)
    {
      control_costs[d](t) = weight * parameters[d][t].dot(control_cost_matrices[d] * parameters[d][t]);
    }
  }

  return true;
}

bool DMPPolicy::getControlCosts(std::vector<MatrixXd>& control_costs)
{
  if (!initialized_)
  {
    return false;
  }

  control_costs.clear();
  std::vector<int > num_thetas;
  if (!getNumParameters(num_thetas))
  {
    ROS_ERROR("Could not get number of parameters.");
    return false;
  }

  std::vector<VectorXd> basis_function_centers;
  if(!dmp_->getBasisFunctionCenters(basis_function_centers))
  {
    ROS_ERROR("Could not get basis function centers, cannot compute control cost.");
    return false;
  }

  if(num_thetas.size() != basis_function_centers.size())
  {
    ROS_ERROR("Number of dimensions >%i< does not correspond to number of basis function vectors >%i<.", (int)num_thetas.size(), (int)basis_function_centers.size());
    return false;
  }

  for (int i = 0; i < (int)num_thetas.size(); ++i)
  {
    if(num_thetas[i] != basis_function_centers[i].size())
    {
      ROS_ERROR("Number of thetas >%i< does not match number of basis function centers >%i<.", num_thetas[i], (int)basis_function_centers[i].size());
      return false;
    }
    MatrixXd control_cost_matrix = MatrixXd::Identity(num_thetas[i], num_thetas[i]);
    for (int j = 0; j < num_thetas[i]; ++j)
    {
      control_cost_matrix(j,j) = basis_function_centers[i](j) * basis_function_centers[i](j);
    }
    control_costs.push_back(control_cost_matrix);
  }
  return true;
}

bool DMPPolicy::updateParameters(const std::vector<MatrixXd>& updates, const std::vector<Eigen::VectorXd>& time_step_weights)
{
  if (!initialized_)
  {
    return false;
  }

  std::vector<VectorXd> theta_vectors;
  if (!dmp_->getThetas(theta_vectors))
  {
    ROS_ERROR("Could not get parameter vector. Cannot update parameters of the DMP policy.");
    return false;
  }
  assert(theta_vectors.size() == updates.size());

  std::vector<MatrixXd> basis_functions;
  if (!getBasisFunctions(basis_functions))
  {
    ROS_ERROR("Could not get basis function matrix. Cannot update parameters of the DMP policy.");
    return false;
  }
  assert(basis_functions.size() == updates.size());

  for (int d = 0; d < static_cast<int> (updates.size()); d++)
  {
    int num_rfs = basis_functions[d].cols();

    assert(updates[d].rows() == static_cast<int>(num_time_steps_));
    assert(updates[d].cols() == static_cast<int>(theta_vectors[d].size()));
    assert(updates[d].rows() == basis_functions[d].rows());
    assert(updates[d].cols() == basis_functions[d].cols());

    for (int j = 0; j < num_rfs; j++)
    {
      double sum_time_weight_times_basis_function_weight = 0;
      double sum_time_weight_times_basis_function_weight_times_update = 0;
      for (int i = 0; i < num_time_steps_; i++)
      {
        double time_weight = num_time_steps_ - i;
        sum_time_weight_times_basis_function_weight += (time_weight * basis_functions[d](i, j));
        sum_time_weight_times_basis_function_weight_times_update += ((time_weight * basis_functions[d](i, j)) * updates[d](i, j));
      }

      // update the theta vector
      theta_vectors[d](j) += sum_time_weight_times_basis_function_weight_times_update / sum_time_weight_times_basis_function_weight;
    }
  }

  if (!dmp_->setThetas(theta_vectors))
  {
    ROS_ERROR("Could not set parameter vector. Cannot update parameters of the DMP policy.");
    return false;
  }

  return true;
}

bool DMPPolicy::getParameters(std::vector<VectorXd>& parameters)
{
  if (!initialized_)
  {
    return false;
  }
  parameters.clear();
  return dmp_->getThetas(parameters);
}

bool DMPPolicy::setParameters(const std::vector<VectorXd>& parameters)
{
  if (!initialized_)
  {
    return false;
  }
  return dmp_->setThetas(parameters);
}

bool DMPPolicy::getDMP(dmp_lib::DMPPtr& dmp)
{
  if (!initialized_)
  {
    ROS_ERROR("DMPPolicy is not initialized.");
    return false;
  }
  dmp = dmp_;
  return true;
}

bool DMPPolicy::writeToFile(const std::string& abs_file_name)
{
  return dmp_utilities::DynamicMovementPrimitiveUtilities::writeToFile(abs_file_name, dmp_);
}

bool DMPPolicy::readFromFile(const std::string& abs_file_name)
{
  return dmp_utilities::DynamicMovementPrimitiveUtilities::readFromFile(abs_file_name, dmp_);
}

std::string DMPPolicy::getClassName()
{
  return "DMPPolicy";
}

}
