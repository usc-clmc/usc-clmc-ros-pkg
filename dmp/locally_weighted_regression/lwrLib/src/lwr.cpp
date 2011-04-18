/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		locally_weighted_regression.cpp

 \author	Peter Pastor
 \date		Nov 4, 2010

 *********************************************************************/

// system include
#include <stdio.h>
#include <cassert>

// local include
#include <lwr_lib/lwr.h>
#include <lwr_lib/logger.h>
#include <lwr_lib/utilities.h>

using namespace Eigen;
using namespace std;

namespace lwr_lib
{

LWR& LWR::operator=(const LWR& lwr_model)
{
  Logger::logPrintf("LWR assignment.", Logger::DEBUG);

  // assign memeber variables
  assert(Utilities<LWRParameters>::assign(parameters_, lwr_model.parameters_));
  initialized_ = lwr_model.initialized_;
  return *this;
}

bool LWR::initialize(const LWRParamPtr parameters)
{
  if (!parameters->initialized_)
  {
    Logger::logPrintf("Cannot initialze LWR model from uninitialized parameters.", Logger::ERROR);
    return (initialized_ = false);
  }
  // copy the content of parameters
  Logger::logPrintf(initialized_, "LWR model already initialized. Re-initializing...", Logger::WARN);
  assert(Utilities<LWRParameters>::assign(parameters_, parameters));
  return (initialized_ = true);
}

// REAL-TIME REQUIREMENTS
double LWR::evaluateKernel(const double x_input,
                           const int center_index) const
{
  assert(parameters_->initialized_);
  return exp(-(static_cast<double> (1.0) / parameters_->widths_(center_index)) * pow(x_input - parameters_->centers_(center_index), 2));
}

bool LWR::generateBasisFunctionMatrix(const VectorXd& x_input_vector,
                                      MatrixXd& basis_function_matrix) const
{
  if(x_input_vector.size() == 0)
  {
    Logger::logPrintf("Cannot generate basis function matrix from empty input vector.", Logger::ERROR);
    return false;
  }
  if((basis_function_matrix.rows() != x_input_vector.size())
      || (basis_function_matrix.cols() != parameters_->centers_.size()))
  {
    Logger::logPrintf("Size of provided basis_function_matrix (%i x %i) is incorrect. It should be (%i x %i). Cannot generate basis function matrix.",
                      Logger::ERROR, basis_function_matrix.rows(), basis_function_matrix.cols(), x_input_vector.size(), parameters_->centers_.size());
    return false;
  }
  for (int i = 0; i < x_input_vector.size(); ++i)
  {
    // TODO: think about using the generateBasisFunction function
    for (int j = 0; j < parameters_->centers_.size(); ++j)
    {
      basis_function_matrix(i, j) = evaluateKernel(x_input_vector[i], j);
    }
  }
  return true;
}

bool LWR::generateBasisFunctionVector(const double x_input, VectorXd& basis_functions) const
{
  if(basis_functions.size() != parameters_->centers_.size())
  {
    Logger::logPrintf("Size of provided basis function vector >%i< is incorrect, it should be of size >%i<.", Logger::ERROR, basis_functions.size(), parameters_->centers_.size());
    return false;
  }
  for (int i = 0; i < parameters_->centers_.size(); ++i)
  {
    basis_functions(i) = evaluateKernel(x_input, i);
  }
  return true;
}

bool LWR::generateBasisFunction(const double x_input, const int rfs_index, double& basis_function) const
{
  if(rfs_index < 0 || rfs_index >= parameters_->centers_.size())
  {
    Logger::logPrintf("Cannot generate basis function value of RFS with index >%i<.", Logger::ERROR, rfs_index);
    return false;
  }
  basis_function = evaluateKernel(x_input, rfs_index);
  return true;
}

bool LWR::learn(const VectorXd& x_input_vector,
                const VectorXd& y_target_vector)
{

  assert(parameters_->initialized_);
  if(x_input_vector.size() != y_target_vector.size())
  {
    Logger::logPrintf("Size of provided vectors (input >%i< and target >%i<) does not match.",
                      Logger::ERROR, x_input_vector.size(), y_target_vector.size());
    return false;
  }

  MatrixXd basis_function_matrix = MatrixXd::Zero(x_input_vector.size(), parameters_->centers_.size());
  if (!generateBasisFunctionMatrix(x_input_vector, basis_function_matrix))
  {
    Logger::logPrintf("Could not generate basis function matrix..", Logger::ERROR);
    return false;
  }

  MatrixXd tmp_matrix_a = MatrixXd::Zero(x_input_vector.size(), parameters_->num_rfs_);
  tmp_matrix_a = x_input_vector.array().square().matrix() * MatrixXd::Ones(1, parameters_->num_rfs_);
  tmp_matrix_a = (tmp_matrix_a.array() * basis_function_matrix.array()).matrix();

  VectorXd tmp_matrix_sx = VectorXd::Zero(parameters_->num_rfs_, 1);
  tmp_matrix_sx = tmp_matrix_a.colwise().sum();

  MatrixXd tmp_matrix_b = MatrixXd::Zero(x_input_vector.size(), parameters_->num_rfs_);
  tmp_matrix_b = (x_input_vector.array() * y_target_vector.array()).matrix() * MatrixXd::Ones(1, parameters_->num_rfs_);
  tmp_matrix_b = (tmp_matrix_b.array() * basis_function_matrix.array()).matrix();

  VectorXd tmp_matrix_sxtd = VectorXd::Zero(parameters_->num_rfs_, 1);
  tmp_matrix_sxtd = tmp_matrix_b.colwise().sum();

  // TODO: change this...
  double ridge_regression = 0.0000000001;
  parameters_->slopes_ = (tmp_matrix_sxtd.array() / (tmp_matrix_sx.array() + ridge_regression)).matrix();

  return true;
}

// REAL-TIME REQUIREMENTS
bool LWR::predict(const double x_query,
                  double& y_prediction)
{
  assert(parameters_->initialized_);
  double sx = 0;
  double sxtd = 0;
  for (int i = 0; i < parameters_->num_rfs_; i++)
  {
    double psi = evaluateKernel(x_query, i);
    sxtd += psi * parameters_->slopes_(i) * x_query;
    sx += psi;
  }
  // TODO: thinkg about this...
  if (sx < 0.000000001 && sx > -0.000000001)
  {
    y_prediction = 0;
    return false;
  }
  y_prediction = sxtd / sx;
  return true;
}

bool LWR::getThetas(VectorXd& thetas) const
{
  if (!initialized_)
  {
    Logger::logPrintf("Cannot return theta vector, LWR model is not initialized.", Logger::ERROR);
    return false;
  }
  return parameters_->getThetas(thetas);
}

bool LWR::getThetas(vector<double>& thetas) const
{
  if (!initialized_)
  {
    Logger::logPrintf("Cannot return theta vector, LWR model is not initialized.", Logger::ERROR);
    return false;
  }
  return parameters_->getThetas(thetas);
}

bool LWR::setThetas(const VectorXd& thetas)
{
  if (!initialized_)
  {
    Logger::logPrintf("Cannot set theta vector, LWR model is not initialized.", Logger::ERROR);
    return false;
  }
  return parameters_->setThetas(thetas);
}

bool LWR::setThetas(const vector<double>& thetas)
{
  if (!initialized_)
  {
    Logger::logPrintf("Cannot set theta vector, LWR model is not initialized.", Logger::ERROR);
    return false;
  }
  return parameters_->setThetas(thetas);
}

bool LWR::updateThetas(const VectorXd& delta_thetas)
{
  if (!initialized_)
  {
    Logger::logPrintf("Cannot update theta vector, LWR model is not initialized.", Logger::ERROR);
    return false;
  }
  return parameters_->updateThetas(delta_thetas);
}

bool LWR::setWidthsAndCenters(const VectorXd& widths,
                              const VectorXd& centers)
{
  if (!initialized_)
  {
    Logger::logPrintf("Cannot set widths and centers, LWR model is not initialized.", Logger::ERROR);
    return false;
  }
  return parameters_->setWidthsAndCenters(widths, centers);
}

bool LWR::setWidthsAndCenters(const vector<double>& widths,
                              const vector<double>& centers)
{
  if (!initialized_)
  {
    Logger::logPrintf("Cannot set widths and centers, LWR model is not initialized.", Logger::ERROR);
    return false;
  }
  return parameters_->setWidthsAndCenters(widths, centers);
}

bool LWR::getWidthsAndCenters(VectorXd& widths,
                              VectorXd& centers) const
{
  if (!initialized_)
  {
    Logger::logPrintf("Cannot get widths and centers, LWR model is not initialized.", Logger::ERROR);
    return false;
  }
  return parameters_->getWidthsAndCenters(widths, centers);
}

bool LWR::getWidthsAndCenters(vector<double>& widths,
                              vector<double>& centers) const
{
  if (!initialized_)
  {
    Logger::logPrintf("Cannot get widths and centers, LWR model is not initialized.", Logger::ERROR);
    return false;
  }
  return parameters_->getWidthsAndCenters(widths, centers);
}

bool LWR::setOffsets(const VectorXd& offsets)
{
  if (!initialized_)
  {
    Logger::logPrintf("Cannot set offsets, LWR model is not initialized.", Logger::ERROR);
    return false;
  }
  return parameters_->setOffsets(offsets);
}

bool LWR::setOffsets(const vector<double>& offsets)
{
  if (!initialized_)
  {
    Logger::logPrintf("Cannot set offsets, LWR model is not initialized.", Logger::ERROR);
    return false;
  }
  return parameters_->setOffsets(offsets);
}

bool LWR::getOffsets(Eigen::VectorXd& offsets) const
{
  if (!initialized_)
  {
    Logger::logPrintf("Cannot get offsets, LWR model is not initialized.", Logger::ERROR);
    return false;
  }
  return parameters_->getOffsets(offsets);
}

bool LWR::getOffsets(vector<double>& offsets) const
{
  if (!initialized_)
  {
    Logger::logPrintf("Cannot get offsets, LWR model is not initialized.", Logger::ERROR);
    return false;
  }
  return parameters_->getOffsets(offsets);
}

bool LWR::setNumRFS(const int num_rfs)
{
  if (!initialized_)
  {
    Logger::logPrintf("Cannot set number of receptive fields, LWR model is not initialzed.", Logger::ERROR);
    return false;
  }
  return parameters_->setNumRFS(num_rfs);
}

int LWR::getNumRFS() const
{
  if (!initialized_)
  {
    Logger::logPrintf("Cannot return number of receptive fields, LWR model is not initialzed.", Logger::ERROR);
    return false;
  }
  return parameters_->getNumRFS();
}

string LWR::getInfoString() const
{
  string info;
  if (initialized_)
  {
    info.assign("LWR model is initialized.\n");
  }
  else
  {
    return string("LWR model is NOT initialized.");
  }
  if (parameters_->isInitialized())
  {
    info.append(parameters_->getInfoString());
  }
  else
  {
    info.append("LWR parameters are NOT initialized.");
  }
  return info;
}

}
