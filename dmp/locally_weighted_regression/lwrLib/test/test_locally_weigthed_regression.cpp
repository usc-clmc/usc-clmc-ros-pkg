/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		test_locally_weigthed_regression.cpp

 \author	Peter Pastor
 \date		Nov 4, 2010

 *********************************************************************/

// system includes
#include <stdio.h>
#include <time.h>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

// local includes
#include <lwr_lib/lwr.h>
#include <lwr_lib/logger.h>

// import most common Eigen types
using namespace Eigen;
using namespace lwr_lib;

/*!
 */
class LWRTest
{

public:

  LWRTest() {};
  virtual ~LWRTest() {};

  bool initialize();

  bool testLearning();

  double targetFunction(const double test_x);

private:

  LWRParamPtr params_;
  LWRPtr lwr_;

};

bool LWRTest::initialize()
{
  int num_rfs = 20;
  double activation = 0.7;
  bool exponentially_spaced = false;

  params_.reset(new LWRParameters());

  if (!params_->initialize(num_rfs, activation, exponentially_spaced))
  {
    Logger::logPrintf("Could not initialize lwr parameters.", Logger::ERROR);
    return false;
  }

  lwr_.reset(new LWR());
  if (!lwr_->initialize(params_))
  {
    Logger::logPrintf("Could not initialize LWR model.", Logger::ERROR);
    return false;
  }

  return true;
}

double LWRTest::targetFunction(const double test_x)
{
  return -pow(test_x - 0.5, 2);
}

bool LWRTest::testLearning()
{
  int num_data_learn = 1000;
  int num_data_query = 2000;

  double mse_prediction_error_threshold = 2.25118;

  // initialize random seed
  srand(time(NULL));

  // generate input vector
  VectorXd test_x = VectorXd::Zero(num_data_learn);
  test_x(0) = 0;
  double dx = static_cast<double> (1.0) / (test_x.size() - 1);
  for (int i = 1; i < test_x.size(); i++)
  {
    test_x(i) = test_x(i - 1) + dx;
  }

  // generate target
  VectorXd test_y = VectorXd::Zero(test_x.size());
  for (int i = 1; i < test_x.size(); i++)
  {
    test_y(i) = targetFunction(test_x(i));
  }

  // learn parameters
  if (!lwr_->learn(test_x, test_y))
  {
    Logger::logPrintf("Could not learn weights.", Logger::ERROR);
    return false;
  }

  VectorXd test_xq = VectorXd::Zero(num_data_query);
  test_xq(0) = 0;
  dx = static_cast<double> (1.0) / (test_xq.size() - 1);
  for (int i = 1; i < test_xq.size(); i++)
  {
    test_xq(i) = test_xq(i - 1) + dx;
  }

  // get predictions
  VectorXd test_yp = VectorXd::Zero(test_xq.size());
  for (int i = 0; i < test_xq.size(); i++)
  {
    if (!lwr_->predict(test_xq(i), test_yp(i)))
    {
      Logger::logPrintf("Could not predict from LWR model.", Logger::ERROR);
      return false;
    }
  }

  // log data
  std::ofstream outfile;
  outfile.open(std::string(std::string("test_x.txt")).c_str());
  outfile << test_x;
  outfile.close();

  outfile.open(std::string(std::string("test_y.txt")).c_str());
  outfile << test_y;
  outfile.close();

  outfile.open(std::string(std::string("test_xq.txt")).c_str());
  outfile << test_xq;
  outfile.close();

  outfile.open(std::string(std::string("test_yp.txt")).c_str());
  outfile << test_yp;
  outfile.close();

  // compute mean squared error
  double mse = 0;
  for (int i = 0; i < test_xq.size(); i++)
  {
    mse += pow(targetFunction(test_xq(i)) - test_yp(i), 2);
  }
  if (mse > mse_prediction_error_threshold)
  {
    Logger::logPrintf("MSE of the prediciton >%f< is larger than the threshold >%f<.", Logger::ERROR, mse, mse_prediction_error_threshold);
    return false;
  }

	Logger::logPrintf("Test finished successfully.", Logger::INFO);
  return true;
}

int main()
{
  LWRTest lwr_test;
  if (!lwr_test.initialize())
  {
    return -1;
  }
  if (lwr_test.testLearning())
  {
    return -1;
  }
  return 0;
}
