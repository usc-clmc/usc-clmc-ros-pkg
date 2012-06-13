/*
 * stomp_2d_test.cpp
 *
 *  Created on: Feb 2, 2012
 *      Author: kalakris
 */

#include "stomp_2d_test.h"
#include <ros/ros.h>
#include <sstream>

namespace stomp
{

int Stomp2DTest::run()
{
  num_time_steps_ = 100;
  num_dimensions_ = 2;
  movement_duration_ = 1.0;

  std::vector<Eigen::MatrixXd> derivative_costs;
  std::vector<Eigen::VectorXd> initial_trajectory;
  derivative_costs.resize(num_dimensions_, Eigen::MatrixXd::Zero(num_time_steps_ + 2*TRAJECTORY_PADDING, NUM_DIFF_RULES));
  initial_trajectory.resize(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_ + 2*TRAJECTORY_PADDING));
  for (int d=0 ; d< num_dimensions_; ++d)
  {
    derivative_costs[d].col(STOMP_ACCELERATION) = Eigen::VectorXd::Ones(num_time_steps_ + 2*TRAJECTORY_PADDING);
    //derivative_costs[d].col(STOMP_POSITION) = 0.0001 * Eigen::VectorXd::Ones(num_time_steps_ + 2*TRAJECTORY_PADDING);
    initial_trajectory[d].head(TRAJECTORY_PADDING) = Eigen::VectorXd::Zero(TRAJECTORY_PADDING);
    initial_trajectory[d].tail(TRAJECTORY_PADDING) = Eigen::VectorXd::Ones(TRAJECTORY_PADDING);

//    derivative_costs[d](30, STOMP_POSITION) = 1000000.0;
//    initial_trajectory[d](30) = 0.3;
//    derivative_costs[d](80, STOMP_POSITION) = 1000000.0;
//    initial_trajectory[d](80) = 0.8;
  }

  policy_.reset(new CovariantMovementPrimitive());
  policy_->initialize(num_time_steps_,
                      num_dimensions_,
                      movement_duration_,
                      derivative_costs,
                      initial_trajectory);

  policy_->setToMinControlCost();

  stomp_.initialize(node_handle_, shared_from_this());

  policy_->writeToFile("noiseless_0.txt");

  CovariantMovementPrimitive tmp_policy = *policy_;

  for (int i=1; i<1000; ++i)
  {
    stomp_.runSingleIteration(i);
    std::stringstream ss;
    ss << "noiseless_" << i << ".txt";
    policy_->writeToFile(ss.str());

    std::vector<Rollout> rollouts;
    stomp_.getAllRollouts(rollouts);
    for (unsigned int j=0; j<rollouts.size(); ++j)
    {
      std::stringstream ss2;
      ss2 << "noisy_" << i << "_" << j << ".txt";
      tmp_policy.setParameters(rollouts[j].parameters_noise_projected_);
      tmp_policy.writeToFile(ss2.str());
    }
  }

  return 0;
}

bool Stomp2DTest::initialize(int num_threads)
{

}

bool Stomp2DTest::execute(std::vector<Eigen::VectorXd>& parameters,
                     Eigen::VectorXd& costs,
                     Eigen::MatrixXd& weighted_feature_values,
                     const int iteration_number,
                     const int rollout_number,
                     int thread_id)
{
  // we have an obstacle at 0.5, 0.5
  // assign cost based on distance to it
  costs = Eigen::VectorXd::Zero(num_time_steps_);
  weighted_feature_values = Eigen::MatrixXd::Zero(num_time_steps_, 1);
  for (int t=0; t<num_time_steps_; ++t)
  {
    double dist = sqrt(pow(parameters[0](t) - 0.5, 2) + pow(parameters[1](t) - 0.5, 2));
    costs(t) = 0;
    if (dist < 0.25)
    {
      costs(t) = 4 * (0.25 - dist);
    }

    // joint limits
    if (parameters[0](t) < 0.0 || parameters[0](t) > 1.0)
      costs(t) += 999999999.0;
    if (parameters[1](t) < 0.0 || parameters[1](t) > 1.0)
      costs(t) += 999999999.0;
  }
  return true;
}

bool Stomp2DTest::filter(std::vector<Eigen::VectorXd>& parameters, int thread_id)
{
  bool filtered = false;
  for (unsigned int d=0; d<parameters.size(); ++d)
  {
    for (int t=0; t<num_time_steps_; ++t)
    {
      if (parameters[d](t) < 0.0)
      {
        parameters[d](t) = 0.0;
        filtered = true;
      }
      if (parameters[d](t) > 1.0)
      {
        parameters[d](t) = 1.0;
        filtered = true;
      }
    }
  }
  return filtered;
}

bool Stomp2DTest::getPolicy(boost::shared_ptr<stomp::CovariantMovementPrimitive>& policy)
{
  policy = policy_;
  return true;
}

bool Stomp2DTest::setPolicy(const boost::shared_ptr<stomp::CovariantMovementPrimitive> policy)
{
  policy_ = policy;
  return true;
}

double Stomp2DTest::getControlCostWeight()
{
  return 0.00000001;
}


} /* namespace stomp */

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "Stomp2DTest");
  boost::shared_ptr<stomp::Stomp2DTest> test(new stomp::Stomp2DTest());
  return test->run();
}
