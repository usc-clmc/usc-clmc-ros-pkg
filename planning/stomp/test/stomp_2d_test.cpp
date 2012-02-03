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
  cost_ridge_factor_ = 0.000001;
  derivative_costs_.push_back(0.0); // vel
  derivative_costs_.push_back(1.0); // acc
  derivative_costs_.push_back(0.0); // jerk

  policy_.reset(new CovariantMovementPrimitive());
  policy_->initialize(node_handle_, num_time_steps_,
                      num_dimensions_, movement_duration_,
                      cost_ridge_factor_, derivative_costs_);

  Eigen::VectorXd start(num_dimensions_);
  start << 0.0, 0.0;
  Eigen::VectorXd goal(num_dimensions_);
  goal << 1.0, 1.0;
  policy_->setToMinControlCost(start, goal);

  stomp_.initialize(node_handle_, shared_from_this());

  policy_->writeToFile("noiseless_0.txt");

  for (int i=1; i<100; ++i)
  {
    stomp_.runSingleIteration(i);
    std::stringstream ss;
    ss << "noiseless_" << i << ".txt";
    policy_->writeToFile(ss.str());
  }

  return 0;
}

bool Stomp2DTest::execute(std::vector<Eigen::VectorXd>& parameters,
                     Eigen::VectorXd& costs,
                     Eigen::MatrixXd& weighted_feature_values,
                     const int iteration_number)
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
  }
  return true;
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

bool Stomp2DTest::getControlCostWeight(double& control_cost_weight)
{
  return 0.00001;
}


} /* namespace stomp */

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "Stomp2DTest");
  boost::shared_ptr<stomp::Stomp2DTest> test(new stomp::Stomp2DTest());
  return test->run();
}
