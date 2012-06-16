/*
 * stomp_2d_test.cpp
 *
 *  Created on: Feb 2, 2012
 *      Author: kalakris
 */

#include "stomp_2d_test.h"
#include <ros/ros.h>
#include <sstream>
#include <cstdio>
#include <usc_utilities/param_server.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace stomp
{

int Stomp2DTest::run()
{
  num_dimensions_ = 2;
  readParameters();
  mkdir(output_dir_.c_str(), 0755);
  writeCostFunction();

  std::vector<Eigen::MatrixXd> derivative_costs;
  std::vector<Eigen::VectorXd> initial_trajectory;
  derivative_costs.resize(num_dimensions_, Eigen::MatrixXd::Zero(num_time_steps_ + 2*TRAJECTORY_PADDING, NUM_DIFF_RULES));
  initial_trajectory.resize(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_ + 2*TRAJECTORY_PADDING));
  for (int d=0; d<num_dimensions_; ++d)
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

  ros::NodeHandle stomp_node_handle(node_handle_, "stomp");
  stomp_.initialize(stomp_node_handle, shared_from_this());

  std::stringstream sss;
  sss << output_dir_ << "/noiseless_0.txt";
  policy_->writeToFile(sss.str());

  CovariantMovementPrimitive tmp_policy = *policy_;

  for (int i=1; i<1000; ++i)
  {
    stomp_.runSingleIteration(i);
    std::stringstream ss;
    ss << output_dir_ << "/noiseless_" << i << ".txt";
    policy_->writeToFile(ss.str());

    std::vector<Rollout> rollouts;
    stomp_.getAllRollouts(rollouts);
    for (unsigned int j=0; j<rollouts.size(); ++j)
    {
      std::stringstream ss2;
      ss2 << output_dir_ << "/noisy_" << i << "_" << j << ".txt";
      tmp_policy.setParameters(rollouts[j].parameters_noise_projected_);
      tmp_policy.writeToFile(ss2.str());
    }
  }

  return 0;
}

bool Stomp2DTest::initialize(int num_threads)
{
  return true;
}

void Stomp2DTest::writeCostFunction()
{
  std::stringstream ss;
  ss << output_dir_ << "/cost_function.txt";
  double resolution = 0.005;
  int num_x = lrint(1.0 / resolution) + 1;
  int num_y = lrint(1.0 / resolution) + 1;

  FILE *f = fopen(ss.str().c_str(), "w");
  fprintf(f, "%d\t%d\n", num_x, num_y);
  for (int i=0; i<num_x; ++i)
  {
    double x = i*resolution;
    for (int j=0; j<num_y; ++j)
    {
      double y = j*resolution;
      double cost = evaluateCost(x, y);
      fprintf(f, "%lf\t%lf\t%lf\n", x, y, cost);
    }
  }
  fclose(f);
}

void Stomp2DTest::readParameters()
{
  // WARNING, TODO: no error checking here!!!
  obstacles_.clear();
  XmlRpc::XmlRpcValue obstacles_xml;
  node_handle_.getParam("cost_function", obstacles_xml);
  for (int i=0; i<obstacles_xml.size(); ++i)
  {
    Obstacle o;
    usc_utilities::getParam(obstacles_xml[i], "center", o.center_);
    usc_utilities::getParam(obstacles_xml[i], "radius", o.radius_);
    usc_utilities::getParam(obstacles_xml[i], "boolean", o.boolean_);
    obstacles_.push_back(o);
  }

  usc_utilities::read(node_handle_, "num_time_steps", num_time_steps_);
  usc_utilities::read(node_handle_, "movement_duration", movement_duration_);
  usc_utilities::read(node_handle_, "control_cost_weight", control_cost_weight_);
  usc_utilities::read(node_handle_, "output_dir", output_dir_);
}

bool Stomp2DTest::execute(std::vector<Eigen::VectorXd>& parameters,
                     Eigen::VectorXd& costs,
                     Eigen::MatrixXd& weighted_feature_values,
                     const int iteration_number,
                     const int rollout_number,
                     int thread_id)
{
  costs = Eigen::VectorXd::Zero(num_time_steps_);
  weighted_feature_values = Eigen::MatrixXd::Zero(num_time_steps_, 1);
  for (int t=0; t<num_time_steps_; ++t)
  {
    double x = parameters[0](t);
    double y = parameters[1](t);
    double cost = evaluateCost(x, y);
    costs(t) = cost;
  }
  return true;
}

double Stomp2DTest::evaluateCost(double x, double y)
{
  double cost = 0.0;
  for (unsigned int o=0; o<obstacles_.size(); ++o)
  {
    double dx = (x - obstacles_[o].center_[0])/obstacles_[o].radius_[0];
    double dy = (y - obstacles_[o].center_[1])/obstacles_[o].radius_[1];

    double dist = dx * dx + dy * dy;

    if (obstacles_[o].boolean_)
    {
      if (dist < 1.0)
        cost += 1.0;
    }
    else
    {
      // TODO
    }
  }

  // joint limits
  if (x < 0.0 || x > 1.0)
    cost += 999999999.0;
  if (y < 0.0 || y > 1.0)
    cost += 999999999.0;
  return cost;
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
  return control_cost_weight_;
}

} /* namespace stomp */

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "Stomp2DTest");
  boost::shared_ptr<stomp::Stomp2DTest> test(new stomp::Stomp2DTest());
  return test->run();
}
