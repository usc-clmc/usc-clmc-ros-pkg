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
#include <stdlib.h>
#include <time.h>

namespace stomp
{

int Stomp2DTest::run()
{
  srand(time(NULL));
  num_dimensions_ = 2;
  resolution_ = 0.002;
  readParameters();
  mkdir(output_dir_.c_str(), 0755);

  std::stringstream stddev_filename, cost_filename;
  stddev_filename << output_dir_ << "/stddevs.txt";
  cost_filename << output_dir_ << "/costs.txt";
  FILE *stddev_file = fopen(stddev_filename.str().c_str(), "w");
  FILE *cost_file = fopen(cost_filename.str().c_str(), "w");

  std::vector<Eigen::MatrixXd> derivative_costs;
  std::vector<Eigen::VectorXd> initial_trajectory;
  derivative_costs.resize(num_dimensions_, Eigen::MatrixXd::Zero(num_time_steps_ + 2*TRAJECTORY_PADDING, NUM_DIFF_RULES));
  initial_trajectory.resize(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_ + 2*TRAJECTORY_PADDING));
  for (int d=0; d<num_dimensions_; ++d)
  {
    derivative_costs[d].col(STOMP_VELOCITY) = Eigen::VectorXd::Ones(num_time_steps_ + 2*TRAJECTORY_PADDING);
    //derivative_costs[d].col(STOMP_ACCELERATION) = 0.01*Eigen::VectorXd::Ones(num_time_steps_ + 2*TRAJECTORY_PADDING);
    //derivative_costs[d].col(STOMP_ACCELERATION) = Eigen::VectorXd::Ones(num_time_steps_ + 2*TRAJECTORY_PADDING);
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
  movement_dt_ = policy_->getMovementDt();

  writeCostFunction();

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
    Rollout noiseless_rollout;
    stomp_.getNoiselessRollout(noiseless_rollout);
    for (unsigned int j=0; j<rollouts.size(); ++j)
    {
      std::stringstream ss2;
      ss2 << output_dir_ << "/noisy_" << i << "_" << j << ".txt";
      tmp_policy.setParameters(rollouts[j].parameters_noise_projected_);
      tmp_policy.writeToFile(ss2.str());
    }
    fprintf(cost_file, "%f\n", noiseless_rollout.total_cost_);

    std::vector<double> stddevs;
    stomp_.getAdaptedStddevs(stddevs);
    for (unsigned int d=0; d<stddevs.size(); ++d)
    {
      fprintf(stddev_file, "%f\t", stddevs[d]);
    }
    fprintf(stddev_file, "\n");
  }

  fclose(stddev_file);
  fclose(cost_file);

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
  int num_x = lrint(1.0 / resolution_) + 1;
  int num_y = lrint(1.0 / resolution_) + 1;

  FILE *f = fopen(ss.str().c_str(), "w");
  fprintf(f, "%d\t%d\n", num_x, num_y);
  for (int i=0; i<num_x; ++i)
  {
    double x = i*resolution_;
    for (int j=0; j<num_y; ++j)
    {
      double y = j*resolution_;
      double cost = evaluateMapCost(x, y);
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
                     int thread_id,
                     bool compute_gradients,
                     std::vector<Eigen::VectorXd>& gradients)
{
  costs = Eigen::VectorXd::Zero(num_time_steps_);
  weighted_feature_values = Eigen::MatrixXd::Zero(num_time_steps_, 1);
  std::vector<Eigen::VectorXd> vel(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_));
  std::vector<Eigen::VectorXd> acc(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_));

  if (compute_gradients)
  {
    gradients.resize(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_));
  }

  for (int d=0; d<num_dimensions_; ++d)
  {
    stomp::differentiate(parameters[d], stomp::STOMP_VELOCITY, vel[d], movement_dt_);
    if (compute_gradients)
    {
      stomp::differentiate(parameters[d], stomp::STOMP_ACCELERATION, acc[d], movement_dt_);
    }
  }
  double px = 0.0;
  double py = 0.0;
  for (int t=0; t<num_time_steps_; ++t)
  {
    double x = parameters[0](t);
    double y = parameters[1](t);
    double cost = 0.0;
    if (compute_gradients)
    {
      cost = evaluateCostPathWithGradients(px, py, x, y, vel[0](t), vel[1](y), true,
                                           acc[0](t), acc[1](t), gradients[0](t), gradients[1](t));
    }
    else
    {
      cost = evaluateCostPath(px, py, x, y, vel[0](t), vel[1](t));
    }
    costs(t) = cost;
    px = x;
    py = y;
  }
  return true;
}

double Stomp2DTest::evaluateCost(double x, double y, double vx, double vy)
{
  double ax=0.0, ay=0.0, gx=0.0, gy=0.0;
  return evaluateCostWithGradients(x, y, vx, vy, false, ax, ay, gx, gy);
}

double Stomp2DTest::evaluateMapCost(double x, double y)
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
      if (dist < 1.0)
        cost += 1.0 - dist;
    }
  }

  // joint limits
  if (x < 0.0 || x > 1.0)
    cost += 999999999.0;
  if (y < 0.0 || y > 1.0)
    cost += 999999999.0;
  return cost;
}

void Stomp2DTest::evaluateMapGradients(double x, double y, double& gx, double& gy)
{
  gx = evaluateMapCost(x+resolution_, y) - evaluateMapCost(x-resolution_, y) / (2*resolution_);
  gy = evaluateMapCost(x, y+resolution_) - evaluateMapCost(x, y-resolution_) / (2*resolution_);
}

double Stomp2DTest::evaluateCostWithGradients(double x, double y, double vx, double vy,
                                bool compute_gradients,
                                double ax, double ay, double& gx, double& gy)
{
  double cost = evaluateMapCost(x,y) * movement_dt_;
  double vel_mag = sqrt(vx*vx + vy*vy);

  if (compute_gradients)
  {
    double map_gx=0.0, map_gy=0.0;
    evaluateMapGradients(x, y, map_gx, map_gy);

    map_gx *= movement_dt_;
    map_gy *= movement_dt_;

    Eigen::Vector2d vel;
    Eigen::Vector2d norm_vel;
    vel(0) = vx;
    vel(1) = vy;
    norm_vel = vel.normalized();
    Eigen::Matrix2d orth_proj = Eigen::Matrix2d::Identity() - norm_vel*norm_vel.transpose();
    Eigen::Vector2d acc;
    acc(0) = ax;
    acc(1) = ay;
    Eigen::Vector2d curvature = (1.0/vel.squaredNorm()) * orth_proj * acc;
    Eigen::Vector2d grad;
    grad(0) = map_gx;
    grad(1) = map_gy;
    Eigen::Vector2d new_grad = vel_mag * (orth_proj*grad - cost*curvature);
    gx = new_grad(0);
    gy = new_grad(1);
  }

  return cost*vel_mag;
}

double Stomp2DTest::evaluateCostPath(double x1, double y1, double x2, double y2, double vx, double vy)
{
  double ax = 0.0, ay = 0.0, gx = 0.0, gy = 0.0;
  return evaluateCostPathWithGradients(x1, y1, x2, y2, vx, vy, false, ax, ay, gx, gy);
}

double Stomp2DTest::evaluateCostPathWithGradients(double x1, double y1, double x2, double y2, double vx, double vy,
                                     bool compute_gradients,
                                     double ax, double ay, double& gx, double& gy)
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  double dist = sqrt(dx*dx + dy*dy);
  int num_samples = ceil(dist / resolution_);
  if (compute_gradients)
  {
    gx = 0.0;
    gy = 0.0;
  }
  if (num_samples == 0)
    return 0.0;
  double cost = 0.0;
  for (int i=0; i<num_samples; ++i) // leave out the last one to avoid double counting
  {
    double d = (double(i) / double(num_samples));
    double x = x1 + d*dx;
    double y = y1 + d*dy;
    double temp_gx, temp_gy;
    cost += evaluateCostWithGradients(x, y, vx, vy, compute_gradients, ax, ay, temp_gx, temp_gy);
    gx += temp_gx;
    gy += temp_gy;
  }
  cost /= num_samples;
  if (compute_gradients)
  {
    gx /= num_samples;
    gy /= num_samples;
  }
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
