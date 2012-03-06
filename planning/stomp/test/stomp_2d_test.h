/*
 * stomp_2d_test.h
 *
 *  Created on: Feb 2, 2012
 *      Author: kalakris
 */

#ifndef STOMP_2D_TEST_H_
#define STOMP_2D_TEST_H_

#include <stomp/stomp.h>
#include <stomp/task.h>
#include <boost/enable_shared_from_this.hpp>

namespace stomp
{

class Stomp2DTest: public Task, public boost::enable_shared_from_this<Stomp2DTest>
{
public:

  Stomp2DTest():
    node_handle_("~")
  {
  }

  int run();

  // functions inherited from Task:

  /**
   * Initialize the task for a given number of threads.
   * @param num_threads Number of threads for multi-threading
   * @return
   */
  virtual bool initialize(int num_threads);

  /**
   * Executes the task for the given policy parameters, and returns the costs per timestep
   * @param parameters [num_dimensions] num_parameters - policy parameters to execute
   * @param costs Vector of num_time_steps, state space cost per timestep (do not include control costs)
   * @param weighted_feature_values num_time_steps x num_features matrix of weighted feature values per time step
   * @return
   */
  virtual bool execute(std::vector<Eigen::VectorXd>& parameters,
                       Eigen::VectorXd& costs,
                       Eigen::MatrixXd& weighted_feature_values,
                       const int iteration_number,
                       int thread_id);

  /**
   * Get the Policy object of this Task
   * @param policy
   * @return
   */
  virtual bool getPolicy(boost::shared_ptr<stomp::CovariantMovementPrimitive>& policy);

  /**
   * Sets the Policy object of this Task
   * @param policy
   * @return
   */
  virtual bool setPolicy(const boost::shared_ptr<stomp::CovariantMovementPrimitive> policy);

  /**
   * Gets the weight of the control cost
   * @param control_cost_weight
   * @return
   */
  virtual double getControlCostWeight();

private:
  stomp::STOMP stomp_;
  boost::shared_ptr<stomp::CovariantMovementPrimitive> policy_;
  ros::NodeHandle node_handle_;

  int num_time_steps_;
  int num_dimensions_;
  double movement_duration_;
};

} /* namespace stomp */
#endif /* STOMP_2D_TEST_H_ */
