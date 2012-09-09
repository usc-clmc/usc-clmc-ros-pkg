/*
 * constrained_ik_solver.h
 *
 *  Created on: Jun 10, 2011
 *      Author: kalakris
 */

#ifndef CONSTRAINED_IK_SOLVER_H_
#define CONSTRAINED_IK_SOLVER_H_

#include <boost/shared_ptr.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <constrained_inverse_kinematics/fk_solver.h>
#include <constrained_inverse_kinematics/chain.h>
#include <constrained_inverse_kinematics/InverseKinematicsRequest.h>
#include <constrained_inverse_kinematics/cost_function_input.h>
#include <learnable_cost_function/cost_function.h>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/function.hpp>

namespace constrained_inverse_kinematics
{

struct IKSolution
{
  KDL::JntArray joint_angles;
  KDL::Frame tool_frame;
  KDL::Frame end_effector_frame;
  double cost_function_value;
  bool success;
  Eigen::VectorXd feature_values;

  // slightly hacky since pre_grasp stuff is now handled in IKWrapper
  std::vector<IKSolution> pre_grasp_solutions;

  double getCostRecursive()
  {
    double cost = cost_function_value;
    for (unsigned int i=0; i<pre_grasp_solutions.size(); ++i)
      cost += pre_grasp_solutions[i].getCostRecursive();
    return cost;
  }
};

class ConstrainedIKSolver
{
public:
  ConstrainedIKSolver(ros::NodeHandle node_handle, const std::string& root, const std::string& tip);
  virtual ~ConstrainedIKSolver();

  void setCostFunction(boost::shared_ptr<learnable_cost_function::CostFunction> cost_function);

  void setFKSolver(boost::shared_ptr<const FKSolver> fk_solver);
  void useDefaultFKSolver();

  virtual bool ikLocal(const InverseKinematicsRequest& ik_request,
             const KDL::JntArray& q_in,
             IKSolution& solution) const;

  void prepareCostFunctionInput(const InverseKinematicsRequest& ik_request,
                                const std::vector<double>& joint_angles,
                                boost::shared_ptr<CostFunctionInput> cost_function_input);

  double evaluateCostFunctionInput(boost::shared_ptr<CostFunctionInput> cost_function_input, Eigen::VectorXd& feature_values);

  void registerDebugCallback(boost::function<void (const KDL::JntArray& q)> f);

  void getRandomJointAngles(KDL::JntArray& joint_angles) const;

  void setCostFunctionWeights(const Eigen::VectorXd& weights);

  void setMaxIterations(int max_iterations);

protected:
  ros::NodeHandle node_handle_;
  boost::shared_ptr<const Chain> chain_;
  boost::function<void (const KDL::JntArray& q)> debug_callback_;

  boost::shared_ptr<const FKSolver> default_fk_solver_;
  boost::shared_ptr<const FKSolver> fk_solver_;

  //std::vector<boost::shared_ptr<learnable_cost_function::CostFunction> > cost_function_copies_;
  boost::shared_ptr<learnable_cost_function::CostFunction> cost_function_;

  // max errors that we try to correct in a single step
  double max_translation_error_;
  double max_orientation_error_;

  // convergence thresholds
  double position_convergence_threshold_;
  double orientation_convergence_threshold_;

  // min joint update
  double min_joint_update_threshold_;

  int max_iterations_;

  // max joint update in null space
  double max_null_space_joint_update_;

  boost::shared_ptr<boost::variate_generator<boost::mt19937, boost::uniform_01<> > > random_generator_;

};

}

#endif /* CONSTRAINED_IK_SOLVER_H_ */
