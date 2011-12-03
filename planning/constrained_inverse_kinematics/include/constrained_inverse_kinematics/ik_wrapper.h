/*
 * ik_wrapper.h
 *
 *  Created on: Oct 25, 2011
 *      Author: mrinal
 */

#ifndef IK_WRAPPER_H_
#define IK_WRAPPER_H_

#include <learnable_cost_function/cost_function.h>
#include <constrained_inverse_kinematics/constrained_ik_solver.h>

namespace constrained_inverse_kinematics
{

class IKWrapper
{
public:
  IKWrapper(ros::NodeHandle node_handle, const std::string& root, const std::string& tip);
  virtual ~IKWrapper();

  void setCostFunction(boost::shared_ptr<learnable_cost_function::CostFunction> cost_function);
  void setSourceCostFunction(boost::shared_ptr<learnable_cost_function::CostFunction> cost_function);
  void setTargetCostFunction(boost::shared_ptr<learnable_cost_function::CostFunction> cost_function);

  virtual bool ik(const InverseKinematicsRequest& ik_request,
                IKSolution& solution);

  virtual bool ik(const InverseKinematicsRequest& ik_request,
                std::vector<IKSolution>& solutions,
                IKSolution& best_solution);

  virtual bool ikPreGrasp(const InverseKinematicsRequest& ik_request,
                  const std::vector<KDL::Frame>& grasp_to_pre_grasp_offsets,
                  std::vector<IKSolution>& solutions,
                  IKSolution& best_solution);

  virtual bool ikPreGrasp(const InverseKinematicsRequest& ik_request,
                  const std::vector<KDL::Frame>& grasp_to_pre_grasp_offsets,
                  IKSolution& best_solution);

  virtual bool ikDualPreGrasp(const InverseKinematicsRequest& ik_request1,
                      const std::vector<KDL::Frame>& grasp_to_pre_grasp_offsets1,
                      IKSolution& best_solution1,
                      const InverseKinematicsRequest& ik_request2,
                      const std::vector<KDL::Frame>& grasp_to_pre_grasp_offsets2,
                      IKSolution& best_solution2);

  virtual bool ikLocal(const InverseKinematicsRequest& ik_request,
             const KDL::JntArray& q_in,
             IKSolution& solution);

  void registerDebugCallback(boost::function<void (const KDL::JntArray& q)> f);

  void setFKSolver(boost::shared_ptr<const FKSolver> fk_solver);
  void useDefaultFKSolver();

private:
  ros::NodeHandle node_handle_;
  int max_openmp_threads_;
  Eigen::VectorXd dual_ik_cost_weights_;
  int max_random_attempts_;

  // random number storage for each thread
  std::vector<KDL::JntArray> random_seeds_;

  // IK solvers for each thread (and different ones for source and target)
  std::vector<boost::shared_ptr<ConstrainedIKSolver> > source_ik_solvers_;
  std::vector<boost::shared_ptr<ConstrainedIKSolver> > target_ik_solvers_;
  bool source_ik_solvers_initialized_;
  bool target_ik_solvers_initialized_;

  bool ik(const InverseKinematicsRequest& ik_request,
                std::vector<IKSolution>& solutions,
                IKSolution& best_solution,
                std::vector<boost::shared_ptr<ConstrainedIKSolver> >& ik_solvers,
                bool& ik_solvers_initialized);

  bool ikPreGrasp(const InverseKinematicsRequest& ik_request,
                  const std::vector<KDL::Frame>& grasp_to_pre_grasp_offsets,
                  std::vector<IKSolution>& solutions,
                  IKSolution& best_solution,
                  std::vector<boost::shared_ptr<ConstrainedIKSolver> >& ik_solvers,
                  bool& ik_solvers_initialized);
};

} /* namespace constrained_inverse_kinematics */

#endif /* IK_WRAPPER_H_ */
