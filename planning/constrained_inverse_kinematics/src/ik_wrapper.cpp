/*
 * ik_wrapper.cpp
 *
 *  Created on: Oct 25, 2011
 *      Author: mrinal
 */

#include <constrained_inverse_kinematics/ik_wrapper.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <conversions/kdl_to_ros.h>
#include <omp.h>

//using namespace KDL;
//using namespace Eigen;
using namespace conversions;

namespace constrained_inverse_kinematics
{

IKWrapper::IKWrapper(ros::NodeHandle node_handle, const std::string& root, const std::string& tip):
    node_handle_(node_handle)
{
  node_handle.param("max_random_attempts", max_random_attempts_, 100);

  std::vector<double> dual_ik_cost_weights;
  usc_utilities::read(node_handle, "dual_ik_cost_weights", dual_ik_cost_weights);
  dual_ik_cost_weights_ = Eigen::VectorXd::Zero(dual_ik_cost_weights.size());
  for (unsigned int i=0; i<dual_ik_cost_weights.size(); ++i)
    dual_ik_cost_weights_[i] = sqrt(dual_ik_cost_weights[i]);

  // init openmp
  max_openmp_threads_ = omp_get_max_threads();
  ROS_DEBUG("constrained_ik: Initializing OpenMP with %d threads", max_openmp_threads_);

  // initialize random number generators:
  random_seeds_.resize(max_openmp_threads_);

  // initialize IK solvers:
  source_ik_solvers_.resize(max_openmp_threads_);
  target_ik_solvers_.resize(max_openmp_threads_);
  for (int i=0; i<max_openmp_threads_; ++i)
  {
    source_ik_solvers_[i].reset(new ConstrainedIKSolver(node_handle, root, tip));
    target_ik_solvers_[i].reset(new ConstrainedIKSolver(node_handle, root, tip));
  }

  source_ik_solvers_initialized_ = false;
  target_ik_solvers_initialized_ = false;
}

IKWrapper::~IKWrapper()
{
}

void IKWrapper::setCostFunction(boost::shared_ptr<learnable_cost_function::CostFunction> cost_function)
{
  setSourceCostFunction(cost_function);
  setTargetCostFunction(cost_function);
}

void IKWrapper::setSourceCostFunction(boost::shared_ptr<learnable_cost_function::CostFunction> cost_function)
{
  for (int i=0; i<max_openmp_threads_; ++i)
  {
    source_ik_solvers_[i]->setCostFunction(cost_function);
  }
  source_ik_solvers_initialized_ = false;
}

void IKWrapper::setTargetCostFunction(boost::shared_ptr<learnable_cost_function::CostFunction> cost_function)
{
  for (int i=0; i<max_openmp_threads_; ++i)
  {
    target_ik_solvers_[i]->setCostFunction(cost_function);
  }
  target_ik_solvers_initialized_ = false;
}

void IKWrapper::setFKSolver(boost::shared_ptr<const FKSolver> fk_solver)
{
  for (int i=0; i<max_openmp_threads_; ++i)
  {
    source_ik_solvers_[i]->setFKSolver(fk_solver->clone());
    target_ik_solvers_[i]->setFKSolver(fk_solver->clone());
  }
  source_ik_solvers_initialized_ = false;
  target_ik_solvers_initialized_ = false;
}

void IKWrapper::useDefaultFKSolver()
{
  for (int i=0; i<max_openmp_threads_; ++i)
  {
    source_ik_solvers_[i]->useDefaultFKSolver();
    target_ik_solvers_[i]->useDefaultFKSolver();
  }
  source_ik_solvers_initialized_ = false;
  target_ik_solvers_initialized_ = false;
}

void IKWrapper::registerDebugCallback(boost::function<void (const KDL::JntArray& q)> f)
{
  for (int i=0; i<max_openmp_threads_; ++i)
  {
    source_ik_solvers_[i]->registerDebugCallback(f);
    target_ik_solvers_[i]->registerDebugCallback(f);
  }
}

bool IKWrapper::ik(const InverseKinematicsRequest& ik_request,
              std::vector<IKSolution>& good_solutions,
              IKSolution& best_solution,
              std::vector<boost::shared_ptr<ConstrainedIKSolver> >& ik_solvers,
              bool& ik_solvers_initialized)
{
  ros::WallTime start_time = ros::WallTime::now();

  std::vector<IKSolution> solutions;
  solutions.resize(max_random_attempts_);

  // prime the solvers if first-time (HACKY HACK HACK!!!)
  if (!ik_solvers_initialized)
  {
    for (int i=0; i<max_openmp_threads_; ++i)
    {
      ik_solvers[i]->getRandomJointAngles(random_seeds_[i]);
      ik_solvers[i]->ikLocal(ik_request, random_seeds_[i], solutions[0]);
    }
    ik_solvers_initialized = true;
  }

  // compute solutions parallelly
  int i, thread_id;
#pragma omp parallel for private(i, thread_id) schedule(dynamic)
  for (i=0; i<max_random_attempts_; ++i)
  {
    thread_id = omp_get_thread_num();
    //printf("attempt %d on thread %d\n", i, thread_id);
    ik_solvers[thread_id]->getRandomJointAngles(random_seeds_[thread_id]);
    //printf("attempt %d on thread %d - intermediate\n", i, thread_id);
    ik_solvers[thread_id]->ikLocal(ik_request, random_seeds_[thread_id], solutions[i]);
    //printf("finished attempt %d on thread %d\n", i, thread_id);
  }

  // find best solution
  best_solution.cost_function_value = std::numeric_limits<double>::max();
  best_solution.success = false;
  int num_success = 0;
  for (int i=0; i<max_random_attempts_; ++i)
  {
    if (!solutions[i].success)
      continue;
    ++num_success;
    good_solutions.push_back(solutions[i]);
    if (solutions[i].cost_function_value < best_solution.cost_function_value)
    {
      best_solution = solutions[i];
    }
  }

  ros::WallDuration duration = ros::WallTime::now() - start_time;
  ROS_INFO("IK Solver took %f millisecs (success = %d / %d)", duration.toSec()*1000.0,
           num_success, max_random_attempts_);

  return best_solution.success;
}

bool IKWrapper::ik(const InverseKinematicsRequest& ik_request,
              IKSolution& solution)
{
  std::vector<IKSolution> solutions;
  return ik(ik_request, solutions, solution);
}

bool IKWrapper::ik(const InverseKinematicsRequest& ik_request,
              std::vector<IKSolution>& solutions,
              IKSolution& best_solution)
{
  return ik(ik_request, solutions, best_solution, source_ik_solvers_, source_ik_solvers_initialized_);
}

bool IKWrapper::ikLocal(const InverseKinematicsRequest& ik_request,
           const KDL::JntArray& q_in,
           IKSolution& solution)
{
  return source_ik_solvers_[0]->ikLocal(ik_request, q_in, solution);
}

bool IKWrapper::ikPreGrasp(const InverseKinematicsRequest& ik_request,
                                     const std::vector<KDL::Frame>& grasp_to_pre_grasp_offsets,
                IKSolution& best_solution)
{
  std::vector<IKSolution> solutions;
  return ikPreGrasp(ik_request, grasp_to_pre_grasp_offsets, solutions, best_solution);
}

bool IKWrapper::ikPreGrasp(const InverseKinematicsRequest& ik_request,
                                     const std::vector<KDL::Frame>& grasp_to_pre_grasp_offsets,
                std::vector<IKSolution>& solutions,
                IKSolution& best_solution)
{
  return ikPreGrasp(ik_request, grasp_to_pre_grasp_offsets, solutions, best_solution, source_ik_solvers_, source_ik_solvers_initialized_);
}


bool IKWrapper::ikDualPreGrasp(const InverseKinematicsRequest& ik_request1,
                    const std::vector<KDL::Frame>& grasp_to_pre_grasp_offsets1,
                    IKSolution& best_solution1,
                    const InverseKinematicsRequest& ik_request2,
                    const std::vector<KDL::Frame>& grasp_to_pre_grasp_offsets2,
                    IKSolution& best_solution2)
{
  std::vector<IKSolution> solutions1;
  std::vector<IKSolution> solutions2;
  if (!ikPreGrasp(ik_request1, grasp_to_pre_grasp_offsets1, solutions1, best_solution1, source_ik_solvers_, source_ik_solvers_initialized_))
  {
    ROS_ERROR("Source IK failed");
    return false;
  }
  if (!ikPreGrasp(ik_request2, grasp_to_pre_grasp_offsets2, solutions2, best_solution2, target_ik_solvers_, target_ik_solvers_initialized_))
  {
    ROS_ERROR("Target IK failed");
    return false;
  }

  double best_solution_cost = std::numeric_limits<double>::max();

  // check all pairs and return lowest cost solution
  for (unsigned int i=0; i<solutions1.size(); ++i)
  {
    for (unsigned int j=0; j<solutions2.size(); ++j)
    {
      double cost = solutions1[i].getCostRecursive() + solutions2[j].getCostRecursive();
      cost += (dual_ik_cost_weights_.array() * (solutions1[i].joint_angles.data - solutions2[j].joint_angles.data).array()).matrix().norm();
      if (cost < best_solution_cost)
      {
        best_solution_cost = cost;
        best_solution1 = solutions1[i];
        best_solution2 = solutions2[j];
      }
    }
  }
  return true;
}

bool IKWrapper::ikPreGrasp(const InverseKinematicsRequest& ik_request,
                const std::vector<KDL::Frame>& grasp_to_pre_grasp_offsets,
                std::vector<IKSolution>& solutions,
                IKSolution& best_solution,
                std::vector<boost::shared_ptr<ConstrainedIKSolver> >& ik_solvers,
                bool& ik_solvers_initialized)
{
  if (grasp_to_pre_grasp_offsets.size() == 0)
    return ik(ik_request, solutions, best_solution);

  solutions.clear();

  // get all ik solutions
  std::vector<IKSolution> all_solutions;
  IKSolution all_best_solution;
  bool success = ik(ik_request, all_solutions, all_best_solution, ik_solvers, ik_solvers_initialized);
  if (!success)
  {
    return false;
  }

  best_solution.cost_function_value = std::numeric_limits<double>::max();
  best_solution.success = false;

  ros::WallTime start_time = ros::WallTime::now();

  // calculate pre-grasp feasibility parallelly
#pragma omp parallel for schedule(dynamic)
  for (unsigned int i=0; i<all_solutions.size(); ++i)
  {
    if (!all_solutions[i].success)
      continue;

    int thread_id = omp_get_thread_num();

    InverseKinematicsRequest pre_grasp_ik_request;
    IKSolution pre_grasp_solution;
    KDL::Frame pre_grasp_hand_pose;

    pre_grasp_ik_request.link_name = ik_request.link_name;
    pre_grasp_ik_request.use_position_constraint = false;
    pre_grasp_ik_request.use_orientation_constraint = false;
    pre_grasp_ik_request.link_to_tool_pose.orientation.w = 1.0;
    pre_grasp_ik_request.link_to_tool_pose.orientation.x = 0.0;
    pre_grasp_ik_request.link_to_tool_pose.orientation.y = 0.0;
    pre_grasp_ik_request.link_to_tool_pose.orientation.z = 0.0;
    pre_grasp_ik_request.link_to_tool_pose.position.x = 0.0;
    pre_grasp_ik_request.link_to_tool_pose.position.y = 0.0;
    pre_grasp_ik_request.link_to_tool_pose.position.z = 0.0;

    bool pre_grasps_successful = true;
    // test every pre-grasp
    for (unsigned int j=0; j<grasp_to_pre_grasp_offsets.size(); ++j)
    {
      // create the ik request
      pre_grasp_hand_pose = all_solutions[i].end_effector_frame * grasp_to_pre_grasp_offsets[j];
      kdlFrameToRosPose(pre_grasp_hand_pose, pre_grasp_ik_request.desired_tool_pose);

      // try the ik
      if (!ik_solvers[thread_id]->ikLocal(pre_grasp_ik_request, all_solutions[i].joint_angles, pre_grasp_solution))
      {
        pre_grasps_successful = false;
        break;
      }
      all_solutions[i].pre_grasp_solutions.push_back(pre_grasp_solution);
    }
    if (!pre_grasps_successful)
    {
      all_solutions[i].success = false;
      continue;
    }

  }

  // retrieve the best
  for (unsigned int i=0; i<all_solutions.size(); ++i)
  {
    if (!all_solutions[i].success)
      continue;

    if (all_solutions[i].cost_function_value < best_solution.cost_function_value)
    {
      best_solution = all_solutions[i];
    }
    solutions.push_back(all_solutions[i]);
  }

  if (!best_solution.success)
  {
    ROS_ERROR("IK failed due to pre-grasp!");
  }

  ros::WallDuration duration = ros::WallTime::now() - start_time;
  ROS_INFO("Pre-grasp checking took %f millisec", duration.toSec()*1000.0);

  return best_solution.success;
}

} /* namespace constrained_inverse_kinematics */
